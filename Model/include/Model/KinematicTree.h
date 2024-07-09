/**
 * @file   KinematicTree.h
 * @author Jon Woolfrey
 * @date   September 2023
 * @brief  A class representing multiple rigid bodies connected in series by actuated joints.
 */
 
#ifndef KINEMATICTREE_H_
#define KINEMATICTREE_H_

#include <fstream>                                                                                  // For loading files
#include <Joint.h>                                                                                  // Custom class for describing a moveable connection between links
#include <Link.h>                                                                                   // Custom class combining a rigid body and joint
#include <map>                                                                                      // map
#include <Math/SkewSymmetric.h>                                                                     // Custom class
#include <tinyxml2.h>                                                                               // For parsing urdf files

template <typename DataType>
struct ReferenceFrame
{
     Link<DataType> *link = nullptr;                                                                // The link it is attached to
     Pose<DataType> relativePose;                                                                   // Pose with respect to local link frame
};

template <class DataType>
class KinematicTree
{
     public:
          /**
           * Constructor for a kinematic tree.
           * @param pathToURDF The location of a URDF file that specifies are robot structure.
           */
          KinematicTree(const std::string &pathToURDF);                                             // Constructor from URDF
          
          /**
           * Updates the forward kinematics and inverse dynamics. Used for fixed base structures.
           * @param jointPosition A vector of the joint positions.
           * @param jointVelocity A vector of the joint velocities.
           * @return Returns false if there is a problem.
           */
          inline
          bool
          update_state(const Eigen::Vector<DataType, Eigen::Dynamic> &jointPosition,
                       const Eigen::Vector<DataType, Eigen::Dynamic> &jointVelocity)
          {
               return update_state(jointPosition, jointVelocity, this->base.pose(), Eigen::Vector<DataType,6>::Zero());
          }
          
          /**
           * Updates the forward kinematics and inverse dynamics. Used for floating base structures.
           * @param jointPosition A vector of all the joint positions.
           * @param jointVelocity A vector of all the joint velocities.
           * @param basePose The transform of the base relative to some global reference frame.
           * @param baseTwist The velocity of the base relative to some global reference frame.
           */
          inline
          bool
          update_state(const Eigen::Vector<DataType, Eigen::Dynamic> &jointPosition,
                       const Eigen::Vector<DataType, Eigen::Dynamic> &jointVelocity,
                       const Pose<DataType>                          &basePose,
                       const Eigen::Vector<DataType, 6>              &baseTwist);

          /**
           * Query how many controllable joints there are in this model.
           * @return Returns what you asked for.
           */
          inline
          unsigned int
          number_of_joints() const { return this->_numberOfJoints; }
          
          /**
           * Get the coupled inertia matrix between the actuated joints and the base.
           * @return An nx6 Eigen::Matrix object.
           */
          inline
          Eigen::Matrix<DataType, Eigen::Dynamic, 6>
          joint_base_inertia_matrix() const { return this->_jointBaseInertiaMatrix; }
          
          /**
           * Get the coupled inertia matrix between the base and actuated joints.
           * @return A 6xn Eigen::Matrix object.
           */
          inline
          Eigen::Matrix<DataType,6,Eigen::Dynamic>
          base_joint_inertia_matrix() const { return this->_jointBaseInertiaMatrix.transpose(); }
          
          /**
           * Get the Coriolis matrix pertaining to coupled inertia between the actuated joints and base.
           * @return An nx6 Eigen::Matrix object.
           */
          inline
          Eigen::Matrix<DataType, Eigen::Dynamic, 6>
          joint_base_coriolis_matrix() const { return this->_jointBaseCoriolisMatrix; }
          
          /**
           * Get the Coriolis matrix pertaining to coupled inertia between the base and actuated joints.
           * @return A 6xn Eigen::Matrix object.
           */
          inline
          Eigen::Matrix<DataType,6,Eigen::Dynamic>
          base_joint_coriolis_matrix() const { return -this->_jointBaseCoriolisMatrix.transpose(); }
          
          /**
           * Get the inertia matrix in the joint space of the model / robot.
           * @return Returns an nxn Eigen::Matrix object.
           */     
          inline        
          Eigen::Matrix<DataType, Eigen::Dynamic, Eigen::Dynamic>
          joint_inertia_matrix() const { return this->_jointInertiaMatrix; }
          
          /**
           * Get the matrix pertaining to centripetal and Coriolis torques in the joints of the model.
           * @return Returns an nxn Eigen::Matrix object.
           */
          Eigen::Matrix<DataType, Eigen::Dynamic, Eigen::Dynamic>
          joint_coriolis_matrix() const { return this->_jointCoriolisMatrix; } 

          /**
           * Get the matrix that maps joint motion to Cartesian motion of the specified frame.
           * @return Returns a 6xn Eigen::Matrix object.
           */
          inline
          Eigen::Matrix<DataType, 6, Eigen::Dynamic>
          jacobian(const std::string &frameName); 

          /**
           * Compute the time derivative for a given Jacobian matrix.
           * @param J The Jacobian for which to take the time derivative.
           * @return A 6xn matrix for the time derivative.
           */
          inline
          Eigen::Matrix<DataType, 6, Eigen::Dynamic>
          time_derivative(const Eigen::Matrix<DataType,6,Eigen::Dynamic> &jacobianMatrix);
          
          /**
           * Compute the partial derivative for a Jacobian with respect to a given joint.
           * @param J The Jacobian with which to take the derivative
           * @param jointNumber The joint (link) number for which to take the derivative.
           * @return A 6xn matrix for the partial derivative of the Jacobian.
           */
          inline
          Eigen::Matrix<DataType, 6, Eigen::Dynamic>
          partial_derivative(const Eigen::Matrix<DataType,6,Eigen::Dynamic> &jacobianMatrix,
                             const unsigned int &jointNumber);

          /**
           * Get the pose of a specified reference frame on the kinematic tree.
           * @return Returns a RobotLibrary::Pose object.
           */
          inline
          Pose<DataType>
          frame_pose(const std::string &frameName);
          
          /**
           * Get the joint torques from viscous friction.
           * @return An nx1 Eigen::Vector object
           */
          inline
          Eigen::Vector<DataType,Eigen::Dynamic>
          joint_damping_vector() const { return this->_jointDampingVector; }
               
          /**
           * Get the joint torques needed to oppose gravitational acceleration.
           * @return Returns an nx1 Eigen::Vector object.
           */
          inline   
          Eigen::Vector<DataType, Eigen::Dynamic>
          joint_gravity_vector() const { return this->_jointGravityVector; }
          
          /** 
           * Get the current joint velocities of all the joints in the model.
           * @return Returns an nx1 Eigen::Vector object.
           */
          inline
          Eigen::Vector<DataType, Eigen::Dynamic>
          joint_velocities() const { return this->_jointVelocity; }
          
          /**
           * Get the name of this model.
           * @return Returns a std::string object.
           */
          inline
          std::string
          name() const { return this->_name; }
          
          /**
           * Returns a pointer to a reference frame on this model.
           * @param name In the URDF, the name of the parent link attached to a fixed joint
           * @return A ReferenceFrame data structure.
           */
          inline
          ReferenceFrame<DataType>*
          find_frame(const std::string &frameName)
          {
               auto container = this->_frameList.find(frameName);                                   // Find the frame in the list
               
               if(container != this->_frameList.end()) return &container->second;                   // Return the ReferenceFrame struct
               else
               {    
                    std::string message = "[ERROR] [KINEMATIC TREE] find_frame(): "
                                          "Unable to find '" + frameName + "' in the model. "
                                          "Here is a list of known reference frames:";
                    
                    for(auto frame = this->_frameList.begin(); frame != this->_frameList.end(); frame++)
                    {
                         message += "\n- " + frame->first;
                    }
                    
                    throw std::runtime_error(message);
               }
          }
          
          /**
           * Compute a matrix that relates joint motion to Cartesian motion for a frame on the robot.
           * @param frame A pointer to the reference frame on the model.
           * @return A 6xn Eigen::Matrix object.
           */
          inline
          Eigen::Matrix<DataType,6,Eigen::Dynamic>
          jacobian(ReferenceFrame<DataType> *frame)
          {
               if(frame == nullptr)
               {
                    throw std::runtime_error("[ERROR] [KINEMATIC TREE] jacobian(): "
                                             "Pointer to reference frame was empty.");
               }
               
               else
               {
                    return jacobian(frame->link,
                                   (frame->link->pose()*frame->relativePose).translation(),
                                    this->_numberOfJoints);
               }
          }
          
          /**
           * Get the joint position vector in the underlying model.
           * @return An nx1 Eigen::Vector object of all the joint positions.
           */
          inline
          Eigen::Vector<DataType,Eigen::Dynamic>
          joint_positions() const { return this->_jointPosition; }
          
          /**
           * Return a pointer to a link on the structure.
           * @param The number of the link in the model.
           * @return A RobotLibrary::Link object
           */
          inline
          Link<DataType>*
          link(const unsigned int &linkNumber)
          {
               if(linkNumber >= this->_link.size())
               {
                    throw std::runtime_error("[ERROR] [KINEMATIC TREE] link(): "
                                             "There are only " + std::to_string(this->_link.size()) + " in this model, "
                                             "but you tried to access link " + std::to_string(linkNumber+1) + ".");
               }
               return this->_link[linkNumber];
          }
          
          /**
           * @param number The number for the joint in the model.
           * @return A pointer to a RobotLibrary::Joint object
           */
          inline
          Joint<DataType>
          joint(const unsigned int &jointNumber)
          {
               return link(jointNumber)->joint();
          }

          RigidBody<DataType> base;                                                                 ///< Specifies the dynamics for the base.
          
     private:
          
          Eigen::Matrix<DataType,Eigen::Dynamic,6> _jointBaseCoriolisMatrix;                        ///< Inertial coupling between base and links
          
          Eigen::Matrix<DataType,Eigen::Dynamic,6> _jointBaseInertiaMatrix;                         ///< Inertial coupling between base and links
          
          Eigen::Matrix<DataType,Eigen::Dynamic,Eigen::Dynamic> _jointCoriolisMatrix;               ///< As it says on the label.
          
          Eigen::Vector<DataType,Eigen::Dynamic> _jointDampingVector;                               ///< From viscous friction in the joints
          
          Eigen::Matrix<DataType,Eigen::Dynamic,Eigen::Dynamic> _jointInertiaMatrix;                ///< As it says on the label.

          Eigen::Vector<DataType, 3> _gravityVector = {0,0,-9.81};                                  ///< 3x1 vector for the gravitational acceleration.
               
          Eigen::Vector<DataType,Eigen::Dynamic> _jointPosition;                                    ///< A vector of all the joint positions.

          Eigen::Vector<DataType,Eigen::Dynamic> _jointVelocity;                                    ///< A vector of all the joint velocities.

          Eigen::Vector<DataType,Eigen::Dynamic> _jointGravityVector;                               ///< A vector of all the gravitational joint torques.
           
          std::map<std::string, ReferenceFrame<DataType>> _frameList;                               ///< A dictionary of reference frames on the kinematic tree.
          
          std::vector<Link<DataType>> _fullLinkList;                                                ///< An array of all the links in the model, including fixed joints.
          
          std::vector<Link<DataType>*> _link;                                                       ///< An array of all the actuated links.
          
          std::vector<Link<DataType>*> _baseLinks;                                                  ///< Array of links attached directly to the base.
          
          std::string _name;                                                                        ///< A unique name for this model.
          
          unsigned int _numberOfJoints;                                                             ///< The number of actuated joint in the kinematic tree.
                  
          /**
           * Computes the Jacobian to a given point on a given link.
           * @param link A pointer to the link for the Jacobian
           * @param point A point relative to the link with which to compute the Jacobian
           * @param numberOfColumns Number of columns for the Jacobian (can be used to speed up calcs)
           * @return A 6xn Jacobian matrix.
           */
          Eigen::Matrix<DataType,6,Eigen::Dynamic>
          jacobian(Link<DataType> *link,
                   const Eigen::Vector<DataType,3> &point,
                   const unsigned int &numberOfColumns);

          /**
           * Converts a char array to a 3x1 vector. Used in the constructor.
           * @param character A char array
           * @return Returns a 3x1 Eigen vector object.
           */
          Eigen::Vector<DataType,3> char_to_vector(const char* character);  
                       
};                                                                                                  // Semicolon needed after class declarations

#endif
