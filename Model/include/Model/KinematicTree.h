/**
 * @file    KinematicTree.h
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    February 2025
 * @version 1.0
 * @brief   A class for a multi rigid body system of branching serial link structures.
 * 
 * @details This class is used to compute the kinematics and dynamics of branching, serial link structures.
 *          It presumes only open-chain branches. It computes forward kinematics, and inverse dynamics.
 *          It is designed to be embedded in to a control class to obtain things like the Jacobian,
 *          inertia matrix, Coriolis matrix, etc.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */
 
#ifndef KINEMATICTREE_H_
#define KINEMATICTREE_H_

#include <Model/Joint.h>                                                                            // Custom class for describing a moveable connection between links
#include <Model/Link.h>                                                                             // Custom class combining a rigid body and joint
#include <Math/SkewSymmetric.h>                                                                     // Custom class

#include <fstream>                                                                                  // For loading files
#include <map>                                                                                      // std::map
#include <tinyxml2.h>                                                                               // For parsing urdf files

namespace RobotLibrary { namespace Model {

/**
 * @brief A structure containing necessary information for defining a reference frame on a kinematic tree.
 */
struct ReferenceFrame
{
     Link *link = nullptr;                                                                          ///< The link it is attached to
     Pose relativePose;                                                                             ///< Pose with respect to local link frame
};

/**
 * @brief A class that defines the kinematics and dynamics of branching, serial link structures.
 */
class KinematicTree
{
     public:
          /**
           * @brief Constructor for a kinematic tree.
           * @param pathToURDF The location of a URDF file that specifies are robot structure.
           */
          KinematicTree(const std::string &pathToURDF);                                             // Constructor from URDF
          
          /**
           * @brief Updates the forward kinematics and inverse dynamics. Used for fixed base structures.
           * @param jointPosition A vector of the joint positions.
           * @param jointVelocity A vector of the joint velocities.
           * @return Returns false if there is a problem.
           */
          bool
          update_state(const Eigen::VectorXd &jointPosition,
                       const Eigen::VectorXd &jointVelocity)
          {
               return update_state(jointPosition, jointVelocity, this->base.pose(), Eigen::Vector<double,6>::Zero());
          }
          
          /**
           * @brief Updates the forward kinematics and inverse dynamics. Used for floating base structures.
           * @param jointPosition A vector of all the joint positions.
           * @param jointVelocity A vector of all the joint velocities.
           * @param basePose The transform of the base relative to some global reference frame.
           * @param baseTwist The velocity of the base relative to some global reference frame.
           */
          bool
          update_state(const Eigen::VectorXd         &jointPosition,
                       const Eigen::VectorXd         &jointVelocity,
                       const Pose                    &basePose,
                       const Eigen::Vector<double,6> &baseTwist);

          /**
           * @brief Query how many controllable joints there are in this model.
           * @return Returns what you asked for.
           */
          unsigned int
          number_of_joints() const { return this->_numberOfJoints; }
          
          /**
           * @brief Get the coupled inertia matrix between the actuated joints and the base.
           * @return An nx6 Eigen::Matrix object.
           */
          Eigen::Matrix<double, Eigen::Dynamic, 6>
          joint_base_inertia_matrix() const { return this->_jointBaseInertiaMatrix; }
          
          /**
           * @brief Get the coupled inertia matrix between the base and actuated joints.
           * @return A 6xn Eigen::Matrix object.
           */
          Eigen::Matrix<double, 6, Eigen::Dynamic>
          base_joint_inertia_matrix() const { return this->_jointBaseInertiaMatrix.transpose(); }
          
          /**
           * @brief Get the Coriolis matrix pertaining to coupled inertia between the actuated joints and base.
           * @return An nx6 Eigen::Matrix object.
           */
          Eigen::Matrix<double, Eigen::Dynamic, 6>
          joint_base_coriolis_matrix() const { return this->_jointBaseCoriolisMatrix; }
          
          /**
           * @brief Get the Coriolis matrix pertaining to coupled inertia between the base and actuated joints.
           * @return A 6xn Eigen::Matrix object.
           */
          Eigen::Matrix<double, 6, Eigen::Dynamic>
          base_joint_coriolis_matrix() const { return -this->_jointBaseCoriolisMatrix.transpose(); }
          
          /**
           * @brief Get the inertia matrix in the joint space of the model / robot.
           * @return Returns an nxn Eigen::Matrix object.
           */            
          Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>
          joint_inertia_matrix() const { return this->_jointInertiaMatrix; }
          
          /**
           * @brief Get the matrix pertaining to centripetal and Coriolis torques in the joints of the model.
           * @return Returns an nxn Eigen::Matrix object.
           */
          Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>
          joint_coriolis_matrix() const { return this->_jointCoriolisMatrix; } 

          /**
           * @brief Get the matrix that maps joint motion to Cartesian motion of the specified frame.
           * @return Returns a 6xn Eigen::Matrix object.
           */
          Eigen::Matrix<double, 6, Eigen::Dynamic>
          jacobian(const std::string &frameName); 

          /**
           * @brief Compute the time derivative for a given Jacobian matrix.
           * @param J The Jacobian for which to take the time derivative.
           * @return A 6xn matrix for the time derivative.
           */
          Eigen::Matrix<double, 6, Eigen::Dynamic>
          time_derivative(const Eigen::Matrix<double,6,Eigen::Dynamic> &jacobianMatrix);
          
          /**
           * @brief Compute the partial derivative for a Jacobian with respect to a given joint.
           * @param J The Jacobian with which to take the derivative
           * @param jointNumber The joint (link) number for which to take the derivative.
           * @return A 6xn matrix for the partial derivative of the Jacobian.
           */
          Eigen::Matrix<double, 6, Eigen::Dynamic>
          partial_derivative(const Eigen::Matrix<double,6,Eigen::Dynamic> &jacobianMatrix,
                             const unsigned int &jointNumber);

          /**
           * @brief Get the pose of a specified reference frame on the kinematic tree.
           * @return Returns a RobotLibrary::Pose object.
           */
          Pose
          frame_pose(const std::string &frameName);
          
          /**
           * @brief Get the joint torques from viscous friction.
           * @return An nx1 Eigen::Vector object
           */
          Eigen::Vector<double,Eigen::Dynamic>
          joint_damping_vector() const { return this->_jointDampingVector; }
               
          /**
           * @brief Get the joint torques needed to oppose gravitational acceleration.
           * @return Returns an nx1 Eigen::Vector object.
           */   
          Eigen::VectorXd
          joint_gravity_vector() const { return this->_jointGravityVector; }
          
          /** 
           * @brief Get the current joint velocities of all the joints in the model.
           * @return Returns an nx1 Eigen::Vector object.
           */
          Eigen::VectorXd
          joint_velocities() const { return this->_jointVelocity; }
          
          /**
           * @brief Get the name of this model.
           * @return Returns a std::string object.
           */
          std::string
          name() const { return this->_name; }
          
          /**
           * @brief Retrieve the name of the base link.
           * @return A std::string object.
           */
          std::string
          base_name() const { return this->_baseName; }
          
          /**
           * @brief Returns a pointer to a reference frame on this model.
           * @param name In the URDF, the name of the parent link attached to a fixed joint
           * @return A ReferenceFrame data structure.
           */
          ReferenceFrame*
          find_frame(const std::string &frameName);
          
          /**
           * @brief Compute a matrix that relates joint motion to Cartesian motion for a frame on the robot.
           * @param frame A pointer to the reference frame on the model.
           * @return A 6xn Eigen::Matrix object.
           */
          Eigen::Matrix<double,6,Eigen::Dynamic>
          jacobian(ReferenceFrame *frame);
          
          /**
           * @brief Get the joint position vector in the underlying model.
           * @return An nx1 Eigen::Vector object of all the joint positions.
           */
          Eigen::Vector<double,Eigen::Dynamic>
          joint_positions() const { return this->_jointPosition; }
          
          /**
           * @brief Return a pointer to a link on the structure.
           * @param The number of the link in the model.
           * @return A RobotLibrary::Link object
           */
          Link*
          link(const unsigned int &linkNumber);
          
          /**
           * @brief Access a numbered joint on the kinematic tree.
           * @param number The number for the joint in the model.
           * @return A pointer to a RobotLibrary::Joint object
           */
          Joint
          joint(const unsigned int &jointNumber) { return link(jointNumber)->joint(); }

          RigidBody base;                                                                           ///< Specifies the dynamics for the base.
          
     private:
          
          Eigen::Matrix<double,Eigen::Dynamic,6> _jointBaseCoriolisMatrix;                          ///< Inertial coupling between base and links
          
          Eigen::Matrix<double,Eigen::Dynamic,6> _jointBaseInertiaMatrix;                           ///< Inertial coupling between base and links
          
          Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> _jointCoriolisMatrix;                 ///< As it says on the label.
          
          Eigen::Vector<double,Eigen::Dynamic> _jointDampingVector;                                 ///< From viscous friction in the joints
          
          Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> _jointInertiaMatrix;                  ///< As it says on the label.

          Eigen::Vector3d _gravityVector = {0,0,-9.81};                                             ///< 3x1 vector for the gravitational acceleration.
               
          Eigen::Vector<double,Eigen::Dynamic> _jointPosition;                                      ///< A vector of all the joint positions.

          Eigen::Vector<double,Eigen::Dynamic> _jointVelocity;                                      ///< A vector of all the joint velocities.

          Eigen::Vector<double,Eigen::Dynamic> _jointGravityVector;                                 ///< A vector of all the gravitational joint torques.
           
          std::map<std::string, ReferenceFrame> _frameList;                                         ///< A dictionary of reference frames on the kinematic tree.
          
          std::vector<Link> _fullLinkList;                                                          ///< An array of all the links in the model, including fixed joints.
          
          std::vector<Link*> _link;                                                                 ///< An array of all the actuated links.
          
          std::vector<Link*> _baseLinks;                                                            ///< Array of links attached directly to the base.
          
          std::string _name;                                                                        ///< A unique name for this model.
          
          std::string _baseName;                                                                    ///< Name of the base link
          
          unsigned int _numberOfJoints;                                                             ///< The number of actuated joint in the kinematic tree.
                  
          /**
           * @brief Computes the Jacobian to a given point on a given link.
           * @param link A pointer to the link for the Jacobian
           * @param point A point relative to the link with which to compute the Jacobian
           * @param numberOfColumns Number of columns for the Jacobian (can be used to speed up calcs)
           * @return A 6xn Jacobian matrix.
           */
          Eigen::Matrix<double,6,Eigen::Dynamic>
          jacobian(Link *link,
                   const Eigen::Vector3d &point,
                   const unsigned int &numberOfColumns);

          /**
           * @brief Converts a char array to a 3x1 vector. Used in the constructor.
           * @param character A char array
           * @return Returns a 3x1 Eigen vector object.
           */
          Eigen::Vector3d char_to_vector(const char* character);                      
};                                                                                                  // Semicolon needed after class declarations

} }                                                                                                 // End namespace

#endif
