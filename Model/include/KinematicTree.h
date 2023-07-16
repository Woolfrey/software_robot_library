    ///////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                               //
  //                      A class representing a rigid, multibody system                           //
 //                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef KINEMATICTREE_H_
#define KINEMATICTREE_H_

#include <fstream>                                                                                  // For loading files
#include <Joint.h>                                                                                  // Custom class for describing a moveable connection between links
#include <Link.h>                                                                                   // Custom class combining a rigid body and joint
#include <map>                                                                                      // std::map
#include <tinyxml2.h>                                                                               // For parsing urdf files

struct ReferenceFrame
{
	Link *link = nullptr;                                                                       // The link it is attached to
	Pose relativePose;                                                                          // The relative pose with respect to said link
};

class KinematicTree
{
	public:
		KinematicTree(const std::string &pathToURDF);                                       // Constructor from URDF

		// Methods
		
		bool update_state(const Eigen::VectorXf &jointPos,
		                  const Eigen::VectorXf &jointVel)
		{
			return update_state(jointPos,jointVel,this->_basePose,Eigen::VectorXf::Zero(6));
		}
		                  
		bool update_state(const Eigen::VectorXf          &jointPosition,
		                  const Eigen::VectorXf          &jointVelocity,
		                  const Pose                     &basePose,
		                  const Eigen::Matrix<float,6,1> &baseTwist);
		                  
		Eigen::MatrixXf time_derivative(const Eigen::MatrixXf &J);                          // Time derivative of a Jacobian
		
		Eigen::MatrixXf partial_derivative(const Eigen::MatrixXf &J,
		                                   const unsigned int &jointNumber);                // Partial derivative of a Jacobian
		
		int number_of_joints() const { return this->numJoints; }
		
		Pose frame_pose(const std::string &frameName);                                      // Return the pose for a reference frame on the robot
		
		std::string name() const { return this->_name; }                                    // Return the name of this robot
		
	private:
		
		// Properties
		
		int numJoints;

		Eigen::Matrix<float,6,1> _baseTwist;                                                // Linear & angular velocity of the base
			
		Eigen::VectorXf _jointPosition,
		                _jointVelocity;
		
		Eigen::Vector3f char_to_vector3f(const char* character);                            // Used to parse urdf properties
		
		Link* base = nullptr;                                                               // Pointer to the base of the tree
		
		Pose _basePose;                                                                     // As it says
		
		std::vector<Joint> joint;                                                           // Vector of joint objects
		
		std::vector<Link> link;                                                             // Vector of link objects
		
		std::map<std::string, ReferenceFrame> referenceFrameList;
		
		std::string _name;                                                                  // The name of this robot
		
		Eigen::MatrixXf jointInertiaMatrix,
		                jointCoriolisMatrix;
		                
		Eigen::VectorXf jointGravityVector;
		
		Eigen::Vector3f gravityVector = {0,0,-9.81};
		
		// Methods
		
		Eigen::MatrixXf jacobian(Joint *joint,                                              
		                         const Eigen::Vector3f &point,
		                         const unsigned int &numberOfColumns);	                         
};                                                                                                  // Semicolon needed after class declarations

#endif
