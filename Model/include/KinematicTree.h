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

		bool update_state(const Eigen::VectorXf &jointPos,
		                  const Eigen::VectorXf &jointVel)
		{
			return update_state(jointPos,jointVel,this->_basePose,Eigen::VectorXf::Zero(6));
		}
		                  
		bool update_state(const Eigen::VectorXf          &jointPosition,
		                  const Eigen::VectorXf          &jointVelocity,
		                  const Pose                     &basePose,
		                  const Eigen::Matrix<float,6,1> &baseTwist);
		                  
		std::string name() const { return this->_name; }
		
		int number_of_joints() const { return this->numJoints; }
		                  
	private:
		
		// Properties
		
		int numJoints;

		Eigen::Matrix<float,6,1> _baseTwist;                                                // Linear & angular velocity of the base
			
		Eigen::VectorXf q, qdot;                                                            // Joint positions and velocities
		
		Eigen::Vector3f char_to_vector3f(const char* character);                            // Used to parse urdf properties
		
		Link* base = nullptr;                                                               // Pointer to the base of the tree
		
		Pose _basePose;                                                                     // As it says
		
		std::vector<Joint> joint;                                                           // Vector of joint objects
		
		std::vector<Link> link;                                                             // Vector of link objects
		
		std::map<std::string, unsigned int> jointDictionary;                                
		
		std::map<std::string, unsigned int> linkDictionary;
		
		std::string _name;                                                                  // The name of this robot
		
		// Methods
		
		Eigen::Matrix3f jacobian(Joint *joint,
		                         const Eigen::Vector3f &point,
		                         const unsigned int &numberOfJoints);
		                         
};                                                                                                  // Semicolon needed after class declarations

#endif
