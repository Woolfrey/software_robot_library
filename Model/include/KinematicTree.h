    ///////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                               //
  //                      A class representing a rigid, multibody system                           //
 //                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef KINEMATICTREE_H_
#define KINEMATICTREE_H_

#include <fstream>                                                                                  // For loading files
#include <Joint.h>
#include <Link.h>
#include <map>                                                                                      // std::map
#include <tinyxml2.h>                                                                               // For parsing urdf files

class KinematicTree
{
	public:
		KinematicTree(const std::string &pathToURDF);                                       // Constructor from URDF

		bool update_state(const Eigen::VectorXf &jointPos,
		                  const Eigen::VectorXf &jointVel)
		{
			return update_state(jointPos,jointVel,this->_basePose,Eigen::VectorXf::Zero(6));
		}
		                  
		bool update_state(const Eigen::VectorXf          &jointPos,
		                  const Eigen::VectorXf          &jointVel,
		                  const Pose                     &basePose,
		                  const Eigen::Matrix<float,6,1> &baseTwist);
		                  
		std::string name() const { return this->_name; }
		
		int number_of_joints() const { return this->numJoints; }
		                  
	private:
		
		int numJoints;
		
		Eigen::VectorXf q, qdot;                                                            // Joint positions and velocities

		std::map<std::string, Joint> jointList;
		
		std::map<std::string, Link> linkList;
		
		Eigen::Vector3f char_to_vector3f(const char* character);                            // Used to parse urdf properties
		
		Link* base = nullptr;                                                               // Pointer to the base of the tree
		
		Pose _basePose;

		Eigen::Matrix<float,6,1> _baseTwist;

		std::string _name;
};                                                                                                  // Semicolon needed after class declarations

#endif
