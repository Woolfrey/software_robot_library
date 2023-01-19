    ///////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                               //
  //            A class representing the position and orientation of an object in 3D space         //
 //                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef POSE_H_
#define POSE_H_

#include <Eigen/Geometry>

class Pose
{
	public:

		Pose(const Eigen::Vector3f &position, const Eigen::Quaternionf &quaternion);        // Constructor
		
		// Functions
		Eigen::Matrix<float,6,1> get_pose_error(const Pose &desired);                       // Error from a desired pose
		
		Eigen::Matrix<float,4,4> get_matrix();                                              // Return a 4x4 Homogeneous Transform
		
		Pose operator*(const Pose &other);
		
		// Variables
		Eigen::Vector3f    pos;
		Eigen::Quaternionf quat;
		
	private:
	
};                                                                                                  // Semicolon needed after class declaration

#endif
