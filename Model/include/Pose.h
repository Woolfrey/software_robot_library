    ///////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                               //
  //            A class representing the position and orientation of an object in 3D space         //
 //                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef POSE_H_
#define POSE_H_

#include <Eigen/Geometry>                                                                           // Eigen::Quaternionf
#include <iostream>                                                                                 // std::cout

class Pose
{
	public:
		// Empty constructor, delegate to full constructor
		Pose() : Pose(Eigen::Vector3f::Zero(),
		              Eigen::Quaternionf(Eigen::AngleAxisf(0,Eigen::Vector3f::UnitX()))) {}
		
		// Full constructor
		Pose(const Eigen::Vector3f    &position,
		     const Eigen::Quaternionf &quaternion);
		
		// Functions
	
		Eigen::Matrix<float,6,1> error(const Pose &desired);                                // Error from a desired pose
		
		Eigen::Matrix<float,4,4> as_matrix();                                               // Return a 4x4 Homogeneous Transform

		Eigen::Matrix3f rotation_matrix() const { return this->_quat.toRotationMatrix(); }
		
		Eigen::Vector3f pos() const { return this->_pos; }
		
		Eigen::Quaternionf quat() const { return this->_quat; }
		
		Pose inverse();
		
		Pose operator*(const Pose &other);
		
		Eigen::Vector3f operator*(const Eigen::Vector3f &other);
		
	private:
	
		Eigen::Vector3f _pos = {0.0, 0.0, 0.0};
		
		Eigen::Quaternionf _quat = {1.0, 0.0, 0.0, 0.0};
	
};                                                                                                  // Semicolon needed after class declaration

#endif
