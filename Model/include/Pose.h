    ///////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                               //
  //            A class representing the position and orientation of an object in 3D space         //
 //                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef POSE_H_
#define POSE_H_

#include <Eigen/Geometry>                                                                           // Quaternion<DataType>
#include <iostream>                                                                                 // std::cout

using namespace Eigen;                                                                              // Eigen::AngleAxis, Eigen::Matrix, Eigen::Quaternion

template <class DataType = float>
class Pose
{
	public:
		// Constructor(s)
		Pose() {};
		
		Pose(const Vector<DataType,3> &position,
		     const Quaternion<DataType> &quaternion) :
		     _position(position),
		     _quaternion(quaternion.normalized()) {}
		
		// Functions
	
		Vector<DataType,6> error(const Pose &desired);                                    // Error from a desired pose
		
		Matrix<DataType,4,4> as_matrix();                                                   // Return a 4x4 Homogeneous Transform

		Matrix<DataType,3,3> rotation() const { return this->_quaternion.toRotationMatrix(); } // Return SO(3) matrix for rotation
		
		Vector<DataType,3> position() const { return this->_position; }                     // Return the translation vector
		
		Quaternion<DataType> quaternion() const { return this->_quaternion; }               // Return the quaternion object
		
		Pose<DataType> inverse();                                                           // Get the opposite of this pose
		
		Pose<DataType> operator*(const Pose &other);                                        // Multiply this Pose with another
		
		void operator*=(const Pose &other);
		
		Vector<DataType,3> operator*(const Vector<DataType,3> &other);                      // Transform a translation vector
		
	private:
	
		Vector<DataType,3> _position = {0.0, 0.0, 0.0};
		
		Quaternion<DataType> _quaternion = {1.0, 0.0, 0.0, 0.0};
	
};                                                                                                  // Semicolon needed after class declaration

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                     Get the error between this pose and another one                           //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
Vector<DataType,6> Pose<DataType>::error(const Pose &desired)
{
	Vector<DataType,6> error;                                                                   // Value to be returned

	error.head(3) = desired.position() - this->_position;                                       // Position error
	
	DataType angle = this->_quaternion.angularDistance(desired.quaternion());                   // Distance between vectors (i.e. dot product)
	
	Vector<DataType,3> temp = this->_quaternion.w()    * desired.quaternion().vec()
		                  - desired.quaternion().w() * this->_quaternion.vec()
		                  - desired.quaternion().vec().cross(this->_quaternion.vec()); 
	
	if(angle <= M_PI) error.tail(3) =  temp;
	else              error.tail(3) = -temp;                                                    // Spin the other way
	
	return error;
}
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //               Get the pose as a 4x4 homogeneous transformation matrix                          //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
Matrix<DataType,4,4> Pose<DataType>::as_matrix()
{
	Matrix<DataType,4,4> T;                                                                     // Value to be returned
	
	T.block(0,0,3,3) = this->_quaternion.toRotationMatrix();                                    // Convert to SO(3) and insert
	T.block(0,3,3,1) = this->_position;                                                         // Insert translation component
	
	T.row(3) << 0, 0, 0, 1;                                                                     // Assign the bottom row
	
	return T;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                      Get the inverse that "undoes" a rotation                                 //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
Pose<DataType> Pose<DataType>::inverse()
{
	return Pose(-this->_quaternion.toRotationMatrix()*this->_position, this->_quaternion.inverse());
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                 Multiply two pose objects                                     //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
Pose<DataType> Pose<DataType>::operator* (const Pose &other)
{
	return Pose(this->_position + this->_quaternion.toRotationMatrix()*other.position(),
	            this->_quaternion*other.quaternion());
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                Multiply this pose in place                                     //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
void Pose<DataType>::operator*= (const Pose &other)
{
	this->_position   += this->_quaternion.toRotationMatrix()*other.position(),
	this->_quaternion *= other.quaternion();
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                    Transform a vector                                         //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
Vector<DataType,3> Pose<DataType>::operator* (const Vector<DataType,3> &other)
{
	return this->_position + this->_quaternion.toRotationMatrix()*other;
}

#endif
