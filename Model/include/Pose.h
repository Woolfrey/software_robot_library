/**
 * @file   Pose.h
 * @author Jon Woolfrey
 * @date   September 2023
 * @brief  A class representing the position and orientation of an object in 3D space.
 */

#ifndef POSE_H_
#define POSE_H_

#include <Eigen/Geometry>                                                                           // Quaternion<DataType>
#include <iostream>                                                                                 // std::cout

using namespace Eigen;                                                                              // Eigen::AngleAxis, Eigen::Matrix, Eigen::Quaternion

template <class DataType = float>
class Pose
{
	public:

		/**
		 * An empty constructor.
		 */
		Pose() {};
		
		/**
		 * A full constructor.
		 * @param translation A 3x1 vector for the position/translation component.
		 * @param quaternion A quaternion object defining the orientation.
		 */
		Pose(const Vector<DataType,3>   &translation,
		     const Quaternion<DataType> &quaternion) :
		     _translation(translation),
		     _quaternion(quaternion.normalized()) {}
		
		/**
		 * Computes the error between this pose object and another.
		 * @param desired The other pose for which to compute the error.
		 * @return A 6x1 vector containing the position error, and orientation error as the angle*axis
		 */
		Vector<DataType,6> error(const Pose &desired);
		
		/**
		 * @return Returns this pose as 4x4 homogeneous transformation / SE(3) matrix.
		 */
		Matrix<DataType,4,4> as_matrix();

		/**
		 * @return Returns the orientation component of the pose as a 3x3 rotation / SO(3) matrix
		 */
		Matrix<DataType,3,3> rotation() const { return this->_quaternion.toRotationMatrix(); }
		
		/**
		 * @return Returns the 3x1 translation component
		 */
		Vector<DataType,3> translation() const { return this->_translation; }
		
		/**
		 * @return Returns the orientation component as a quaternion object.
		 */
		Quaternion<DataType> quaternion() const { return this->_quaternion; }
		
		/**
		 * @return Computes the inverse / opposite of this pose.
		 */
		Pose<DataType> inverse();
		
		/**
		 * @return Returns the product of this pose with another.
		 */
		Pose<DataType> operator*(const Pose &other) const;
		
		/**
		 * @return Returns an in-place product of this pose and another.
		 */
		void operator*=(const Pose &other);
		
		/**
		 * @return Performs a point transformation.
		 */
		Vector<DataType,3> operator*(const Vector<DataType,3> &other);
		
	private:
	
		Vector<DataType,3> _translation = {0.0, 0.0, 0.0};                                  ///< The position or translation component
		
		Quaternion<DataType> _quaternion = {1.0, 0.0, 0.0, 0.0};                            ///< The orientation or rotation component
	
};                                                                                                  // Semicolon needed after class declaration

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                     Get the error between this pose and another one                           //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
Vector<DataType,6> Pose<DataType>::error(const Pose &desired)
{
	Vector<DataType,6> error;                                                                   // Value to be returned

	error.head(3) = desired.translation() - this->_translation;                                 // translation error
	
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
	T.block(0,3,3,1) = this->_translation;                                                         // Insert translation component
	
	T.row(3) << 0, 0, 0, 1;                                                                     // Assign the bottom row
	
	return T;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                      Get the inverse that "undoes" a rotation                                 //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
Pose<DataType> Pose<DataType>::inverse()
{
	return Pose(-this->_quaternion.toRotationMatrix()*this->_translation, this->_quaternion.inverse());
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                 Multiply two pose objects                                     //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
Pose<DataType> Pose<DataType>::operator* (const Pose &other) const
{
	return Pose(this->_translation + this->_quaternion.toRotationMatrix()*other.translation(),
	            this->_quaternion*other.quaternion());
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                Multiply this pose in place                                     //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
void Pose<DataType>::operator*= (const Pose &other)
{
	this->_translation += this->_quaternion.toRotationMatrix()*other.translation(),
	this->_quaternion  *= other.quaternion();
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                    Transform a vector                                         //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
Vector<DataType,3> Pose<DataType>::operator* (const Vector<DataType,3> &other)
{
	return this->_translation + this->_quaternion.toRotationMatrix()*other;
}

#endif
