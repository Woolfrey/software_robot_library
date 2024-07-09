/**
 * @file   Pose.cpp
 * @author Jon Woolfrey
 * @date   July 2024
 * @brief  Source files for the Pose class.
 */

#include <Pose.h>

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                     Get the error between this pose and another one                           //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
Eigen::Vector<DataType,6> Pose<DataType>::error(const Pose<DataType> &desired)
{
     Eigen::Vector<DataType,6> error;                                                               // Value to be returned

     error.head(3) = desired.translation() - this->_translation;                                    // translation error

     // Compute the vector component of the quaternion error
     Eigen::Vector<DataType,3> vec = this->_quaternion.w() * desired.quaternion().vec()
                                    - desired.quaternion().w() * this->_quaternion.vec()
                                    - desired.quaternion().vec().cross(this->_quaternion.vec());
     
     DataType angle = this->_quaternion.angularDistance(desired.quaternion());                      // Angle between the 2 vectors
     
     if(angle <= M_PI) error.tail(3) =  vec;
     else              error.tail(3) = -vec;                                                        // Angle > 180 degrees, spin opposite (shortest) direction
     
     return error;
}
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //               Get the pose as a 4x4 homogeneous transformation matrix                          //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
Eigen::Matrix<DataType,4,4> Pose<DataType>::as_matrix()
{
     Eigen::Matrix<DataType,4,4> T;                                                                 // Value to be returned
     
     T.block(0,0,3,3) = this->_quaternion.toRotationMatrix();                                       // Convert to SO(3) and insert
     T.block(0,3,3,1) = this->_translation;                                                         // Insert translation component
     
     T.row(3) << 0, 0, 0, 1;                                                                        // Assign the bottom row
     
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
Eigen::Vector<DataType,3> Pose<DataType>::operator* (const Eigen::Vector<DataType,3> &other)
{
     return this->_translation + this->_quaternion.toRotationMatrix()*other;
}
