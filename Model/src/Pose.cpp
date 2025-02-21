/**
 * @file    Pose.cpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    February 2025
 * @version 1.0
 * @brief   A class that describes the position & orientation of an object in 3D space.
 * 
 * @details This class describes the position of an object as a 3D vector, and the orientation
 *          using a quaternion. Arithmetic can be used to propagate & invert these objects for
 *          performing transforms in 3D space.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */
 
#include "Pose.h"

namespace RobotLibrary { namespace Model {

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                   Get this object as a 4x4 homogeneous transformation matrix                  //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4d
Pose::as_matrix()
{
    Eigen::Matrix4d T;
    
    T.block(0,0,3,3) = this->_quaternion.toRotationMatrix();
    T.block(0,3,3,1) = this->_translation;
    T.row(3) << 0, 0, 0, 1;
    
    return T;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                     Get the error between this pose and another one                           //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Vector<double,6>
Pose::error(const Pose &desired)
{
     Eigen::Vector<double,6> error;                                                                 // Value to be returned

     error.head(3) = desired.translation() - this->_translation;                                    // translation error

     // Compute the vector component of the quaternion error
     Eigen::Vector<double,3> vec = this->_quaternion.w() * desired.quaternion().vec()
                                    - desired.quaternion().w() * this->_quaternion.vec()
                                    - desired.quaternion().vec().cross(this->_quaternion.vec());
     
     double angle = this->_quaternion.angularDistance(desired.quaternion());                        // Angle between the 2 vectors
     
     if(angle <= M_PI) error.tail(3) =  vec;
     else              error.tail(3) = -vec;                                                        // Angle > 180 degrees, spin opposite (shortest) direction
     
     return error;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                      Get the inverse that "undoes" a rotation                                 //
///////////////////////////////////////////////////////////////////////////////////////////////////
Pose
Pose::inverse()
{
     return Pose(-this->_quaternion.toRotationMatrix()*this->_translation, this->_quaternion.inverse());
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                 Multiply two pose objects                                     //
///////////////////////////////////////////////////////////////////////////////////////////////////
Pose
Pose::operator* (const Pose &other) const
{
     return Pose(this->_translation + this->_quaternion.toRotationMatrix()*other.translation(),
                 this->_quaternion*other.quaternion());
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                Multiply this pose in place                                     //
////////////////////////////////////////////////////////////////////////////////////////////////////
void
Pose::operator*= (const Pose &other)
{
     this->_translation += this->_quaternion.toRotationMatrix()*other.translation(),
     this->_quaternion  *= other.quaternion();
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                    Transform a vector                                         //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Vector<double,3>
Pose::operator* (const Eigen::Vector<double,3> &other)
{
     return this->_translation + this->_quaternion.toRotationMatrix()*other;
}

} }
