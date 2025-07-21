/**
 * @file    Pose.cpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    July 2025
 * @version 1.1
 * @brief   A class that describes the position & orientation of an object in 3D space.
 * 
 * @details This class describes the position of an object as a 3D vector, and the orientation
 *          using a quaternion. Arithmetic can be used to propagate & invert these objects for
 *          performing transforms in 3D space.
 *
 * @update  June 2025 - Fixed quaternion unwinding in Pose::error() by enforcing hemisphere alignment.
 * 
 * @copyright (c) 2025 Jon Woolfrey
 *
 * @license   OSCL - Free for non-commercial open-source use only.
 *            Commercial use requires a license.
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */
 
#include <Model/Pose.h>

namespace RobotLibrary { namespace Model {

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                   Get this object as a 4x4 homogeneous transformation matrix                  //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4d
Pose::as_matrix()
{
    Eigen::Matrix4d T;
    
    T.block(0,0,3,3) = _quaternion.toRotationMatrix();
    T.block(0,3,3,1) = _translation;
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

     error.head(3) = desired.translation() - _translation;                                          // translation error

     Eigen::Quaterniond orientationError = (desired.quaternion() * _quaternion.inverse()).normalized();
     
     double angle = 2 * acos(std::clamp(orientationError.w(), -1.0, 1.0));
     
     if (angle < 1e-04) error.tail(3).setZero();
     else               error.tail(3) = angle * orientationError.vec().normalized();
     
     return error;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                      Get the inverse that "undoes" a rotation                                 //
///////////////////////////////////////////////////////////////////////////////////////////////////
Pose
Pose::inverse()
{
     return Pose(-_quaternion.toRotationMatrix() * _translation, _quaternion.inverse());
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                 Multiply two pose objects                                     //
///////////////////////////////////////////////////////////////////////////////////////////////////
Pose
Pose::operator* (const Pose &other) const
{
     
     return Pose(_translation + _quaternion.toRotationMatrix()*other.translation(),
                (_quaternion * other.quaternion()).normalized());
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                Multiply this pose in place                                     //
////////////////////////////////////////////////////////////////////////////////////////////////////
void
Pose::operator*= (const Pose &other)
{
     _translation += _quaternion.toRotationMatrix() * other.translation();
     _quaternion  *= other.quaternion();
     _quaternion.normalize();
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                    Transform a vector                                         //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Vector<double,3>
Pose::operator* (const Eigen::Vector<double,3> &other)
{
     return _translation + _quaternion.toRotationMatrix() * other;
}

} } // namespace
