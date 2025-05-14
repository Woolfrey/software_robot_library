/**
 * @file    Pose2D.cpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    April 2025
 * @version 1.0
 * @brief   A class that describes the position & orientation of an object in 2D space.
 * 
 * @details This class describes the position of an object as a 2D vector, and the orientation
 *          using a scalar. Arithmetic can be used to propagate & invert these objects for
 *          performing transforms in 2D space.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */
 
#include <Model/Pose2D.h>

namespace RobotLibrary { namespace Model {

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                   Get this object as a 4x4 homogeneous transformation matrix                  //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix3d
Pose2D::as_matrix()
{
    Eigen::Matrix3d T;
    
    T << cos(_angle), -sin(_angle), _translation[0],
         sin(_angle),  cos(_angle), _translation[1],
                   0,            0,               1;
    
    return T;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                     Get the error between this pose and another one                           //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Vector3d
Pose2D::error(const Pose2D &desired)
{
     Eigen::Vector3d error;                                                                         // Value to be returned
     
     error.head(2) = desired.translation() - _translation;
     error.tail(1) = wrap_to_pi(desired.angle() - _angle);
     
     return error;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                      Get the inverse that "undoes" a rotation                                 //
///////////////////////////////////////////////////////////////////////////////////////////////////
Pose2D
Pose2D::inverse()
{
    return Pose2D(-rotation().transpose() * _translation(), -_angle);
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                 Multiply two pose objects                                     //
///////////////////////////////////////////////////////////////////////////////////////////////////
Pose2D
Pose2D::operator* (const Pose2D &other) const
{
    return Pose2D(_translation + rotation() * other.translation, wrap_to_pi(_angle + other.angle()));
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                Multiply this pose in place                                     //
////////////////////////////////////////////////////////////////////////////////////////////////////
void
Pose2D::operator*= (const Pose2D &other)
{
    _translation += rotation() * other.translation();
    _angle = wrap_to_pi(_angle + other.angle());
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                    Transform a vector                                         //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Vector2d
Pose2D::operator* (const Eigen::Vector2d &other)
{
    return _translation + rotation() * other;
}

} } // namespace
