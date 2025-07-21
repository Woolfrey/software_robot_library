/**
 * @file    Pose.h
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
 * @copyright (c) 2025 Jon Woolfrey
 *
 * @license   OSCL - Free for non-commercial open-source use only.
 *            Commercial use requires a license.
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */

#ifndef POSE_H
#define POSE_H

#include <Eigen/Geometry>                                                                           // Quaternion
#include <iostream>                                                                                 // std::cout

namespace RobotLibrary { namespace Model {

class Pose
{
     public:

          /**
           * @brief An empty constructor.
           */
          Pose() {};
          
          /**
           * @brief A full constructor.
           * @param translation A 3x1 vector for the position/translation component.
           * @param quaternion A quaternion object defining the orientation.
           */
          Pose(const Eigen::Vector3d    &translation,
               const Eigen::Quaterniond &quaternion)
               :
               _translation(translation),
               _quaternion(quaternion.normalized()) {}
          
          /**
           * @brief Computes the error between this pose object and another.
           * @param desired The other pose for which to compute the error.
           * @return A 6x1 vector containing the position error, and orientation error as the angle*axis
           */
          Eigen::Vector<double,6> error(const Pose &desired);
          
          /**
           * @brief  Returns this pose as 4x4 homogeneous transformation / SE(3) matrix.
           */
          Eigen::Matrix4d as_matrix();

          /**
           * @brief  Returns the orientation component of the pose as a 3x3 rotation / SO(3) matrix
           */
          Eigen::Matrix3d rotation() const { return this->_quaternion.toRotationMatrix(); }
          
          /**
           * @return Returns the 3x1 translation component
           */
          Eigen::Vector3d translation() const { return this->_translation; }
          
          /**
           * @brief  Returns the orientation component as a quaternion object.
           */
          Eigen::Quaterniond quaternion() const { return this->_quaternion; }
          
          /**
           * @brief  Computes the inverse / opposite of this pose.
           */
          Pose
          inverse();
          
          /**
           * @brief Multiply this pose with another to produce a third.
           */
          Pose
          operator* (const Pose &other) const;
          
          /**
           * @brief Multiply this pose in place.
           */
          void
          operator*= (const Pose &other);
          
          /**
           * @brief Apply a point transformation to a vector.
           */
          Eigen::Vector<double,3>
          operator* (const Eigen::Vector<double,3> &other);
           
        private:

            Eigen::Vector3d _translation = {0.0, 0.0, 0.0};                                         ///< The position or translation component

            Eigen::Quaterniond _quaternion = {1.0, 0.0, 0.0, 0.0};                                  ///< The orientation or rotation component
     
};                                                                                                  // Semicolon needed after class declaration

} } // namespace

#endif
