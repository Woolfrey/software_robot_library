/**
 * @file    Pose2D.h
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    May 2025
 * @version 1.0
 * @brief   A class that describes the position & orientation of an object in 2D space.
 * 
 * @details This class describes the position of an object as a 2D vector, and the orientation as a
 *          scalar. Arithmetic can be used to propagate & invert these objects for
 *          performing transforms in 2D space.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */

#ifndef POSE2D_H
#define POSE2D_H

#include <Eigen/Geometry>                                                                           // Quaternion
#include <iostream>                                                                                 // std::cout

namespace RobotLibrary { namespace Model {

class Pose2D
{
     public:

          /**
           * @brief An empty constructor.
           */
          Pose2D() {};
          
          /**
           * @brief A full constructor.
           * @param translation A 3x1 vector for the position/translation component.
           * @param quaternion A quaternion object defining the orientation.
           */
          Pose2D(const Eigen::Vector2d &translation,
                 const double &angle)
          : _translation(translation),
            _angle(wrap_to_pi(angle)) {}
          
          /**
           * @brief Computes the error between this pose object and another.
           * @param desired The other pose for which to compute the error.
           * @return A 6x1 vector containing the position error, and orientation error as the angle*axis
           */
          Eigen::Vector3d
          error(const Pose2D &desired);
          
          /**
           * @brief Returns this pose as 4x4 homogeneous transformation / SE(3) matrix.
           */
          Eigen::Matrix3d
          as_matrix();

          /**
           * @brief Returns the orientation component of the pose as a 2x2 rotation / SO(2) matrix
           */
          Eigen::Matrix2d
          rotation();
          
          /**
           * @return Returns the 2x1 translation component
           */
          Eigen::Vector2d
          translation() const { return this->_translation; }

          /**
           * @brief  Computes the inverse / opposite of this pose.
           */
          Pose2D
          inverse();
          
          /**
           * @brief Multiply this pose with another to produce a third.
           */
          Pose2D
          operator* (const Pose2D &other) const;
          
          /**
           * @brief Multiply this pose in place.
           */
          void
          operator*= (const Pose2D &other);
          
          /**
           * @brief Apply a point transformation to a vector.
           */
          Eigen::Vector2d
          operator* (const Eigen::Vector2d &other);
           
        private:

            double _angle;                                                                          ///< Orientaiton.
            
            Eigen::Vector32 _translation = {0.0, 0.0, 0.0};                                         ///< The position or translation component
     
            /**
             * @brief Ensure that an angle is between -3.141592.. and 3.141592...
             * @param angle The value to be capped.
             * @return The angle re-mapped.
             */
            double
            wrap_to_pi(double &angle);
            {
                angle = fmod(angle + M_PI, 2 * M_PI);
                
                if(angle < 0) angle += 2 * M_PI;
                
                return angle - M_PI;
            }
};                                                                                                  // Semicolon needed after class declaration

} } // namespace

#endif
