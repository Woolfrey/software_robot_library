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

#include <Math/MathFunctions.h>

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
           * @brief A delegating constructor using 3 arguments because I'm lazy.
           * @param x The position in the x-coordinate
           * @param y The position in the y-coordinate
           * @param angle The orientation relative to the global frame.
           */
          Pose2D(const double &x,
                 const double &y,
                 const double &angle)
          : Pose2D(Eigen::Vector2d(x, y), angle) {}
          
          /**
           * @brief A full constructor.
           * @param translation A 3x1 vector for the position/translation component.
           * @param quaternion A quaternion object defining the orientation.
           */
          Pose2D(const Eigen::Vector2d &translation,
                 const double &angle)
          : _translation(translation),
            _angle(RobotLibrary::Math::wrap_to_pi(angle)) {}
          
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
          rotation() const;
          
          /**
           * @return Returns the 2x1 translation component
           */
          Eigen::Vector2d
          translation() const { return _translation; }

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
          
          /**
           * @brief Return the angle member of this class.
           * @return What you asked for.
           */
          double
          angle() const { return _angle; }
           
        private:

            double _angle = 0;                                                                      ///< Orientaiton.
            
            Eigen::Vector2d _translation = {0.0, 0.0};                                              ///< The position or translation component
            
};                                                                                                  // Semicolon needed after class declaration

} } // namespace

#endif
