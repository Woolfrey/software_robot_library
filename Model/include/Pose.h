/**
 * @file   Pose.h
 * @author Jon Woolfrey
 * @date   September 2023
 * @brief  A class representing the position and orientation of an object in 3D space.
 */

#ifndef POSE_H_
#define POSE_H_

#include <Eigen/Geometry>                                                                           // Quaternion
#include <iostream>                                                                                 // std::cout

namespace RobotLibrary {

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
          Pose(const Eigen::Vector3d   &translation,
               const Eigen::Quaterniond &quaternion)
               :
               _translation(translation),
               _quaternion(quaternion.normalized()) {}
          
          /**
           * Computes the error between this pose object and another.
           * @param desired The other pose for which to compute the error.
           * @return A 6x1 vector containing the position error, and orientation error as the angle*axis
           */
          Eigen::Vector<double,6> error(const Pose &desired);
          
          /**
           * @return Returns this pose as 4x4 homogeneous transformation / SE(3) matrix.
           */
          Eigen::Matrix4d as_matrix();

          /**
           * @return Returns the orientation component of the pose as a 3x3 rotation / SO(3) matrix
           */
          Eigen::Matrix3d rotation() const { return this->_quaternion.toRotationMatrix(); }
          
          /**
           * @return Returns the 3x1 translation component
           */
          Eigen::Vector3d translation() const { return this->_translation; }
          
          /**
           * @return Returns the orientation component as a quaternion object.
           */
          Eigen::Quaterniond quaternion() const { return this->_quaternion; }
          
          /**
           * @return Computes the inverse / opposite of this pose.
           */
          Pose
          inverse();
          
          /**
           * Multiply this pose with another to produce a third.
           */
          Pose
          operator* (const Pose &other) const;
          
          /**
           * Multiply this pose in place.
           */
          void
          operator*= (const Pose &other);
          
          /**
           * Apply a point transformation to a vector.
           */
          Eigen::Vector<double,3>
          operator* (const Eigen::Vector<double,3> &other);
           
        private:

            Eigen::Vector3d _translation = {0.0, 0.0, 0.0};                                         ///< The position or translation component

            Eigen::Quaterniond _quaternion = {1.0, 0.0, 0.0, 0.0};                                  ///< The orientation or rotation component
     
};                                                                                                  // Semicolon needed after class declaration
}

#endif
