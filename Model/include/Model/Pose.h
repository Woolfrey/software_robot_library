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
          Pose(const Eigen::Vector<DataType,3>   &translation,
               const Eigen::Quaternion<DataType> &quaternion)
               :
               _translation(translation),
               _quaternion(quaternion.normalized()) {}
          
          /**
           * Computes the error between this pose object and another.
           * @param desired The other pose for which to compute the error.
           * @return A 6x1 vector containing the position error, and orientation error as the angle*axis
           */
          Eigen::Vector<DataType,6> error(const Pose<DataType> &desired);
          
          /**
           * @return Returns this pose as 4x4 homogeneous transformation / SE(3) matrix.
           */
          Eigen::Matrix<DataType,4,4> as_matrix();

          /**
           * @return Returns the orientation component of the pose as a 3x3 rotation / SO(3) matrix
           */
          Eigen::Matrix<DataType,3,3> rotation() const { return this->_quaternion.toRotationMatrix(); }
          
          /**
           * @return Returns the 3x1 translation component
           */
          Eigen::Vector<DataType,3> translation() const { return this->_translation; }
          
          /**
           * @return Returns the orientation component as a quaternion object.
           */
          Eigen::Quaternion<DataType> quaternion() const { return this->_quaternion; }
          
          /**
           * @return Computes the inverse / opposite of this pose.
           */
          Pose<DataType> inverse();
          
          /**
           * @return Returns the product of this pose with another.
           */
          Pose<DataType> operator*(const Pose<DataType> &other) const;
          
          /**
           * @return Returns an in-place product of this pose and another.
           */
          void operator*=(const Pose<DataType> &other);
          
          /**
           * @return Performs a point transformation.
           */
          Eigen::Vector<DataType,3> operator*(const Eigen::Vector<DataType,3> &other);
          
     private:
     
          Eigen::Vector<DataType,3> _translation = {0.0, 0.0, 0.0};                                 ///< The position or translation component
          
          Eigen::Quaternion<DataType> _quaternion = {1.0, 0.0, 0.0, 0.0};                           ///< The orientation or rotation component
     
};                                                                                                  // Semicolon needed after class declaration

using Pose_f = Pose<float>;
using Pose_d = Pose<double>;

#endif
