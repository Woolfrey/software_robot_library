/**
 * @file    SkewSymmetric.h
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    July 2025
 * @version 1.1
 * @brief   Represents a 3D skew-symmetric matrix such that S^T = -S.
 * 
 * @details This class is used to generate a skew-symmetric matrix for use in computations that
 *          are otherwise not offered by the Eigen library. Eigen offers the cross() method for
 *          3D vectors, but does not enable representation as a matrix-vector product.
 * 
 * @copyright (c) 2025 Jon Woolfrey
 *
 * @license   OSCL - Free for non-commercial open-source use only.
 *            Commercial use requires a license.
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */
 
#ifndef SKEW_SYMMETRIC_H
#define SKEW_SYMMETRIC_H

#include <Eigen/Core>

namespace RobotLibrary { namespace Math {
 
class SkewSymmetric
{
     public:
          /**
           * @brief Constructor.
           * @param vec A 3D Eigen::Vector to be made as a skew-symmetric matrix.
           */
          SkewSymmetric(const Eigen::Vector3d &vec) : _vec(vec) {}
          
          /**
           * @brief Get this object as a 3x3 matrix.
           * @return An 3D Eigen::Matrix object.
           */
          Eigen::Matrix3d
          as_matrix();

          /**
           * @brief Multiply this skew-symmetric matrix with another tensor.
           *        This speeds up calcs by skipping the 0's along the diagonal.
           * @param other The other tensor to multiply with (3xn)
           * @return A 3xn matrix resulting from the product.
           */
          Eigen::Matrix<double,3,Eigen::Dynamic>
          operator*(const Eigen::Matrix<double,3,Eigen::Dynamic> &other);
          
     private:
     
          Eigen::Vector3d _vec;                                                                     ///< The underlying 3D vector
          
};                                                                                                  // Semicolon needed after class declaration

} } // namespace

#endif
