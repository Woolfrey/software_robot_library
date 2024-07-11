/**
 * @file   SkewSymmetric.h
 * @author Jon Woolfrey
 * @date   June 2024
 * @brief  A class uses for representing the skew-symmetric matrix mapping of a 3D vector.
 */
 
#ifndef SKEW_SYMMETRIC_H
#define SKEW_SYMMETRIC_H

#include <Eigen/Core>
 
class SkewSymmetric
{
     public:
          /**
           * Constructor.
           * @param vec A 3D Eigen::Vector to be made as a skew-symmetric matrix.
           */
          SkewSymmetric(const Eigen::Vector3d &vec) : _vec(vec) {}
          
          /**
           * Get this object as a 3x3 matrix.
           * @return An 3D Eigen::Matrix object.
           */
          Eigen::Matrix3d
          as_matrix();

          /**
           * Multiply this skew-symmetric matrix with another tensor.
           * This speeds up calcs by skipping the 0's along the diagonal.
           * @param other The other tensor to multiply with (3xn)
           * @return A 3xn matrix resulting from the product.
           */
          Eigen::Matrix<double,3,Eigen::Dynamic>
          operator*(const Eigen::Matrix<double,3,Eigen::Dynamic> &other);
          
     private:
     
          Eigen::Vector3d _vec;                                                                     ///< The underlying 3D vector
          
};                                                                                                  // Semicolon needed after class declaration

#endif
