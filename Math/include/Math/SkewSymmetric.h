/**
 * @file   SkewSymmetric.h
 * @author Jon Woolfrey
 * @date   June 2024
 * @brief  A class uses for representing the skew-symmetric matrix mapping of a 3D vector.
 */

#ifndef SKEWSYMMETRIC_H_
#define SKEWSYMMETRIC_H_

#include <Eigen/Core>

template <class DataType>
class SkewSymmetric
{
     public:
          /**
           * Constructor.
           * @param vec A 3D Eigen::Vector to be made as a skew-symmetric matrix.
           */
          SkewSymmetric(const Eigen::Vector<DataType,3> vec) : _vec(vec) {}
          
          /**
           * @return The object as a 3x3 Eigen::Matrix.
           */
          inline
          Eigen::Matrix<DataType,3,3>
          as_matrix();

          /**
           * Multiply this skew-symmetric matrix with another tensor.
           * This speeds up calcs by skipping the 0's along the diagonal.
           * @param other The other tensor to multiply with (3xn)
           * @return A 3xn matrix resulting from the product.
           */
          Eigen::Matrix<DataType,3,Eigen::Dynamic>
          operator*(const Eigen::Matrix<DataType,3,Eigen::Dynamic> &other);
          
     private:
     
          Eigen::Vector<DataType,3> _vec;
};                                                                                                  // Semicolon needed after class declaration

#endif
