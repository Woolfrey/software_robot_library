/**
 * @file   SkewSymmetric.cpp
 * @author Jon Woolfrey
 * @date   July 2024
 * @brief  Source files for the SkewSymmetric class.
 */

#include <SkewSymmetric.h>

/**
 * Get this object as a 3x3 matrix.
 */
Eigen::Matrix<double,3,3>
SkewSymmetric::as_matrix()
{
    Eigen::Matrix3d S;
    S <<            0 , -this->_vec(2),  this->_vec(1),
         this->_vec(2),             0 , -this->_vec(0),
        -this->_vec(1),  this->_vec(0),             0 ;
        
    return S;
}

/**
 * Multiply this skew-symmetric matrix with another tensor.
 * This speeds up calcs by skipping the 0's along the diagonal.
 * @param other The other tensor to multiply with (3xn)
 * @return A 3xn matrix resulting from the product.
 */
Eigen::Matrix<double,3,Eigen::Dynamic>
SkewSymmetric::operator*(const Eigen::Matrix<double,3,Eigen::Dynamic> &other)
{
    Eigen::Matrix<double,3,Eigen::Dynamic> result;
    result.resize(Eigen::NoChange,other.cols());

    for(int j = 0; j < other.cols(); j++)
    {
        result(0,j) = this->_vec(1)*other(2,j) - this->_vec(2)*other(1,j);
        result(1,j) = this->_vec(2)*other(0,j) - this->_vec(0)*other(2,j);
        result(2,j) = this->_vec(0)*other(1,j) - this->_vec(1)*other(0,j);
    }

    return result;
}
