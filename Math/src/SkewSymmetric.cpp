/**
 * @file   SkewSymmetric.cpp
 * @author Jon Woolfrey
 * @date   June 2024
 * @brief  A class uses for representing the skew-symmetric matrix mapping of a 3D vector.
 */

#include <SkewSymmetric.h>

template <class DataType>
Eigen::Matrix<DataType,3,3>
SkewSymmetric<DataType>::as_matrix()
{
   Eigen::Matrix<DataType,3,3> S;
   S <<             0 , -this->_vec(2),  this->_vec(1),
         this->_vec(2),             0 , -this->_vec(0),
        -this->_vec(1),  this->_vec(0),             0 ;
        
   return S;
}

template <class DataType>
Eigen::Matrix<DataType,3,Eigen::Dynamic>
SkewSymmetric<DataType>::operator*(const Eigen::Matrix<DataType,3,Eigen::Dynamic> &other)
{
    Eigen::Matrix<DataType,3,Eigen::Dynamic> result;
    result.resize(Eigen::NoChange,other.cols());

    for(int j = 0; j < other.cols(); j++)
    {
        result(0,j) = this->_vec(1)*other(2,j) - this->_vec(2)*other(1,j);
        result(1,j) = this->_vec(2)*other(0,j) - this->_vec(0)*other(2,j);
        result(2,j) = this->_vec(0)*other(1,j) - this->_vec(1)*other(0,j);
    }

    return result;
}
