/**
 * @file    SkewSymmetric.cpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    April 2025
 * @version 1.0
 * @brief   Represents a 3D skew-symmetric matrix such that S^T = -S.
 * 
 * @details This class is used to generate a skew-symmetric matrix for use in computations that
 *          are otherwise not offered by the Eigen library. Eigen offers the cross() method for
 *          3D vectors, but does not enable representation as a matrix-vector product.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */

#include <Math/SkewSymmetric.h>

namespace RobotLibrary { namespace Math {

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                 Get this object as a 3x3 matrix                                //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix<double,3,3>
SkewSymmetric::as_matrix()
{
    Eigen::Matrix3d S;
    S <<            0 , -this->_vec(2),  this->_vec(1),
         this->_vec(2),             0 , -this->_vec(0),
        -this->_vec(1),  this->_vec(0),             0 ;
        
    return S;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                       Multiply this skew-symmetric matrix with another tensor                  //
////////////////////////////////////////////////////////////////////////////////////////////////////
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

} }
