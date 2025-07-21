/**
 * @file    DataStructures.h
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    July 2025
 * @version 1.1
 * @brief   Data structures used in math functions & classes.
 * 
 * @copyright (c) 2025 Jon Woolfrey
 *
 * @license   OSCL - Free for non-commercial open-source use only.
 *            Commercial use requires a license.
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */

#ifndef MATH_DATA_STRUCTS_H
#define MATH_DATA_STRUCTS_H

#include <Eigen/Core>                                                                               // Eigen::MatrixXd

namespace RobotLibrary { namespace Math {

/**
 * @brief A data structure for representing the output of a function f(x), and its derivatives df/dx, d^2f/dx^2.
 */
struct FunctionPoint
{
    double value            = 0.0;                                                                  ///< The value y = f(x)
    double firstDerivative  = 0.0;                                                                  ///< dy/dx
    double secondDerivative = 0.0;                                                                  ///< d^2y/dx^2
    
};   

/**
 * A data structure for holding the results of the QR decomposition.
 */
struct QRDecomposition
{
     Eigen::MatrixXd Q;                                                                             ///< An orthogonal matrix such that Q'*Q = I
     Eigen::MatrixXd R;                                                                             ///< An upper-triangular matrix 
                                                                 
};   

} } // namespace
                     
#endif                                    
