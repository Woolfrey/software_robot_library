/**
 * @file    MathFunctions.h
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    July 2025
 * @version 1.1
 * @brief   Useful math functions for robot kinematics & control.
 * 
 * @details This header files contains forward declarations for useful math functions that are not
 *          offered by the Eigen library.
 * 
 * @copyright (c) 2025 Jon Woolfrey
 *
 * @license   OSCL - Free for non-commercial open-source use only.
 *            Commercial use requires a license.
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */

#ifndef MATH_FUNCTIONS_H
#define MATH_FUNCTIONS_H

#include <Math/DataStructures.h>

#include <Eigen/Dense>                                                                               // Eigen::Vector, Eigen::Matrix etc
#include <iostream>
#include <vector>

namespace RobotLibrary { namespace Math {

/**
 * It's obvious what this function does.
 * @param A a square matrix
 * @return True if positive-definite, false otherwise
 */
bool is_positive_definite(const Eigen::MatrixXd &A,
                          std::string &info);
       
/**
 * Decompose a matrix A = Q*R where Q is an orthogonal matrix, and R is upper-triangular.
 * @param A The matrix to be decomposed.
 * @param tolerance The rounding error on a singularity
 * @return A QRDecomposition data structure
 */
RobotLibrary::Math::QRDecomposition
schwarz_rutishauser(const Eigen::MatrixXd &A,
                    const double tolerance = 1e-04);

/**
 * Solve a system of equations y = L*x, where L is a lower-triangular matrix.
 * @param L A lower-triangular matrix.
 * @param U A vector of known values.
 * @param tolerance For singularities.
 * @return A solution for x.
 */
Eigen::MatrixXd
forward_substitution(const Eigen::MatrixXd &L,
                     const Eigen::MatrixXd &Y,
                     const double tolerance = 1e-04);

/**
 * Solve a system of equations Y = U*X, where U is an upper-triangular matrix.
 * @param U An upper-triangular matrix.
 * @param Y A tensor (vector or matrix) of known values.
 * @param tolerance For handling singularities.
 * @return A solution for X.
 */
Eigen::MatrixXd
backward_substitution(const Eigen::MatrixXd &U,
                      const Eigen::MatrixXd &Y,         
                      const double tolerance = 1e-04);

/**
 * This function solves for the derivatives at each point of a cubic spline such that there is continuity.
 * @param y The dependent variable for the spline.
 * @param x The independent variable for the spline.
 * @param firstDerivative The value for dy/dx at the very first point.
 * @param finalDerivative The value for dy/dx at the final point.
 * @return An array containing the derivatives dy/dx for each point x.
 */
std::vector<double>
solve_cubic_spline_derivatives(const std::vector<double> &y,
                               const std::vector<double> &x,
                               const double &firstDerivative = 0,
                               const double &finalDerivative = 0);

/**
 * @brief Ensure that an angle is between -3.141592.. and 3.141592...
 * @param angle The value to be capped.
 * @return The angle re-mapped.
 */
double
wrap_to_pi(const double &angle);
            
} }
                     
#endif                                    
