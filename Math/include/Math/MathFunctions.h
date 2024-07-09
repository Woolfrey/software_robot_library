/**
 * @file   MathFunctions.h
 * @author Jon Woolfrey
 * @date   September 2023
 * @brief  Useful math functions.
 */

#ifndef MATHFUNCTIONS_H_
#define MATHFUNCTIONS_H_

#include <Eigen/Core>                                                                               // Eigen::Vector, Eigen::Matrix etc
#include <iostream>

/**
 * It's obvious what this function does.
 * @param A a square matrix
 * @return True if positive-definite, false otherwise
 */
template <typename DataType>
bool is_positive_definite(const Eigen::Matrix<DataType,Eigen::Dynamic,Eigen::Dynamic> &A);

/**
 * A data structure for holding the results of the QR decomposition.
 */
template <typename DataType>
struct QRdecomposition
{
     Eigen::Matrix<DataType,Eigen::Dynamic,Eigen::Dynamic> Q;                                       ///< An orthogonal matrix such that Q'*Q = I
     Eigen::Matrix<DataType,Eigen::Dynamic,Eigen::Dynamic> R;                                       ///< An upper-triangular matrix                                                             
};                                                                                                  // Semicolon needed after struct declaration
       
/**
 * Decompose a matrix A = Q*R where Q is an orthogonal matrix, and R is upper-triangular.
 * @param A The matrix to be decomposed.
 * @param tolerance The rounding error on a singularity
 * @return A QRDecomposition data structure
 */
template <typename Derived>
QRdecomposition<typename Derived::Scalar>
schwarz_rutishauser(const Eigen::MatrixBase<Derived> &A, const double tolerance = 1e-04);

/**
 * Solve a system of equations y = L*x, where L is a lower-triangular matrix.
 * @param y A vector of known values.
 * @param L A lower-triangular matrix.
 * @param tolerance For singularities.
 * @return A solution for x.
 */
template <typename Derived, typename OtherDerived> inline
Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic>
forward_substitution(const Eigen::MatrixBase<Derived>      &Y,
                     const Eigen::MatrixBase<OtherDerived> &L,
                     const double tolerance = 1e-04);

/**
 * Solve a system of equations Y = U*X, where U is an upper-triangular matrix.
 * @param Y A tensor (vector or matrix) of known values.
 * @param U An upper-triangular matrix.
 * @param tolerance For handling singularities.
 * @return A solution for X.
 */
template <typename Derived, typename OtherDerived> inline
Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic>
backward_substitution(const Eigen::MatrixBase<Derived> &Y,
                      const Eigen::MatrixBase<OtherDerived> &U,
                      const double tolerance = 1e-04);

/**
 * This function solves for the derivatives at each point of a cubic spline such that there is continuity.
 * @param y The dependent variable for the spline.
 * @param x The independent variable for the spline.
 * @param firstDerivative The value for dy/dx at the very first point.
 * @param finalDerivative The value for dy/dx at the final point.
 * @return An array containing the derivatives dy/dx for each point x.
 */
template <typename DataType>
inline
std::vector<DataType>
solve_cubic_spline_derivatives(const std::vector<DataType> &y,
                               const std::vector<DataType> &x,
                               const DataType &firstDerivative,
                               const DataType &finalDerivative);

/**
 * Fits the derivatives for points on a cubic spline. Assumes the initial and final derivatives are zero.
 * @param y The independent variable on the spline.
 * @param x The dependent variable on the spline.
 * @return An array containing the derivatives dy/dx associated with every point y.
 */
template <typename DataType>
inline
std::vector<DataType>
solve_cubic_spline_derivatives(const std::vector<DataType> &y,
                               const std::vector<DataType> &x)
{
    return solve_cubic_spline_derivatives(y,x,0.0,0.0);
}

#endif                                    
