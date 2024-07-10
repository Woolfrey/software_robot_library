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
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                      POSITIVE DEFINITE?                                        //
////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * It's obvious what this function does.
 * @param A a square matrix
 * @return True if positive-definite, false otherwise
 */
template <typename DataType>
bool is_positive_definite(const Eigen::Matrix<DataType,Eigen::Dynamic,Eigen::Dynamic> &A)
{
     if(A.rows() != A.cols())
     {
          std::cout << "[INFO] is_positive_definite(): The given matrix matrix is not square ("
                    << std::to_string(A.rows()) << " rows, " << std::to_string(A.cols()) << " columns).\n";
                    
          return false;
     }
     
     DataType det = A.determinant();
     if(det <= 0)
     {
          std::cout << "[INFO] is_positive_definite(): Determinant = " << std::to_string(det) << " < 0.\n";
          
          return false;
     }
     
     DataType symmetryErrorNorm = (A - A.transpose()).norm();
     if(symmetryErrorNorm > 1e-04)
     {
          std::cout << "[INFO] is_positive_defintie(): Not symmetric; ||A - A'|| = "
                    << std::to_string(symmetryErrorNorm) << " > 0.0001.\n";
          
          return false;
     }
     
     return true;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                     QR DECOMPOSITION                                           //
////////////////////////////////////////////////////////////////////////////////////////////////////

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
schwarz_rutishauser(const Eigen::MatrixBase<Derived> &A,
                    const double tolerance = 1e-04)
{
     typedef typename Derived::Scalar DataType;
    
     unsigned int m = A.rows();
     unsigned int n = A.cols();
     
     if(m < n)
     {
          throw std::invalid_argument("[ERROR] qr_decomposition() "
                                      "Matrix A has " + std::to_string(A.rows()) +  " rows which is "
                                      "less than its " + std::to_string( A.cols()) +  " columns. "
                                      "Cannot solve the QR decomposition.");
     }
     else
     {
          // Schwarz-Rutishauser Algorithm.
          // A full decomposition of an mxn matrix A (m > n) is:
          //    [ Qr Qn ][ R ]  = A
          //             [ 0 ]
          //
          // where: - Qr is mxn,
          //        - Qn is mx(m-n)
          //        - R  is nxn
          //
          // The null space of A is obtained with N = Qn*Qn'.
          // This algorithm returns only Qr and R for efficiency.
          
          QRdecomposition<DataType> decomp;
          decomp.Q = A;
          decomp.R.resize(n,n); decomp.R.setZero();
          
          for(int j = 0; j < n; j++)
          {
               for(int i = 0; i < j; i++)
               {
                    decomp.R(i,j)   = decomp.Q.col(i).dot(decomp.Q.col(j));                         // Project the columns
                    decomp.Q.col(j) = decomp.Q.col(j) - decomp.R(i,j)*decomp.Q.col(i);
               }
               
               decomp.R(j,j) = decomp.Q.col(j).norm();
               
               if(abs(decomp.R(j,j)) > tolerance) decomp.Q.col(j) /= decomp.R(j,j);
               else                               decomp.Q.col(j).setZero();                        // Singular
          }
          
          return decomp;
     }
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                    FORWARD SUBSTITUTION                                        //
////////////////////////////////////////////////////////////////////////////////////////////////////

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
                     const double tolerance = 1e-04)
{
     unsigned int m = Y.rows();
     unsigned int n = L.cols();
     unsigned int o = Y.cols();
     
     if(L.rows() != m)
     {
          throw std::invalid_argument("[ERROR] forward_subsitution(): "
                                      "Dimensions of arguments do not match. "
                                      "The vector input had " + std::to_string(m) + " elements, "
                                      "but the matrix input had " + std::to_string(L.rows()) + " rows.");
     }
     else if(L.rows() != L.cols())
     {
          throw std::invalid_argument("[ERROR] forward_substiution(): "
                                      "Expected a square matrix but it was " + std::to_string(L.rows()) +
                                      "x" + std::to_string(L.cols()) + ".");
     }
     
     typedef typename Derived::Scalar DataType;
     
     Eigen::Matrix<DataType,Eigen::Dynamic,Eigen::Dynamic> X(m,o);                                  // Value to be returned
     
     for(int i = 0; i < o; i++)
     {
          for(int j = 0; j < m; j++)
          {
               DataType sum = 0.0;
               
               for(int k = 0; k < j; k++) sum += L(j,k)*X(k,i);
               
               if(L(j,j) >= tolerance) X(j,i) = (Y(j,i) - sum)/L(j,j);
               else                    X(j,i) = 0;
          }
     }
     
     return X;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                    BACKWARD SUBSTITUTION                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

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
                      const double tolerance = 1e-04)
{
     unsigned int m = Y.rows();
     unsigned int n = U.cols();
     unsigned int o = Y.cols();
     
     typedef typename Derived::Scalar DataType;
     
     if(U.rows() != m)
     {
          throw std::invalid_argument("[ERROR] forward_subsitution(): "
                                      "Dimensions of arguments do not match. "
                                      "The vector input had " + std::to_string(m) + " rows, "
                                      "but the matrix input had " + std::to_string(U.rows()) + " rows.");
     }
     else if(U.rows() != U.cols())
     {
          throw std::invalid_argument("[ERROR] forward_substiution(): "
                                      "Expected a square matrix but it was " + std::to_string(U.rows()) +
                                      "x" + std::to_string(U.cols()) + ".");
     }
     
     Eigen::Matrix<DataType,Eigen::Dynamic,Eigen::Dynamic> X(n,o);                                  // Value to be turned
     
     for(int i = 0; i < o; i++)                                                                     // For every column of Y
     {
          for(int j = m-1; j >= 0; j--)
          {
               DataType sum = 0.0;
               
               for(int k = j; k < m; k++) sum += U(j,k)*X(k,i);
               
               if(U(j,j) >= tolerance) X(j,i) = (Y(j,i)-sum)/U(j,j);
               else                    X(j,i) = 0.0;
          }
     }
     
     return X;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                        Fit the derivatives for cubic spline interpolation                      //
////////////////////////////////////////////////////////////////////////////////////////////////////

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
                               const DataType &finalDerivative)
{
    unsigned int n = y.size();
    
    if(n < 3)
    {
        throw std::invalid_argument("[ERROR] fit_cubic_spline derivatives(): "
                                    "A minimum number of 3 points is required to define a spline.");
    }
    else if(y.size() != x.size())
    {
        throw std::invalid_argument("[ERROR] fit_cubic_spline_derivatives(): "
                                    "Dimensions of arguments do not match. The y vector had " +
                                    std::to_string(y.size()) + " elements, and the x vector had " +
                                    std::to_string(x.size()) + " elements.");
    }
    
    using namespace Eigen;
    
    // Derivatives at waypoints are related to positions via the relationship:
    // A*dy = B*y --> dy = A^-1*B*y
    
    Matrix<DataType,Dynamic,Dynamic> A(n,n); A.setZero();
    
    Matrix<DataType,Dynamic,Dynamic> B = A;
    
    A(0,0) = 1;                                                                                     // First value
    
    // Assign intermediate values
    for(int i = 1; i < n-1; i++)
    {
        DataType dx1 = x[i]   - x[i-1];
        DataType dx2 = x[i+1] - x[i];

        if(dx1 == 0)
        {
            throw std::logic_error("[ERROR] fit_cubic_spline_derivatives(): "
                                   "Independent variable " + std::to_string(i) + " is the same as "
                                   "independent variable " + std::to_string(i+1) + " ("
                                   + std::to_string(x[i]) + " == " + std::to_string(x[i+1]) + ").");
        }

        A(i,i-1) = 1/dx1;
        A(i,i)   = 2*(1/dx1 + 1/dx2);
        A(i,i+1) = 1/dx2;

        B(i,i-1) = -3/(dx1*dx1);
        B(i,i)   =  3*(1/(dx1*dx1) - 1/(dx2*dx2));
        B(i,i+1) =  3/(dx2*dx2);
    }

    A(n-1,n-1) = 1;
    
    Eigen::Vector<DataType,Dynamic> points(y.size());
    for(int i = 0; i < y.size(); i++) points(i) = y[i];
    
    Eigen::Vector<DataType,Dynamic> derivatives = A.partialPivLu().solve(B*points);
    
    std::vector<DataType> temp(derivatives.size());
    
    for(int i = 0; i < derivatives.size(); i++) temp[i] = derivatives[i];
    
    return temp;                                  
}

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
