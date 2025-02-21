/**
 * @file    MathFunctions.cpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    February 2025
 * @version 1.0
 * @brief   Useful math functions for robot kinematics & control.
 * 
 * @details This header files contains forward declarations for useful math functions that are not
 *          offered by the Eigen library.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */
 
#include <MathFunctions.h>

namespace RobotLibrary { namespace Math {

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                      POSITIVE DEFINITE?                                        //
////////////////////////////////////////////////////////////////////////////////////////////////////
bool is_positive_definite(const Eigen::MatrixXd &A)
{
     if(A.rows() != A.cols())
     {
          std::cout << "[INFO] is_positive_definite(): The given matrix matrix is not square ("
                    << std::to_string(A.rows()) << " rows, " << std::to_string(A.cols()) << " columns).\n";
                    
          return false;
     }
     
     double det = A.determinant();
     if(det <= 0)
     {
          std::cout << "[INFO] is_positive_definite(): Determinant = " << std::to_string(det) << " < 0.\n";
          
          return false;
     }
     
     double symmetryErrorNorm = (A - A.transpose()).norm();
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
RobotLibrary::Math::QRDecomposition
schwarz_rutishauser(const Eigen::MatrixXd &A,
                    const double tolerance)
{
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
          
          RobotLibrary::Math::QRDecomposition decomp;
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
Eigen::MatrixXd
forward_substitution(const Eigen::MatrixXd &Y,
                     const Eigen::MatrixXd &L,
                     const double tolerance)
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
     
     Eigen::MatrixXd X(m,o);                                                                        // Value to be returned
     
     for(int i = 0; i < o; i++)
     {
          for(int j = 0; j < m; j++)
          {
               double sum = 0.0;
               
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
Eigen::MatrixXd
backward_substitution(const Eigen::MatrixXd &U,
                      const Eigen::MatrixXd &Y,
                      const double tolerance)
{
     unsigned int m = Y.rows();
     unsigned int n = U.cols();
     unsigned int o = Y.cols();
       
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
     
     Eigen::MatrixXd X(n,o);                                                                        // Value to be turned
     
     for(int i = 0; i < o; i++)                                                                     // For every column of Y
     {
          for(int j = m-1; j >= 0; j--)
          {
               double sum = 0.0;
               
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
std::vector<double>
solve_cubic_spline_derivatives(const std::vector<double> &y,
                               const std::vector<double> &x,
                               const double &firstDerivative,
                               const double &finalDerivative)
{
    unsigned int n = y.size();
    
    if(n < 3)
    {
        throw std::invalid_argument("[ERROR] solve_cubic_spline derivatives(): "
                                    "A minimum number of 3 points is required to define a spline.");
    }
    else if(y.size() != x.size())
    {
        throw std::invalid_argument("[ERROR] solve_cubic_spline_derivatives(): "
                                    "Dimensions of arguments do not match. The y vector had " +
                                    std::to_string(y.size()) + " elements, and the x vector had " +
                                    std::to_string(x.size()) + " elements.");
    }
    
    using namespace Eigen;
    
    // Derivatives at waypoints are related to positions via the relationship:
    // A*dy = B*y --> dy = A^-1*B*y
    
    Eigen::MatrixXd A(n,n); A.setZero();
    
    Eigen::MatrixXd B = A;
    
    A(0,0) = 1;                                                                                     // First value
    
    // Assign intermediate values
    for(int i = 1; i < n-1; i++)
    {
        double dx1 = x[i]   - x[i-1];
        double dx2 = x[i+1] - x[i];

        if(dx1 == 0)
        {
            throw std::logic_error("[ERROR] fit_cubic_spline_derivatives(): "
                                   "Independent variable " + std::to_string(i) + " is the same as "
                                   "independent variable " + std::to_string(i+1) + " ("
                                   + std::to_string(x[i-1]) + " == " + std::to_string(x[i]) + ").");
        }

        A(i,i-1) = 1/dx1;
        A(i,i)   = 2*(1/dx1 + 1/dx2);
        A(i,i+1) = 1/dx2;

        B(i,i-1) = -3/(dx1*dx1);
        B(i,i)   =  3*(1/(dx1*dx1) - 1/(dx2*dx2));
        B(i,i+1) =  3/(dx2*dx2);
    }

    A(n-1,n-1) = 1;
    
    Eigen::VectorXd points(y.size());
    for(int i = 0; i < y.size(); i++) points(i) = y[i];
    
    Eigen::VectorXd derivatives = A.partialPivLu().solve(B*points);
    
    std::vector<double> temp(derivatives.size());
    
    for(int i = 0; i < derivatives.size(); i++) temp[i] = derivatives[i];
    
    return temp;                                  
}

} }
