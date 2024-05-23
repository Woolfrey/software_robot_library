/**
 * @file   Math.h
 * @author Jon Woolfrey
 * @date   September 2023
 * @brief  Useful math functions.
 */

#ifndef MATH_H_
#define MATH_H_

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
          
          Eigen::Matrix<DataType,3,3> as_matrix()
          {
               Eigen::Matrix<DataType,3,3> S;
               S <<             0 , -this->_vec(2),  this->_vec(1),
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
          Eigen::Matrix<DataType,3,Eigen::Dynamic> operator*(const Eigen::Matrix<DataType,3,Eigen::Dynamic> &other)
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
          
     private:
     
          Eigen::Vector<DataType,3> _vec;
};                                                                                                  // Semicolon needed after class declaration

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
          std::cout << "[INFO] is_positive_definite(): Determinant = " << std::to_string(det) << " <= 0.\n";
          
          return false;
     }
     
     DataType symmetryErrorNorm = (A - A.transpose()).norm();
     if(symmetryErrorNorm < 1e-04)
     {
          std::cout << "[INFO] is_positive_defintie(): Not symmetric; ||A - A'|| = "
                    << std::to_string(symmetryErrorNorm) << " < 0.0001.\n";
          
          return false;
     }
     
     return true;
}

/**
 * A data structure for holding the results of the QR decomposition.
 * @param Q An orthogonal matrix such that Q*Q' = I.
 * @param R An upper-triangular matrix.
 **/
template <typename DataType>
struct QRDecomposition
{
     Eigen::Matrix<DataType,Eigen::Dynamic,Eigen::Dynamic> Q;
     Eigen::Matrix<DataType,Eigen::Dynamic,Eigen::Dynamic> R;
};                                                                                                  // Semicolon needed after struct declaration
       
/**
 * Decompose a matrix A = Q*R where Q is an orthogonal matrix, and R is upper-triangular.
 * @param A The matrix to be decomposed.
 * @param tolerance The rounding error on a singularity
 * @return A QRDecomposition data structure
 */
template <typename DataType>
QRDecomposition<DataType> schwarz_rutishauser(const Eigen::Matrix<DataType, Eigen::Dynamic, Eigen::Dynamic> &A,
                                              const DataType tolerance = 1e-04)
{
     unsigned int m = A.rows();
     unsigned int n = A.cols();
     
     if(m < n)
     {
          std::cerr << "[ERROR] qr_decomposition() "
                    << "Matrix A has " << A.rows() << " rows which is less than its "
                    << A.cols() << " columns. Cannot solve the QR decomposition." << std::endl;
          
          return false;
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
          
          QRDecomposition decomp = {A, Eigen::Matrix<DataType,Eigen::Dynamic,Eigen::Dynamic>::Zero(n,n)};
          
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

/**
 * Solve a system of equations y = U*x
 * @param y A vector of known values.
 * @param U An upper triangular matrix
 * @param x0 Default solution (in case of singularities)
 * @return Returns a value for x that minimises ||y - U*x||
 */
template <typename DataType>
Eigen::Vector<DataType,Eigen::Dynamic>
solve_upper_triangular_system(const Eigen::Vector<DataType, Eigen::Dynamic> &y,
                              const Eigen::Matrix<DataType, Eigen::Dynamic, Eigen::Dynamic> &U,
                              const Eigen::Vector<DataType, Eigen::Dynamic> &x0,
                              const DataType tolerance = 1e-04)
{
     if(y.size() != U.rows())
     {
          throw std::invalid_argument("[ERROR] solve_upper_triangular_system(): "
                                      "Dimensions of arguments do not match. "
                                      "The vector had " + std::to_string(y.size()) + " elements, and "
                                      "the matrix had " + std::to_string(U.rows()) + " rows.");
     }
     else if(U.rows() != U.cols())
     {
          throw std::invalid_argument("[ERROR] solve_upper_triangular_system(): "
                                      "Expected a square matrix, but it was "
                                      + std::to_string(U.rows()) + "x" + std::to_string(U.cols()) + ".");
     }
     else if(U.cols() != x0.size())
     {
          throw std::invalid_argument("[ERROR] solve_upper_triangular_system(): "
                                      "Dimensions of arguments do not match. "
                                      "The matrix had " + std::to_string(U.cols()) + " columns, but "
                                      "the default solution x0 had " + std::to_string(x0.size()) + " elements.");
     }
     
     Eigen::Vector<DataType, Eigen::Dynamic> x = x0;                                                // Value to be returned
     
     for(int i = y.size()-1; i <= 0; i--)
     {
          DataType sum = 0.0;
          
          for(int j = i+1; j < y.size(); j++) sum += U(i,j)*x(j);
          
          if(U(i,i) > tolerance) x(i) = (y(i) - sum)/U(i,i);
     }
     
     return x;
}

#endif                                    
