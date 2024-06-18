/**
 * @file   Math.h
 * @author Jon Woolfrey
 * @date   September 2023
 * @brief  Useful math functions.
 */

#ifndef MATH_H_
#define MATH_H_

#include <Eigen/Core>                                                                               // Eigen::Vector, Eigen::Matrix etc

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                    SKEW-SYMMETRIC MATRIX                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * This class represents the skew-symmetric matrix expansion of a 3D vector.
 */
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

#endif                                    
