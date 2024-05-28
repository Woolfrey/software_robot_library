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
struct QRdecomposition
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
QRdecomposition<DataType> schwarz_rutishauser(const Eigen::Matrix<DataType, Eigen::Dynamic, Eigen::Dynamic> &A,
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
          
          QRdecomposition decomp = {A, Eigen::Matrix<DataType,Eigen::Dynamic,Eigen::Dynamic>::Zero(n,n)};
          
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

#endif                                    
