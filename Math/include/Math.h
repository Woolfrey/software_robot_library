/**
 * @file   Math.h
 * @author Jon Woolfrey
 * @date   September 2023
 * @brief  Useful math functions.
 */

#ifndef MATH_H_
#define MATH_H_

#include <Eigen/Core>

/**
 * A data structure for holding the results of the QR decomposition.
 * @param Q An orthogonal matrix such that Q*Q' = I.
 * @param R An upper-triangular matrix.
 */
template <typename DataType>
struct QRDecomposition
{
	Eigen::Matrix<DataType,Eigen::Dynamic,Eigen::Dynamic> Q;
	Eigen::Matrix<DataType,Eigen::Dynamic,Eigen::Dynamic> R;
};                                                                                                  // Semicolon needed after struct declaration
       
/**
 * Decompose a matrix A = Q*R where Q is an orthogonal matrix, and R is upper-triangular.
 * @param A The matrix to be decomposed.
 * @param Q A placeholder for the orthogonal matrix.
 * @param R A placeholder for the triangular matrix.
 * @return Returns false if there was a problem.
 */
template <typename DataType>
QRDecomposition<DataType> schwarz_rutishauser(const Eigen::Matrix<DataType, Eigen::Dynamic, Eigen::Dynamic> &A)
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
				decomp.R(i,j)   = decomp.Q.col(i).dot(decomp.Q.col(j));             // Project the columns
				decomp.Q.col(j) = decomp.Q.col(j) - decomp.R(i,j)*decomp.Q.col(i);
			}
			
			decomp.R(j,j) = decomp.Q.col(j).norm();
			
			if(abs(decomp.R(j,j)) > 1E-07) decomp.Q.col(j) /= decomp.R(j,j);
			else                           decomp.Q.col(j).setZero();                   // Singular
		}
		
		return decomp;
	}
}

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

#endif                                    
