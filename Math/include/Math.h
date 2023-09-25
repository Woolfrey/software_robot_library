/**
 * @file   Pose.h
 * @author Jon Woolfrey
 * @date   September 2023
 * @brief  Useful math functions.
 */

#ifndef MATH_H_
#define MATH_H_

#include <Eigen/Core>

using namespace Eigen;
       
/**
 * Decompose a matrix A = Q*R where Q is an orthogonal matrix, and R is upper-triangular.
 * @param A The matrix to be decomposed.
 * @param Q A placeholder for the orthogonal matrix.
 * @param R A placeholder for the triangular matrix.
 * @return Returns false if there was a problem.
 */
template <typename DataType>
bool qr_decomposition(const Matrix<DataType, Dynamic, Dynamic> &A,
                      const Matrix<DataType, Dynamic, Dynamic> &Q,
                      const Matrix<DataType, Dynamic, Dynamic> &R)
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
		
		Q = A;
		R = Matrix<DataType,Dynamic,Dynamic>::Zero(n,n);
		
		for(int j = 0; j < n; j++)
		{
			for(int i = 0; i < j; i++)
			{
				R(i,j)   = Q.col(i).dot(Q.col(j));                                  // Project the columns
				Q.col(j) = Q.col(j) - R(i,j)*Q.col(i);
			}
			
			R(j,j) = Q.col(j).norm();
			
			
			if(abs(R(j,j)) > 1E-07) Q.col(j) /= R(j,j);
			else                    Q.col(j).setZero();
		}
		
		return true;
	}
}

#endif                                    
