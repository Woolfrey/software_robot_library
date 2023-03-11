#include <iostream>                                                                                 // std::cerr
#include <Math.h>                                                                                   // Declaration of function names

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //           Decompose a matrix A in to orthogonal matrix Q and triangular matrix R              //
///////////////////////////////////////////////////////////////////////////////////////////////////
bool qr_decomposition(const Eigen::MatrixXf &A,
                            Eigen::MatrixXf &Q,
                            Eigen::MatrixXf &R)
{
	int m = A.rows();
	int n = A.cols();
	
	if(m < n)
	{
		std::cerr << "[ERROR] qr_decomp() "
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
		R = Eigen::MatrixXf::Zero(n,n);
		
		for(int j = 0; j < n; j++)
		{
			for(int i = 0; i < j; i++)
			{
				R(i,j)   = Q.col(i).dot(Q.col(j));                                  // Project the columns
				Q.col(j) = Q.col(j) - R(i,j)*Q.col(i);
			}
			
			R(j,j) = Q.col(j).norm();
			
			if(abs(R(j,j)) > 1E-6) Q.col(j) /= R(j,j);
			else                   Q.col(j).setZero();
		}
		
		return true;
	}
}
