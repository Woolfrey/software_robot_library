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
		R = Eigen::MatrixXf::Zero(n,n);
		
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
 
  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                     Find the null space of an upper-right triangular matrix                   //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXf triangular_null_space(const Eigen::MatrixXf &U)
{
	if(U.rows() != U.cols())
	{
		std::cerr << "[ERROR] triangular_null_space(): Expected a square matrix, but the "
		          << "input was " << U.rows() << "x" << U.cols() << ".\n";
		
		return Eigen::MatrixXf::Zero(U.rows(),U.cols());
	}
	else
	{
		unsigned int n = U.rows();
		
		Eigen::MatrixXf N(n,n); N.setZero();                                                // Value to be returned
		
		for(int i = n-1; i >= 0; i--)                                                       // Work from last value up
		{
			for(int k = 0; k < n; k++)                                                  // Solve every column of N
			{
				float sum = 0.0;
				
				for(int j = n-1; j > i; j--) sum += U(i,j)*N(j,k);
				
				if(abs(U(i,i)) < 1e-07) N(i,k) = 1;                                 // Trivial
				else                    N(i,k) = -sum/U(i,i);
			}
		}
		
		return N;
	}
}

