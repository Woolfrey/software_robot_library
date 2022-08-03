#include <iostream>                                                                                 // std::cout 
#include <Math.h>
#include <time.h>                                                                                   // clock, clock_t, CLOCKS_PER_SEC

int main(int argc, char *argv[])
{
	srand((unsigned int) time(0));					                       // Random seed generator
	
	// Variables used in this scope
	clock_t timer;
	float time, hertz;
	int m, n;
	Eigen::MatrixXf A, invA;
	Eigen::VectorXf y;
	Eigen::VectorXf x, xHat;
	Eigen::MatrixXf comparison;
	
	
	std::cout << "\n************************************************************" << std::endl;
	std::cout <<   "*                      LU DECOMPOSITION                    *" << std::endl;
	std::cout <<   "************************************************************" << std::endl;
	
	m = 6;
	n = 6;
	A = Eigen::MatrixXf::Random(m,n);
	A.col(n-2) = A.col(0); // Force a singularity
	
	std::cout << "\nWe can express a matrix A = LU, where L is a lower-triangular "
	          << "and U is upper-triangular." << std::endl;
	          
	std::cout << "\nHere is the matrix A:\n" << std::endl;
	std::cout << A << std::endl;
	
	Eigen::MatrixXf L, U;
	timer = clock();
	get_lu_decomposition(A,L,U);
	timer = clock() - timer;
	time  = (float)timer/CLOCKS_PER_SEC;
	
	std::cout << "\nHere is the matrix L:\n" << std::endl;
	std::cout << L << std::endl;
	
	std::cout << "\nAnd here is the matrix U:\n" << std::endl;
	std::cout << U << std::endl;
	
	std::cout << "\nThe error norm ||A-L*U|| is " << (A-L*U).norm() << ". "
	          << "It took " << time*1000 << " ms to solve (" << 1/time << " Hz)." << std::endl;
	
	std::cout << "\nWe can use the triangular property to get a fast inverse.\n" << std::endl;
	timer = clock();
	invA = get_inverse(A);
	timer = clock() - timer;
	time = (float)timer/CLOCKS_PER_SEC;
	
	std::cout << "\nHere is the inverse of A:\n" << std::endl;
	std::cout << invA << std::endl;
	
	std::cout << "\nIt took " << time*1000 << " ms to compute (" << 1/time << " Hz)." << std::endl;
	
	std::cout << "\nHere is A times its inverse:\n" << std::endl;
	std::cout << A*invA << std::endl;
	
	std::cout << "\nWe can use the triangular property to solve a system of equations y = A*x "
	          << "without having to compute the inverse matrix directly.\n" << std::endl;
	x = Eigen::VectorXf::Random(n);
	y = A*x;
	
	std::cout << "\nHere is the vector x:\n" << std::endl;
	std::cout << x << std::endl;
	
	std::cout << "\nAnd here is y = A*x:\n" << std::endl;
	std::cout << y << std::endl;
	
	timer = clock();
	xHat  = solve_linear_system(y,A,Eigen::VectorXf::Random(n));
	timer = clock() - timer;
	time  = (float)timer/CLOCKS_PER_SEC;	
	
	std::cout << "\nThen we call solve_linear_system(y,A) to get the estimate x'. "
		  << "Here is x and x' side by side:\n" << std::endl;

	comparison.resize(n,2);
	comparison.col(0) = x;
	comparison.col(1) = xHat;
	
	std::cout << comparison << std::endl;
	
	std::cout << "\nThe error norm ||y-A*x'|| is " << (y-A*xHat).norm() << ". "
	          << "It took " << time*1000 << " ms to solve (" << 1/time << " Hz)." << std::endl;
	          
	std::cout << "\nNotice that get_inverse(A) is not robust to singularities "
	          << "because it cannot return a proper inverse since A*invA =/= I. "
	          << "However, solve_linear_system(y,A) is robust!" << std::endl;
	
	std::cout << "\n************************************************************"   << std::endl;
	std::cout <<   "*                   CHOLESKY DECOMPOSITION                 *"   << std::endl;
	std::cout <<   "************************************************************\n" << std::endl;
	
	m = 5;
	n = 5;
	A = Eigen::MatrixXf::Random(m,n);
	A = A*A.transpose();
	
	/**************************** General test of Cholesky decomposition **********************/
	std::cout << "\nWe can use Cholesky decomposition to express a positive-definite matrix A as L*L'. "
	          << "\nHere is the matrix A:\n" << std::endl;
	          
	std::cout << A << std::endl;
	
	timer = clock();
	L = get_cholesky_decomposition(A);
	timer = clock() - timer;
	time  = (float)timer/CLOCKS_PER_SEC;
	
	std::cout << "\nHere is the matrix L:\n" << std::endl;
	std::cout << L << std::endl;
	std::cout << "\nIt took " << time*1000 << "ms to solve (" << 1/time << " Hz)." << std::endl;
	std::cout << "\nThe error norm ||A - L*L'|| is: " << (A - L*L.transpose()).norm() << ".\n" << std::endl;
	          
	/****************** Solving linear systems with a positive-definite matrix ****************/
	std::cout << "\nIf A is part of a linear system y = A*x, give y we can solve for x.\n" << std::endl;
	std::cout << "\nHere is the vector x:\n" << std::endl;
	x = Eigen::VectorXf::Random(n);
	std::cout << x << std::endl;
	
	std::cout << "\nAnd here is y = A*x:\n" << std::endl;
	y = A*x;
	std::cout << y << std::endl;
	
	timer = clock();
	xHat  = solve_cholesky_system(y,A);
	timer = clock() - timer;
	time  = (float)timer/CLOCKS_PER_SEC;
	
	std::cout << "\nHere is x and the estimate of x side by side:\n" << std::endl;
	comparison.resize(n,2);
	comparison.col(0) = x;
	comparison.col(1) = xHat;
	std::cout << comparison << std::endl;
	
	std::cout << "\nThe error norm ||x - x'|| is " << (x-xHat).norm() << ", "
	          << "and it took " << time*1000 << " ms to solve (" << 1/time << " Hz)." << std::endl;
	
	          
	std::cout << "\nWe can also use the triangular property to compute the inverse of A.\n" << std::endl;
	timer = clock();
	invA = get_cholesky_inverse(A);
	timer = clock() - timer;
	
	std::cout << "\nHere is the inverse of A:\n" << std::endl;
	std::cout << invA << std::endl;
	
	std::cout << "\nIt took " << time*1000 << " ms to solve (" << 1/time << " Hz)." << std::endl;
	
	std::cout << "\nHere is A*inv(A):\n" << std::endl;
	std::cout << A*invA << std::endl;
	
	
	std::cout << "\n************************************************************" << std::endl;
	std::cout <<   "*                      QR DECOMPOSITION                    *" << std::endl;
	std::cout <<   "************************************************************\n" << std::endl;
	
	m = 6;
	n = 5;
	A = Eigen::MatrixXf::Random(m,n);
	A.col(0) = A.col(n-2);                                                                      // Force a singularity
	
	/*************************** Demonstration of QR decomposition ***************************/
	std::cout << "\nHere is a random " << m << "x" << n << " matrix:\n" << std::endl;
	std::cout << A << std::endl;
	
	std::cout << "\nIt can be decomposed in to A = Q*R, where Q is orthogonal and R "
	          << "is upper triangular.\n" << std::endl;
	
	Eigen::MatrixXf Q, R;
	if( get_qr_decomposition(A,Q,R) )
	{
		std::cout << "\nHere is the orthogonal Q matrix:\n" << std::endl;
		std::cout << Q << std::endl;
		std::cout << "\nHere is Q'*Q:\n" << std::endl;
		std::cout << Q.transpose()*Q << std::endl;
		std::cout << "\nHere is the triangular R matrix:\n" << std::endl;
		std::cout << R << std::endl;
	}
	
	std::cout << "\nNotice that A is singular and the corresponding diagonal element of "
	          << "R is close to zero: " << R(n-2,n-2) << ".\n" << std::endl;
	
	std::cout << "\nThe QR decomposition is robust to singularities. "
                  << "The error norm ||A - Q*R|| is: " << (A-Q*R).norm() << "." << std::endl;

	std::cout << "\nThe null space matrix N can be computed with I - Q*Q':\n" << std::endl;
	int dim = std::max(m,n);
	Eigen::MatrixXf N = Eigen::MatrixXf::Identity(dim,dim) - Q*Q.transpose();
	std::cout << N << std::endl;
	
	if(m > n)
	{
		std::cout << "\nHere is N*A:\n" << std::endl;
		std::cout << N*A << std::endl;
	}
	else if(m < n)
	{
		std::cout << "\nHere is A*N:\n" << std::endl;
		std::cout << A*N << std::endl;
	}

	return 0;                                                                                   // No problems with main()
}

