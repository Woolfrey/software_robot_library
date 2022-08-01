#include <iostream>                                                                                 // std::cout 
#include <QPSolver.h>
#include <time.h>                                                                                   // clock, clock_t, CLOCKS_PER_SEC

int main(int argc, char *argv[])
{
	srand((unsigned int) time(0));					                       // Random seed generator
	
	// Variables used in this scope
	clock_t timer;
	int m, n;
	Eigen::MatrixXf A;
	Eigen::VectorXf y;
	Eigen::VectorXf x;
	Eigen::VectorXf x0;
	Eigen::VectorXf xHat;
	
	
	std::cout << "\n************************************************************" << std::endl;
	std::cout <<   "*                   CHOLESKY DECOMPOSITION                 *" << std::endl;
	std::cout <<   "************************************************************\n" << std::endl;
	
	m = 5;
	n = 5;
	A = Eigen::MatrixXf::Random(m,n);
	A = A*A.transpose();
	
	std::cout << "\nWe can use Cholesky decomposition to express a positive-definite matrix A as L*L'.\n" << std::endl;
	std::cout << "\nHere is the matrix A:\n" << std::endl;
	std::cout << A << std::endl;
	
	timer = clock();
	Eigen::MatrixXf L = get_cholesky_decomp(A);
	timer = clock() - timer;
	
	std::cout << "\nHere is the matrix L:\n" << std::endl;
	std::cout << L << std::endl;
	std::cout << "\nIt took " << (float)timer/CLOCKS_PER_SEC*1000 << "ms to solve ("
	          <<  1/((float)timer/CLOCKS_PER_SEC) << " Hz).\n" << std::endl;
	          
	std::cout << "\nThe error norm ||A - L*L'|| is: " << (A - L*L.transpose()).norm() << ".\n" << std::endl;
	          
	std::cout << "\nIf A is part of a linear system y = A*x, give y we can solve for x.\n" << std::endl;
	
	x = Eigen::VectorXf::Random(n);
	y = A*x;
	timer = clock();
	xHat = solve_cholesky_system(y,A);
	timer = clock() - timer;
	
	std::cout << "\nThe error for the estimate of x is: " << (x - xHat).norm() << ". "
	          << "It took " << (float)timer/CLOCKS_PER_SEC*1000 << "ms to solve ("
	          << 1/((float)timer/CLOCKS_PER_SEC) << " Hz).\n" << std::endl;
	          
	
	std::cout << "\nWe can also use the triangular property to compute the inverse of A.\n" << std::endl;
	std::cout << "\nHere is the inverse of A:\n" << std::endl;
	timer = clock();
	Eigen::MatrixXf invA = get_cholesky_inverse(A);
	timer = clock() - timer;
	
	std::cout << invA << std::endl;
	
	std::cout << "\nIt took " << (float)timer/CLOCKS_PER_SEC*1000 << " ms to solve ("
	          << 1/((float)timer/CLOCKS_PER_SEC) << " Hz)." << std::endl;
	          
	std::cout << "\nHere is A*inv(A):\n" << std::endl;
	std::cout << A*invA << std::endl;
	
	std::cout << "\n************************************************************" << std::endl;
	std::cout <<   "*                      QR DECOMPOSITION                    *" << std::endl;
	std::cout <<   "************************************************************\n" << std::endl;
	
	m = 6;
	n = 5;
	
	A = Eigen::MatrixXf::Random(m,n);
	
	A.col(0) = A.col(n-2);                                                                        // Force a singularity
	std::cout << "\nHere is a random matrix:\n" << std::endl;
	std::cout << A << std::endl;

	
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
	
	std::cout << "\nHere is Q*R:\n" << std::endl;
	std::cout << Q*R << std::endl;
	
	std::cout << "\nThe error norm || A - Q*R || is: " << (A-Q*R).norm() << "." << std::endl;
	
	if( m > n )
	{
		std::cout << "\nThe null space of A is:\n" << std::endl;
		std::cout << Eigen::MatrixXf::Identity(m,m) - Q*Q.transpose() << std::endl;
	}


	std::cout << "\n************************************************************" << std::endl;
	std::cout <<   "*             SOLVING LINEAR SYSTEMS WITH QR               *" << std::endl;
	std::cout <<   "************************************************************\n" << std::endl;
	
	std::cout << "\nHere is a random system of equations y = A*x.\n" << std::endl;
	
	m = 6;
	n = 6;
	
	A = Eigen::MatrixXf::Random(m,n);
	x = Eigen::VectorXf::Random(n);
	y = A*x;
	
	std::cout << "\ny:\n" << std::endl;
	std::cout << y << std::endl;
	std::cout << "\nA:\n" << std::endl;
	std::cout << A << std::endl;

	std::cout << "\nUsing QR decomposition, the estimate for x is:\n" << std::endl;
	
	timer   = clock();
	xHat = solve_linear_system(y,A,x);
	timer    = clock() - timer;
	std::cout << xHat << std::endl;
	
	std::cout << "\nAnd here was the original vector:\n" << std::endl;
	std::cout << x << std::endl;
	
	std::cout << "\nThe solution error is " << (x - xHat).norm() << std::endl;
	
	std::cout << "\nIt took " << (float)timer/CLOCKS_PER_SEC*1000 << "ms to solve ("
	          << 1/((float)timer/CLOCKS_PER_SEC) << "Hz).\n" << std::endl;

	std::cout << "\n************************************************************" << std::endl;
	std::cout <<   "*               QUADRATIC PROGRAMMING (QP)                 *" << std::endl;
	std::cout <<   "************************************************************\n" << std::endl;
	
	
	// Test an underdetermined system
	m = 6;
	n = 4;
	A = Eigen::MatrixXf::Random(m,n);
	x = Eigen::VectorXf::Random(n);
	y = A*x;
	
	std::cout << "\nHere is the matrix for a random, underdetermined system y = A*x:\n" << std::endl;
	std::cout << A << std::endl;
	
	std::cout << "\nAnd here is the output values y:\n" << std::endl;
	std::cout << y << std::endl;
	
	std::cout << "\nWe can use linear least squares to get an estimate for the input vector x." << std::endl;
	timer = clock();
	xHat = QPSolver::least_squares(y,A,Eigen::MatrixXf::Identity(m,m), Eigen::VectorXf::Zero(n));
	timer = clock() - timer;
	
	std::cout << "The estimate is:\n" << std::endl;
	std::cout << xHat << std::endl;
	
	std::cout << "\nIt took " << (float)timer/CLOCKS_PER_SEC*1000<< "ms to solve ("
	          << 1/((float)timer/CLOCKS_PER_SEC) << "Hz).\n" << std::endl;
	std::cout << "\nThe solution error was " << (x-xHat).norm() << std::endl;
	
	m = 6;
	n = 7;
	A = Eigen::MatrixXf::Random(m,n);
	x = Eigen::VectorXf::Random(n);
	y = A*x;

	std::cout << "\nNow here is a random, over-determined system with matrix A:\n" << std::endl;
	std::cout << A << std::endl;
	std::cout << "\nAnd the vector y:\n" << std::endl;
	std::cout << y << std::endl;
	
	timer = clock();
	xHat = QPSolver::least_squares(Eigen::VectorXf::Zero(n),                                    // Desired value for xd
	                               Eigen::MatrixXf::Identity(n,n),                              // Weighting matrix W on solution
	                               y,
	                               A,
	                               Eigen::VectorXf::Zero(n));
	timer = clock() - timer;
	
	std::cout << "\nThe solution error ||y-A*x|| is: " << (y-A*xHat).norm() << ".\n" << std::endl;
	std::cout << "\nIt took " << (float)timer/CLOCKS_PER_SEC*1000 << "ms to solve ("
	          << 1/((float)timer/CLOCKS_PER_SEC) << "Hz).\n" << std::endl;
	          
	          
	std::cout << "\nWe can put constraints on the solution. "
	          << "We need to create a QPSolver object for this one since it relies on the Interior Point method." << std::endl;
	QPSolver solver;
	m = 6;
	n = 6;
	x.resize(n);
	x << 2, 1, -3, 3 ,2, -2;
	A = Eigen::MatrixXf::Random(m,n);
	y = A*x;
	Eigen::VectorXf xMax(n);
	Eigen::VectorXf xMin(n);
	xMax = 4*Eigen::VectorXf::Ones(n);
	xMin = -4*Eigen::VectorXf::Ones(n);
	
	timer = clock();
	x0.resize(n);
	x0 << 2, 1, -3, 3, 2, -2;
	xHat = solver.least_squares(y,A,Eigen::MatrixXf::Identity(m,m),xMin,xMax,x0);
	timer = clock() - timer;
	
	std::cout << "\nHere is xMax x xMin:\n" << std::endl;
	Eigen::MatrixXf result(3,n);
	result.row(0) = xMax.transpose();
	result.row(1) = xHat.transpose();
	result.row(2) = xMin.transpose();
	
	std::cout << result << std::endl;
	
	std::cout << "\nIt took " << (float)timer/CLOCKS_PER_SEC*1000 << "ms to solve. ("
	          << 1/((float)timer/CLOCKS_PER_SEC) << "Hz)." << std::endl;
	          
	std::cout << "\nThe solution error is " << ((y - A*xHat).norm())/y.norm()*100<< "% .\n" << std::endl;
	
	return 0;                                                                                   // No problems with main()
}

