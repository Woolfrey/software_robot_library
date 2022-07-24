#include <iostream>                                                                                 // std::cout 
#include <QPSolver.h>
#include <time.h>                                                                                   // clock, clock_t, CLOCKS_PER_SEC

int main(int argc, char *argv[])
{
	srand((unsigned int) time(0));					                       // Random seed generator
	
	// Variables used in this scope
	int m, n;
	Eigen::MatrixXf A;
	Eigen::VectorXf y;
	Eigen::VectorXf x;
	Eigen::VectorXf x0;
	Eigen::VectorXf xHat;
	
	std::cout << "\n************************************************************" << std::endl;
	std::cout <<   "*                      QR DECOMPOSITION                    *" << std::endl;
	std::cout <<   "************************************************************\n" << std::endl;
	
	m = 5;
	n = 3;
	
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
	
	std::cout << "\nThe error norm || A - Q*R || is:\n" << std::endl;
	std::cout << (A-Q*R).norm() << std::endl;
	
	if( m > n )
	{
		std::cout << "\nThe null space of A is:\n" << std::endl;
		std::cout << Eigen::MatrixXf::Identity(m,m) - Q*Q.transpose() << std::endl;
	}


	std::cout << "\n************************************************************" << std::endl;
	std::cout <<   "*               SOLVING LINEAR SYSTEMS WITH QR             *" << std::endl;
	std::cout <<   "************************************************************\n" << std::endl;
	
	std::cout << "\nHere is a random system of equations y = A*x.\n" << std::endl;
	
	m = 25;
	n = 25;
	
	A = Eigen::MatrixXf::Random(m,n);
	x = Eigen::VectorXf::Random(n);
	y = A*x;
	
	std::cout << "\ny:\n" << std::endl;
	std::cout << y << std::endl;
	std::cout << "\nA:\n" << std::endl;
	std::cout << A << std::endl;

	std::cout << "\nUsing QR decomposition, the estimate for x is:\n" << std::endl;
	
	clock_t time;
	time    = clock();
	xHat = solve_linear_system(y,A,x);
	time    = clock() - time;
	std::cout << xHat << std::endl;
	
	std::cout << "\nAnd here was the original vector:\n" << std::endl;
	std::cout << x << std::endl;
	
	std::cout << "\nThe solution error is " << (x - xHat).norm() << std::endl;
	
	std::cout << "\nIt ran at " << 1/((float)time/CLOCKS_PER_SEC) << "Hz." << std::endl;
	
	return 0;                                                                                   // No problems with main()
}

