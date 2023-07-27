#include <iostream>                                                                                 // std::cout
#include <fstream>                                                                                  // std::ofstream
#include <QPSolver.h>                                                                               // Custom cass
#include <time.h>                                                                                   // clock_t

int main(int argc, char *argv[])
{
	srand((unsigned int) time(0));					                            // Random seed generator
	
	// Variables used in this scope
	int m, n;
	Eigen::MatrixXf A, comparison, W;
	Eigen::VectorXf x, xd, xHat, x0, xMin, xMax, y;
	QPSolver solver;
	clock_t timer;
	float time;
	
	std::cout << "\n************************************************************\n"
	          <<   "*                  UNDERDETERMINED SYSTEMS	           *\n"
	          <<   "************************************************************\n" << std::endl;
	          
	m = 6;
	n = 5;
	A  = Eigen::MatrixXf::Random(m,n);
	x  = Eigen::VectorXf::Random(n);
	y  = A*x;
	W  = Eigen::MatrixXf::Identity(m,m);
	
	std::cout << "\nHere is an underdetermined system y = A*x.\n" << std::endl;
	
	std::cout << "\nA:\n" << std::endl;
	std::cout << A << std::endl;
	
	std::cout << "\ny:\n" << std::endl;
	std::cout << y << std::endl;
	
	std::cout << "\nWe can use quadratic programming (QP) to get the best estimate of x.\n" << std::endl;
	
	timer = clock();
	xHat  = QPSolver::least_squares(y,A,W);
	timer = clock() - timer;
	time  = (float)timer/CLOCKS_PER_SEC;
	
	std::cout << "\nHere is the estimate for x:\n" << std::endl;
	std::cout << xHat << std::endl;
	
	std::cout << "\nThe error ||y-A*x|| is " << (y-A*xHat).norm() << ". "
	          << "It took " << time*1000 << " ms to solve (" << 1/time << " Hz).\n" << std::endl;
	          
	std::cout << "\n************************************************************\n"
	          <<   "*                    CONSTRAINED SYSTEM                    *\n"
	          <<   "************************************************************\n" << std::endl;
	
	m = 8;
	n = 8;
	
	A = Eigen::MatrixXf::Random(m,n);
	x.resize(n);
	x << 1, 2, 3, 4, 5, 3, 2, 1;
	
	y = A*x;
	W = Eigen::MatrixXf::Identity(m,m);
	
	xMin = -6*Eigen::VectorXf::Ones(n);
	xMax =  6*Eigen::VectorXf::Ones(n);
	xMax(2) = 0.9*x(2);
	xMax(3) = 0.9*x(3);
	
	x0   = 0.5*(xMin + xMax);
	
	std::cout << "\nConsider the problem to minimize ||y-A*x|| for xMin <= x <= xMax.\n" << std::endl;
	
	std::cout << "\nHere is A:\n" << std::endl;
	std::cout << A << std::endl;
	
	std::cout << "\nand y:\n" << std::endl;
	std::cout << y << std::endl;
	
	timer = clock();
	xHat  = solver.least_squares(y,A,W,xMin,xMax,x0);
	timer = clock() - timer;
	time  = (float)timer/CLOCKS_PER_SEC;
	
	std::cout << "\nHere is xMin, the estimate for x, and xMax side-by-side:\n" << std::endl;
	comparison.resize(n,3);
	comparison.col(0) = xMin;
	comparison.col(1) = xHat;
	comparison.col(2) = xMax;
	std::cout << comparison << std::endl;
	
	std::cout << "\nThe error norm ||y-A*x|| is " << (y-A*xHat).norm() << ". "
	          << "It took " << time*1000 << " ms to solve (" << 1/time << " Hz)." << std::endl;
        
	std::cout << "\n************************************************************\n"
	          <<   "*                 OVERDETERMINED SYSTEMS                   *\n"
	          <<   "************************************************************\n" << std::endl;
	m = 12;
	n = 17;

	x = Eigen::VectorXf::Random(n);
	A = Eigen::MatrixXf::Random(m,n);
	y = A*x;
	
	xMin = -5*Eigen::VectorXf::Ones(n);
	xMax =  5*Eigen::VectorXf::Ones(n);
	
	x0 = 0.5*(xMin + xMax);
	
	xd = 10*Eigen::VectorXf::Random(n);
	
	try
	{
		timer = clock();
		xHat  = solver.redundant_least_squares(xd,Eigen::MatrixXf::Identity(n,n),y,A,xMin,xMax,x0);
		timer = clock() - timer;
		time  = (float)timer/CLOCKS_PER_SEC;
	
		std::cout << "\nHere is xMin, x, xHat, and xMax:\n" << std::endl;
		comparison.resize(n,3);
		comparison.col(0) = xMin;
		comparison.col(1) = xHat;
		comparison.col(2) = xMax;
		std::cout << comparison << std::endl;

		std::cout << "\nThe error norm ||y - A*x|| is " << (y-A*xHat).norm() << ". "
			  << "It took " << time*1000 << " ms to solve (" << 1/time << " Hz)." << std::endl;
	}
	catch(const std::exception &exception)
	{
		std::cout << exception.what() << std::endl;
	}
/*
	// Record statistics on performance
	std::ofstream out("qp_test_data.csv");
	for(int i = 0; i < 100; i++)
	{
		x = Eigen::VectorXf::Random(n);
		A = Eigen::MatrixXf::Random(m,n);
		y = A*x;
		
		xMin = -5*Eigen::VectorXf::Ones(n);
		xMax =  5*Eigen::VectorXf::Ones(n);
		
		x0 = 0.5*(xMin + xMax);
		
		xd = 10*Eigen::VectorXf::Random(n);
		
		timer = clock();
		xHat  = solver.redundant_least_squares(xd,Eigen::MatrixXf::Identity(n,n),y,A,xMin,xMax,x0);
		timer = clock() - timer;
		
		out << (float)timer/CLOCKS_PER_SEC << "," << (y-A*xHat).norm() << "\n";
	}
*/
	return 0; 
}
