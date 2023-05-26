    ///////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                               //
  //                              Test the QR Decomposition algorithm                              //
 //                                                                                               //
///////////////////////////////////////////////////////////////////////////////////////////////////

#include <iostream>                                                                                 // std::cout
#include <Math.h>                                                                                   // QR decomposition

int main(int argc, char* argv[])
{
	int n = 7;
	
	Eigen::MatrixXf A = Eigen::MatrixXf::Random(n,n);
	
	A.col(6) = 0.7*A.col(1);                                                                    // Force a singularity
	
	Eigen::MatrixXf Q, R;
	
	if(not qr_decomposition(A,Q,R))
	{
		std::cerr << "[FLAGRANT SYSTEM ERROR] qr_test(): "
		          << "Unable to obtain the QR decomposition.\n";
		
		return 1;
	}
	
	std::cout << "\nHere is Q:\n";
	std::cout << Q << std::endl;
	
	std::cout << "\nHere is Q'*Q:\n";
	std::cout << Q.transpose()*Q << std::endl;
	
	std::cout << "\nHere is R:\n";
	std::cout << R << std::endl;
	
	std::cout << "\nHere is A - Q*R:\n";
	std::cout << A - Q*R << std::endl;
	
	Eigen::MatrixXf N = triangular_null_space(R);
	
	std::cout << "\nHere is the null space projector of R:\n";
	std::cout << N << std::endl;
	
	std::cout << "\nAnd here is R*N:\n";
	std::cout << R*N << std::endl;
	
	std::cout << "\nHere is A*N:\n";
	std::cout << A*N << std::endl;
		
	return 0;                                                                                   // No problems with main
}
