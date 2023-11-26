#include <iostream>                                                                                 // std::cerr, std::cout
#include <Polynomial.h>                                                                             // We want to test this
#include <string>
 
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                           MAIN                                                 //
////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
	if(argc != 2)
	{
		std::cerr << "[ERROR] [POLYNOMIAL TEST] Incorrect number of input arguments. "
		          << "Usage: './polynomial_test n' where 'n' is an odd integer." << std::endl;
		          
	
		return -1;                                                                           // Failure
	}
	
	Eigen::Vector<float,1> startPosition; startPosition(0) = 0;
	Eigen::Vector<float,1> endPosition;   endPosition(0)   = 1;
	Eigen::Vector<float,1> startVelocity; startVelocity(0) = 0;
	Eigen::Vector<float,1> endVelocity;   endVelocity(0)   = 0;
	
	try
	{
		unsigned int order = std::stoi(argv[1]);
		
		Polynomial<float> trajectory(startPosition,endPosition,startVelocity,endVelocity,0,1,order);
	
		float hertz = 50.0;
		unsigned int steps = 50;
		
		Eigen::MatrixXf state(steps,3);
		
		for(int i = 0; i < steps; i++)
		{
			float t = i/hertz;
			
			const auto &[pos, vel, acc] = trajectory.query_state(t);
			
			state(i,0) = pos(0);
			state(i,1) = vel(0);
			state(i,2) = acc(0);
		}
		
		std::cout << "\nHere is the state:\n\n";
		std::cout << state << std::endl;
	}
	catch(const std::exception &exception)
	{
		std::cerr << exception.what() << std::endl;
		
		return -1;
	}
	
	
	return 0;
}
