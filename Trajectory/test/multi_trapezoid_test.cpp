#include <iostream>                                                                                 // std::cerr, std::cout
#include <MultiTrapezoid.h>                                                                         // We want to test this
#include <string>
 
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                           MAIN                                                 //
////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
	if(argc != 4)
	{
		std::cerr << "[ERROR] [MULTI TRAPEZOID TEST] Incorrect number of arguments. "
		          << "Usage: './multi_trapezoid_test numberOfPoints maxVel maxAcc'." << std::endl;
		          
		return -1;
	}
	
	unsigned int n = std::stoi(argv[1]);
	unsigned int maxVel = std::stod(argv[2]);
	unsigned int maxAcc = std::stod(argv[3]);
	
	// Set up the points for the spline
	std::vector<Eigen::Vector<double,Eigen::Dynamic>> points;
	
	for(int i = 0; i < n; i++)
	{
		Eigen::Vector<double,Eigen::Dynamic> temp(1); temp(0) = i;
		
		points.push_back(temp);
	}
		
	try
	{
		MultiTrapezoid<double> trajectory(points,maxVel,maxAcc,0);
		
		double hertz = 20.0;
		
		unsigned int steps = 50;
		
		Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> state(steps,3);
		
		for(int i = 0; i < steps; i++)
		{
			double t = i/hertz;
			
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
