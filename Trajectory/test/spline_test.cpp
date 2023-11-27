#include <iostream>                                                                                 // std::cerr, std::cout
#include <Spline.h>                                                                                 // We want to test this
#include <string>
 
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                           MAIN                                                 //
////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
	if(argc != 3)
	{
		std::cerr << "[ERROR] [SPLINE TEST] Incorrect number of arguments. "
		          << "Usage: './spline_test numberOfPoints polynomialOrder'." << std::endl;
		          
		return -1;
	}
	
	unsigned int n = std::stoi(argv[1]);
	unsigned int order = std::stoi(argv[2]);
	
	// Set up the points for the spline
	std::vector<Eigen::Vector<double,Eigen::Dynamic>> points;
	std::vector<double> times;
	
	for(int i = 0; i < n; i++)
	{
		Eigen::Vector<double,Eigen::Dynamic> temp(1); temp(0) = i;
		
		points.push_back(temp);
		times.push_back(i);
	}
		
	try
	{
		Spline<double> trajectory(points, times, order);
		
		double hertz = 20.0;
		unsigned int steps = 21*(n-1);
		
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
