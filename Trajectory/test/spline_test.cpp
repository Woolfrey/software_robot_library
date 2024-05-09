/**	
 * @file  : spline_test.cpp
 * @author: Jon Woolfrey
 * @date  : March 2024
 * @brief : Saves data on a plynomial trajectory with waypoints (spline) for analysis.
 */
 
#include <fstream>  
#include <iostream>                                                                                 // std::cerr, std::cout
#include <Trajectory/Spline.h>                                                                      // We want to test this
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
	
	unsigned int n = std::stoi(argv[1]);                                                           // Number of waypoints
	unsigned int order = std::stoi(argv[2]);                                                       // Polynomial order
	
	// Set up the points for the spline
	// We need a std::vector of Eigen::Vector objects for the spline.
	// Each Eigen::Vector represents a position, the std::vector collates them as waypoints.
	
	std::vector<Eigen::Vector<double,Eigen::Dynamic>> points;
	std::vector<double> times;
	
	for(int i = 0; i < n; i++)
	{
		Eigen::Vector<double,Eigen::Dynamic> temp(1);                                             // 1D vector
		
          if(i%2 == 0) temp(0) = 1;
          else         temp(0) =-1;
		
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
			double t = (i-1)/hertz;
			
			const auto &[pos, vel, acc] = trajectory.query_state(t);
			
			state(i,0) = pos(0);
			state(i,1) = vel(0);
			state(i,2) = acc(0);
		}
		
		// Save the data to .csv file
	     std::ofstream file;
	     file.open("spline_test_data.csv");
          
          for(int i = 0; i < steps; i++)
          {
               file << i/hertz;                                                                     // Time
               for(int j = 0; j < 3; j++) file << "," << state(i,j);                                // Position, velocity, acceleration
               file << "\n";                                                                        // New line
          }
          
          file.close();                                                                             // As it says on the label
	}
	catch(const std::exception &exception)
	{
		std::cerr << exception.what() << std::endl;
		
		return -1;
	}
	
	return 0;
}
