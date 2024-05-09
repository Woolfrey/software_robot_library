/**	
 * @file  : multi_trapezoid_test.cpp
 * @author: Jon Woolfrey
 * @date  : March 2024
 * @brief : Saves data on trapezoidal velocity trajectory with waypoints.
 */
 
#include <fstream>
#include <iostream>                                                                                 // std::cerr, std::cout
#include <Trajectory/MultiTrapezoid.h>                                                              // We want to test this
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
		Eigen::Vector<double,Eigen::Dynamic> temp(1);                                             // 1D vector
		
          if(i%2 == 0) temp(0) = 1;
          else         temp(0) =-1;
		
		points.push_back(temp);
	}
		
	try
	{
		MultiTrapezoid<double> trajectory(points,maxVel,maxAcc,0);
		
		double hertz = 20.0;
		
		unsigned int steps = (int)(trajectory.end_time()*hertz+3);
		
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
	     file.open("multi_trapezoid_test_data.csv");
          
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
