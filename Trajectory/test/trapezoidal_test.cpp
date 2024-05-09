/**	
 * @file  : trapezoidal_test.cpp
 * @author: Jon Woolfrey
 * @date  : March 2024
 * @brief : Saves data on a trapezoidal velocity trajectory for analysis.
 */
 
#include <fstream>                                                                                  // For opening/creating/saving text files
#include <iostream>                                                                                 // std::cerr, std::cout
#include <Trajectory/TrapezoidalVelocity.h>                                                         // We want to test this
#include <string>
 
  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                           MAIN                                                 //
////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
	if(argc != 3)
	{
		std::cerr << "[ERROR] [TRAPEZOIDAL TEST] Incorrect number of input arguments. "
		          << "Usage: './trapezoidal_test maxVel maxAccel' ." << std::endl;
		          
	
		return -1;                                                                           // Failure
	}
	
	Eigen::Vector<float,1> startPosition; startPosition(0) = 0;
	Eigen::Vector<float,1> endPosition;   endPosition(0)   = 1;
	Eigen::Vector<float,1> startVelocity; startVelocity(0) = 0;
	Eigen::Vector<float,1> endVelocity;   endVelocity(0)   = 0;
	
	try
	{
		unsigned int order = std::stoi(argv[1]);
		
		float maxVel = std::stof(argv[1]);
		float maxAcc = std::stof(argv[2]);
		float startTime = 0.0;
		
		TrapezoidalVelocity<float> trajectory(startPosition,endPosition,maxVel,maxAcc,startTime);
	
		float hertz = 50.0;
		
		unsigned int steps = (int)(trajectory.duration()*hertz) + 3;
		
		Eigen::MatrixXf state(steps,3);
		
		for(int i = 0; i < steps; i++)
		{
			float t = (i-1)/hertz;
			
			const auto &[pos, vel, acc] = trajectory.query_state(t);
			
			state(i,0) = pos(0);
			state(i,1) = vel(0);
			state(i,2) = acc(0);
		}
		
		// Save the data to .csv file
	     std::ofstream file;
	     file.open("trapezoidal_test_data.csv");
          
          for(int i = 0; i < steps; i++)
          {
               file << i/hertz;                                                                     // Time
               for(int j = 0; j < 3; j++) file << "," << state(i,j);                                // Position, velocity, acceleration
               file << "\n";                                                                        // New line
          }
          
          file.close();
		
	}
	catch(const std::exception &exception)
	{
		std::cerr << exception.what() << std::endl;
		
		return -1;
	}
	
	return 0;
}
