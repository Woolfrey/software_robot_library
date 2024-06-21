/**	
 * @file  : trapezoidal_test.cpp
 * @author: Jon Woolfrey
 * @date  : June 2024
 * @brief : Tests the output of the trapezoidal velocity trajectory.
 */
 
#include <fstream>
#include <iostream>                                                                                 // std::cerr, std::cout
#include <Trajectory/TrapezoidalVelocity.h>                                                         // We want to test this
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
	
	unsigned int n = std::stoi(argv[1]);                                                            // Number of waypoints
	double maxVel = std::stod(argv[2]);                                                             
	double maxAcc = std::stod(argv[3]);
	
    srand(time(NULL));                                                                              // Seed the random number generator
	
	// Set up the points for the spline
	std::vector<Eigen::Vector<double,Eigen::Dynamic>> points;
	points.push_back(Eigen::VectorXd::Zero(1));                                                     // Start at zero
	
	for(int i = 0; i < n-1; i++)
	{
	    points.push_back(Eigen::VectorXd::Random(1));                                               // Random intermediate point
    }
    
    points.push_back(Eigen::VectorXd::Ones(1));                                                     // End at one
		
	try
	{
		TrapezoidalVelocity<double> trajectory(points,maxVel,maxAcc,0);

		double hertz = 20.0;
		
		unsigned int steps = (int)(trajectory.end_time()*hertz+3);

		Eigen::MatrixXd stateData(steps,4);
		
		for(int i = 0; i < steps; i++)
		{
			double time = (i-1)/hertz;
			
			const auto &[pos, vel, acc] = trajectory.query_state(time);
			
			stateData.row(i) << time, pos, vel, acc;
		}
		
		// Save the data to .csv file
        std::ofstream file;
        file.open("trajectory_test_data.csv");

        for(int i = 0; i < steps; i++)
        {
            for(int j = 0; j < 4; j++)
            {
                file << stateData(i,j);
                if(j < 3) file << ",";
                else      file << "\n";
            }
        }

        file.close();                                                                               // As it says on the label
	}
	catch(const std::exception &exception)
	{
		std::cerr << exception.what() << std::endl;
		
		return -1;
	}
	
	
	std::cout << "[INFO] [MULTI-TRAPEZOIDAL TEST] Complete. "
	          << "Data saved to 'trajectory_test_data.csv' for analysis.\n";
	
	return 0;
}
