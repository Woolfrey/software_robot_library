/**	
 * @file  : polynomial_test.cpp
 * @author: Jon Woolfrey
 * @date  : March 2024
 * @brief : Saves data on a polynomial trajectory for analysis.
 */
 
#include <fstream>                                                                                  // For opening/creating/saving text files
#include <iostream>                                                                                 // std::cerr, std::cout
#include <Trajectory/PolynomialTrajectory.h>                                                        // We want to test this
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
		          
	
		return -1;                                                                                  // Failure
	}
	
	using namespace Eigen;
	
	State<float> startPoint = { Eigen::VectorXf::Zero(1),                                         // Position
	                            Eigen::VectorXf::Zero(1),                                           // Velocity
	                            Eigen::VectorXf::Zero(1) };                                         // Acceleration
    
    State<float> endPoint = { Eigen::VectorXf::Ones(1),                                           // Position
                              Eigen::VectorXf::Zero(1),                                             // Velocity
                              Eigen::VectorXf::Zero(1) };                                           // Acceleration
	
	try
	{
		unsigned int order = std::stoi(argv[1]);                                                    // Convert to int
		
        PolynomialTrajectory<float> trajectory(startPoint, endPoint, 0, 1, order);
	    
		float hertz = 50.0;
		
		unsigned int steps = 51;
		
		Eigen::MatrixXf stateData(steps,4);
		
		// Run a numerical simulation
		for(int i = 0; i < steps; i++)
		{
			float t = i/hertz;
			
			const auto &[pos, vel, acc] = trajectory.query_state(t);
			
			stateData.row(i) << t, pos, vel, acc;                                                   // Save data to array
		}
		
		// Output the data to .csv for analysis
		std::ofstream file; file.open("trajectory_test_data.csv");
		
		for(int i = 0; i < stateData.rows(); i++)
		{
		    for(int j = 0; j < stateData.cols(); j++)
		    {
		        file << stateData(i,j);
		        
		        if(j < stateData.cols()-1) file << ",";
		        else                       file << "\n";
	        }
        }
        
        file.close();
    }
	catch(const std::exception &exception)
	{
		std::cerr << exception.what() << std::endl;
		
		return -1;
	}
	
	std::cout << "[INFO] [POLYNOMIAL TRAJECTORY TEST] Complete. "
	          << "Data output to 'trajectory_test_data.csv' for analysis.\n";
	
	return 0;
}
