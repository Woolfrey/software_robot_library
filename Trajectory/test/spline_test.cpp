/**	
 * @file  : spline_test.cpp
 * @author: Jon Woolfrey
 * @date  : March 2024
 * @brief : Saves data on a plynomial trajectory with waypoints (spline) for analysis.
 */
 
#include <fstream>  
#include <iostream>                                                                                 // std::cerr, std::cout
#include <Math/MathFunctions.h>
#include <time.h>
#include <Trajectory/SplineTrajectory.h>                                                            // We want to test this
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

    unsigned int numberOfWaypoints = std::stoi(argv[1]);                                            // As it says
    unsigned int order = std::stoi(argv[2]);                                                        // Polynomial order

    std::vector<State<double>> trajectoryPoints(numberOfWaypoints);
    
    // Start at zero
    trajectoryPoints.front() = { Eigen::VectorXd::Zero(1),
                                 Eigen::VectorXd::Zero(1),
                                 Eigen::VectorXd::Zero(1) };
    
    // End at 1, but with zero velocity
    trajectoryPoints.back() = { Eigen::VectorXd::Ones(1),
                                Eigen::VectorXd::Zero(1),
                                Eigen::VectorXd::Zero(1) };
                                
    srand(time(NULL));                                                                              // Seed the random number generator
    
    // Set random values for the intermediate points
    for(int i = 1; i < numberOfWaypoints-1; i++)
    {
        trajectoryPoints[i] = { Eigen::VectorXd::Random(1),
                                Eigen::VectorXd::Random(1),
                                Eigen::VectorXd::Random(1) };
    }
    
    // Create the vector of times for each waypoint
    std::vector<double> times;
    for(int i = 0; i < numberOfWaypoints; i++) times.push_back(i);

    // Override the values for the cubic case to test the custom function
    if(order == 3)
    {
        std::vector<double> positions(numberOfWaypoints);
        for(int i = 0; i < positions.size(); i++)
        {
            positions[i] = trajectoryPoints[i].position(0);
        }
        
        std::vector<double> derivatives = solve_cubic_spline_derivatives(positions, times);
        
        for(int i = 0; i < numberOfWaypoints; i++)
        {
            trajectoryPoints[i].velocity(0) = derivatives[i];
        }
    } 
    
    // Sample equispaced points in time along the trajectory
    try
    {
        SplineTrajectory<double> trajectory(trajectoryPoints, times, order);                        // Create the trajectory
        
        double hertz = 50;
        
        unsigned int steps = times.back()*hertz+1;
        
        Eigen::MatrixXd stateData(steps,4);
        for(int i = 0; i < steps; i++)
        {
            double time = i/hertz;
            
            const auto &[pos, vel, acc] = trajectory.query_state(time);
            
            stateData.row(i) << time, pos(0), vel(0), acc(0);
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
        
        // Save the waypoints
        file.open("waypoint_test_data.csv");
        for(int i = 0; i < numberOfWaypoints; i++)
        {
            file << times[i] << "," << trajectoryPoints[i].position(0);
            
            if(i < numberOfWaypoints-1) file << "\n";
        }
        file.close();
            
    }
    catch(const std::exception &exception)
    {
        std::cerr << exception.what() << std::endl;

        return -1;
    }
    
	std::cout << "[INFO] [SPLINE TRAJECTORY TEST] Complete. "
	          << "Data output to 'trajectory_test_data.csv' for analysis.\n";
	          
    return 0;
}
