/**
 * @file   joint_velocity_control_test.cpp
 * @author Jon Woolfrey
 * @date   May 2024
 * @brief  A script for testing the SerialPositionControl class.
 */

#include <Eigen/Core>                                                                               // Eigen::Vector, Eigen::Matrix classes
#include <fstream>                                                                                  // Reading and writing to files
#include <iostream>                                                                                 // std::cout, std::cerr
#include <Control/SerialKinematicControl.h>                                                         // Custom control class
#include <Trajectory/Polynomial.h>                                                                  // Custom trajectory generator
#include <time.h>

// Parameters for the numerical simulation
double simulationFrequency = 1000;
unsigned int ratio         = 10;
double controlFrequency    = simulationFrequency/ratio;
double simulationDuration  = 3.0;
unsigned int simulationSteps = simulationDuration*simulationFrequency;

// Parameters for the trajectory
double startTime = 0.0;
double endTime = simulationDuration - 0.5;
int polynomialOrder = 5;


int main(int argc, char** argv)
{
     // Default for argc is 1 but I don't know why ┐(ﾟ ～ﾟ )┌
	if(argc != 2)
	{
		std::cerr << "[ERROR] [URDF TEST] No path to file was given. "
		          << "Usage: ./kinematic_control_test /path/to/file.urdf\n";
		         
		return 1;
	}
	
	// Set up the robot model
	KinematicTree_d model(argv[1]);                                                                // Create model from urdf
	
	SerialKinematicControl_d controller(&model,"right_hand");                                      // Create controller for the "right hand" frame

     Eigen::VectorXd jointPosition(model.number_of_joints()); jointPosition.setZero();              // Set the initial joint positions
     
     Eigen::VectorXd jointVelocity(model.number_of_joints()); jointVelocity.setZero();              // Set the initial joint velocities
     
     // Set up the trajectory
     srand(time(NULL));                                                                             // Seed the random number generator
    
     Polynomial_d trajectory(jointPosition,
                             3*Eigen::VectorXd::Random(model.number_of_joints()),
                             startTime, endTime, polynomialOrder);
                             
     unsigned int m = simulationSteps/ratio;
     unsigned int n = model.number_of_joints();
     
     Eigen::MatrixXd positionArray(m,n);
     Eigen::MatrixXd velocityArray(m,n);
     Eigen::MatrixXd positionErrorArray(m,n);
     
     unsigned int rowCounter = 0;
     
     for(int i = 0; i < simulationSteps; i++)
     {
          double simulationTime = i/simulationFrequency;                                            // Current simulation time
          
          jointPosition += jointVelocity/simulationFrequency;                                       // Update the joint position
          
          // Run the control at 1/10th of the simulation
          if(i%ratio == 0)
          { 
               model.update_state(jointPosition, jointVelocity);                                    // Update kinematics & dynamics
               controller.update();                                                                 // Update the controller

               const auto &[desiredPosition,
                            desiredVelocity,
                            desiredAcceleration] = trajectory.query_state(simulationTime);
                            
               jointVelocity = controller.track_joint_trajectory(desiredPosition,
                                                                 desiredVelocity,
                                                                 desiredAcceleration);
                                                                 
               // Record data
               positionArray.row(rowCounter)      = jointPosition.transpose();
               velocityArray.row(rowCounter)      = jointVelocity.transpose();
               positionErrorArray.row(rowCounter) = (desiredPosition - jointPosition).transpose();
               
               rowCounter++;
          }
     }
    
     std::ofstream file;

     // Save the position data
     file.open("joint_position_data.csv");
     for(int i = 0; i < m; i++)
     {
          file << (double)(i/controlFrequency);
          for(int j = 0; j < n; j++) file << "," << positionArray(i,j);
          file << "\n";
     }
     file.close();
     
     // Save the velocity data
     file.open("joint_velocity_data.csv");
     for(int i = 0; i < m; i++)
     {
          file << (double)(i/controlFrequency);
          for(int j = 0; j < n; j++) file << "," << velocityArray(i,j);
          file << "\n";
     }
     file.close();
     
     // Save position error tracking data
     file.open("joint_position_error_data.csv");
     for(int i = 0; i < m; i++)
     {
          file << (double)(i/controlFrequency);
          for(int j = 0; j < n; j++) file << "," << positionErrorArray(i,j);
          file << "\n";
     }
     file.close();
     
     // Save position limits
     file.open("joint_limits.csv");
     {
          for(int j = 0; j < n; j++)
          {
               std::string name = model.joint(j).name();
               const auto &[lower, upper] = model.joint(j).position_limits();
               double velocity = model.joint(j).speed_limit();
               
               file << name << "," << lower << "," << upper << "," << velocity << "\n";
          }
     }
     file.close();
     
     std::cout << "[INFO] [KINEMATIC JOINT CONTROL TEST]: "
               << "Numerical simulation complete. Data saved to .csv file for analyis.\n";
     
     return 0;                                                                                      // No problems with main()
}
