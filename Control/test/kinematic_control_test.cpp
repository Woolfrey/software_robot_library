/**
 * @file   kinematic_control_test.cpp
 * @author Jon Woolfrey
 * @data   April 2024
 * @brief  A script for testing the SerialPositionControl class.
 */

#include <Eigen/Core>                                                                               // Eigen::Vector, Eigen::Matrix classes
#include <fstream>                                                                                  // Reading and writing to files
#include <iostream>                                                                                 // std::cout, std::cerr
#include <Control/SerialKinematicControl.h>                                                         // Custom control class
#include <Trajectory/Polynomial.h>                                                                  // Custom trajectory generator

// Parameters for the simulation:
double simulationFrequency = 1000.0;
double controlFrequency = 100;
double simulationDuration = 5.0;


int main(int argc, char** argv)
{
     // Default for argc is 1 but I don't know why ┐(ﾟ ～ﾟ )┌
	if(argc != 2)
	{
		std::cerr << "[ERROR] [URDF TEST] No path to file was given. "
		          << "Usage: ./kinematic_control_test /path/to/file.urdf\n";
		         
		return 1;
	}
	
	KinematicTree_d model(argv[1]);                                                                // Create model from urdf
	
	SerialKinematicControl_d controller(&model,"right_hand");                                      // Create controller for the "right hand" frame

     // Set the initial joint state
     Eigen::VectorXd jointPosition(model.number_of_joints());
     jointPosition.setZero(); 
     
     Eigen::VectorXd jointVelocity(model.number_of_joints());
     jointVelocity.setZero();
     
     // Set up the trajectory
     Polynomial_d trajectory(jointPosition,                                                         // Start point
                             Eigen::VectorXd::Random(model.number_of_joints()),                     // Set a random endpoint
                             0.0,                                                                   // Start time
                             simulationDuration,                                                    // End time
                             3);                                                                    // Order of the polynomial
     
     // Run the numerical simulation
     unsigned int simulationSteps = simulationDuration*simulationFrequency;
     
     Eigen::MatrixXd positionError(simulationSteps/10,model.number_of_joints());                    // Record trajectory tracking error (for plotting)
          
     Eigen::MatrixXd velocities(simulationSteps/10,model.number_of_joints());                       // Record joint speeds (for plotting)
     
     unsigned int rowCounter = 0;
     
     for(int i = 0; i < simulationSteps; i++)
     {
          double simulationTime = (i-1)/simulationFrequency;                                        // Current simulation time
          
          jointPosition += jointVelocity/simulationFrequency;                                       // Update the joint position
          
          // Run the control at 1/10th of the simulation
          if(i%10 == 0)
          {
               model.update_state(jointPosition, jointVelocity);                                    // Update kinematics & dynamics
               controller.update_state();                                                           // Update the controller

               const auto &[desiredPosition,
                            desiredVelocity,
                            desiredAcceleration] = trajectory.query_state(simulationTime);
               
               jointVelocity = controller.track_joint_trajectory(desiredPosition,
                                                                 desiredVelocity,
                                                                 desiredAcceleration);
                                                                 
               // Record data
               positionError.row(rowCounter) = (desiredPosition - jointPosition).transpose();
               velocities.row(rowCounter)    = jointVelocity.transpose();
               rowCounter++;
          }
     }
     
     // Put the data in to .csv files for analysis, plotting etc.
     std::ofstream file;
     file.open("joint_position_error_data.csv");
     for(int i = 0; i < positionError.rows(); i++)
     {
          file << (double)(i/controlFrequency);                                                     // Time
          for(int j = 0; j < positionError.cols(); j++) file << "," << positionError(i,j);
          file << "\n";
     }
     file.close();
     
     return 0;                                                                                      // No problems with main()
}
