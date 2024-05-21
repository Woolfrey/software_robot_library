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
	
	KinematicTree_d model(argv[1]);                                                                // Create model from urdf
	
	SerialKinematicControl_d controller(&model,"right_hand");                                      // Create controller for the "right hand" frame

     // Set the initial joint state
     Eigen::VectorXd jointPosition(model.number_of_joints());
     jointPosition.setZero(); 
     
     Eigen::VectorXd jointVelocity(model.number_of_joints());
     jointVelocity.setZero();
     
     // Set up the trajectory
     srand(time(NULL));                                                                             // Seed the random number generator
    
     Eigen::VectorXd startPoint = jointPosition;
     Eigen::VectorXd endPoint   = 3*Eigen::VectorXd::Random(model.number_of_joints());
     
     Polynomial_d trajectory(startPoint, endPoint, startTime, endTime, polynomialOrder);            // Create polynomial trajectory
        
     Eigen::MatrixXd positionError(simulationSteps/ratio, model.number_of_joints());                // Record trajectory tracking error (for plotting)
     Eigen::MatrixXd velocities(simulationSteps/ratio,    model.number_of_joints());                // Record joint speeds (for plotting)
     
     unsigned int rowCounter = 0; 
     
     for(int i = 0; i < simulationSteps; i++)
     {
          double simulationTime = i/simulationFrequency;                                            // Current simulation time
          
          jointPosition = jointPosition + jointVelocity/simulationFrequency;                        // Update the joint position
          
          // Run the control at 1/10th of the simulation
          if(i%ratio == 0)
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
     
     std::cout << "[INFO] [KINEMATIC JOINT CONTROL TEST]: "
               << "Numerical simulation complete. Data saved to .csv file for analyis.\n";
     
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
