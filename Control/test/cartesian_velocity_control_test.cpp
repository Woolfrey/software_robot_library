/**
 * @file cartesian_velocity_control_test.cpp
 * @author Jon Woolfrey
 * @date May 2024
 * @brief Runs a numerical simulation to check the resolved motion rate control of a serial link robot.
 */

#include <Eigen/Core>                                                                               // Eigen::Vector, Eigen::Matrix classes
#include <fstream>                                                                                  // Reading and writing to files
#include <iostream>                                                                                 // std::cout, std::cerr
#include <Control/SerialKinematicControl.h>                                                         // Custom control class
#include <Trajectory/CartesianTrajectory.h>                                                         // Custom trajectory generator
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

int main(int argc, char** argv)
{
     // Default for argc is 1 but I don't know why ┐(ﾟ ～ﾟ )┌
	if(argc != 3)
	{
		std::cerr << "[ERROR] [URDF TEST] No path to file was given. "
		          << "Usage: ./kinematic_control_test /path/to/file.urdf endpoint_name\n";
	         
		return -1;                                                                                // Exit main() with error
	}
	
	srand(time(NULL));                                                                             // Seed the random number generator	
	
     // Set up the controller
     KinematicTree_d model(argv[1]);                                                                // Create the model from urdf
     
     SerialKinematicControl_d controller(&model, argv[2]);                                          // Create controller for given endpoint
     
     unsigned int n = model.number_of_joints();
     
     Eigen::VectorXd jointPosition = Eigen::VectorXd::Random(n);                                    // Set a random start configuration
     Eigen::VectorXd jointVelocity = Eigen::VectorXd::Zero(n);                                      // Start at rest
     
     model.update_state(jointPosition, jointVelocity);                                              // Updates the forward kinematics
     controller.update();                                                                           // Updates properties specific to this controller
     
     // Set up the Cartesian trajectory
     Pose_d startPose = controller.endpoint_pose();                                                 // Get the current endpoint pose
     
     Eigen::Vector3d offset = {0.0, 0.1, 0.0};                                                      // Translation in y-direction
     
     Pose_d endPose(startPose.translation() + offset, startPose.quaternion());                      // Offset the start pose
     
     CartesianTrajectory<double,Spline<double>> trajectory(startPose, endPose, startTime, endTime); // Create the trajectory
     
     // Establish arrays for saving data
     unsigned int m = simulationSteps/ratio;
     
     Eigen::MatrixXd positionArray(m,n);
     Eigen::MatrixXd velocityArray(m,n);
     Eigen::MatrixXd positionErrorArray(m,n);
     
     unsigned int rowCounter = 0;                                                                   // For indexing across arrays
     
     for(int i = 0; i < simulationSteps; i++)
     {
          double simulationTime = i/simulationFrequency;                                            // Current simulation time
          
          jointPosition += jointVelocity/simulationFrequency;                                       // Update the joint position
          
          // Run the control at 1/10th of the simulation
          if(i%ratio == 0)
          { 
               model.update_state(jointPosition, jointVelocity);                                    // Update kinematics & dynamics
               controller.update();                                                                 // Update the controller
               
               CartesianState state = trajectory.query_state(simulationTime);
               
               // NOTE: This is also feasible, but the const casting means we cannot print
               //       data to the console:
               // const auto &[desiredPose,
               //              desiredTwist,
               //              desiredAcceleration] = trajectory.query_state(simulationTime);
                
               // std::cout << state.pose.as_matrix() << "\n\n";
               
               // jointVelocity = controller.track_endpoint_trajectory(state.pose, state.twist, state.acceleration);
                          
               /*                                                
               // Record data
               positionArray.row(rowCounter)      = jointPosition.transpose();
               velocityArray.row(rowCounter)      = jointVelocity.transpose();
               positionErrorArray.row(rowCounter) = (desiredPosition - jointPosition).transpose();
               
               rowCounter++;
               */
          }
     }
     
     /*
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
               const auto &[lower, upper] = model.joint(j).position_limits();
               double velocity = model.joint(j).speed_limit();
               
               file << lower << "," << upper << "," << velocity << "\n";
          }
     }
     file.close();
     
     std::cout << "[INFO] [KINEMATIC JOINT CONTROL TEST]: "
               << "Numerical simulation complete. Data saved to .csv file for analyis.\n";
     */
     
     return 0;                                                                                      // No problems with main()
}
