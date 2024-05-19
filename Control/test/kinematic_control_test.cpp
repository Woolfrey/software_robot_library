/**
 * @file   kinematic_control_test.cpp
 * @author Jon Woolfrey
 * @data   April 2024
 * @brief  A script for testing the SerialPositionControl class.
 */

#include <Eigen/Core>
#include <iostream>
#include <Control/SerialKinematicControl.h>

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
	
	//std::cerr << "Fuck you why won't this run?!" << std::endl;
	
     // Parameters
 
     unsigned int simulationFrequency = 1000;
     unsigned int controlFrequency = 100;
     double simulationDuration = 5.0;
     
     Eigen::VectorXd jointPosition(8);
     jointPosition << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; 
     
     Eigen::VectorXd jointVelocity(8);
     jointVelocity.setZero();

     unsigned int count = 0;
     
     for(int i = 0; i < (int)(simulationDuration*simulationFrequency); i++)
     {
          double simulationTime = (i-1)/simulationFrequency;                                        // Current simulation time
          
          jointPosition += jointVelocity/simulationFrequency;                                       // Update the joint position
          
          // Run the control at 1/10th of the simulation
          if(i%10 == 0)
          {
             //  model.update_state(jointPosition, jointVelocity);                                    // Update kinematics & dynamics
             //  controller.update_state();                                                           // Update the controller
          }
     }
     
     return 0;                                                                                      // No problems with main()
}
