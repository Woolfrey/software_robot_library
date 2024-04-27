/**
 * @file   kinematic_control_test.cpp
 * @author Jon Woolfrey
 * @data   April 2024
 * @brief  A script for testing the SerialPositionControl class.
 */

#include <SerialKinematicControl.h>

int main(int argc, char** argv)
{
     // Default for argc is 1 but I don't know why ┐(ﾟ ～ﾟ )┌
	if(argc != 2)
	{
		std::cerr << "[ERROR] [URDF TEST] No path to file was given. "
		          << "Usage: ./kinematic_control_test /path/to/file.urdf\n";
		         
		return 1;
	}
	
	KinematicTree<double> model(argv[1]);                                                          // Create model from urdf
	
	SerialKinematicControl<double> controller(&model,"right_hand");
	
     return 0;                                                                                      // No problems with main()
}
