#include <iostream>                                                                                 // std::cerr, std::cout
#include <fstream>
#include <KinematicTree.h>                                                                          // Custom class for robot physics
#include <time.h>                                                                                   // Timer

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                          MAIN                                                 //
///////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
	if(argc != 2)
	{
		std::cerr << "[ERROR] [URDF TEST] No path to file was given. "
		          << "Usage: ./urdf_test /path/to/file.urdf" << std::endl;
		         
		return 1;
	}
	
	std::string pathToURDF = argv[1];
	
	try
	{
		KinematicTree<float> model(pathToURDF);                                                   
		
		int numJoints = model.number_of_joints();
	
		clock_t timer;
		
		timer = clock();                                                                          // Start the timer

		srand((unsigned int) time(0));					                                   // Random seed generator

		VectorXf q    = VectorXf::Random(numJoints);
		VectorXf qdot = VectorXf::Random(numJoints);

		if(not model.update_state(q, qdot))
		{
			std::cerr << "[ERROR] [URDF TEST] Couldn't update the state for some reason." << std::endl;
		}
		
		timer = clock() - timer;                                                                  // Difference from the start
		
		float time = (float)timer/CLOCKS_PER_SEC;                                                 // Convert
		
		std::cout << "\nIt took " << time*1000 << " milliseconds (" << 1/time << " Hz) "
		          << "to compute the inverse dynamics.\n";

		if(model.name() == "sawyer")
		{
			std::cout << "\nHere is the pose of the 'right_hand' frame:\n\n";
			std::cout << model.frame_pose("right_hand").as_matrix() << "\n";
			
			std::cout << "\nHere is the jacobian matrix of the 'right_hand' frame:\n\n";
			std::cout << model.jacobian("right_hand") << "\n";
		}

		std::cout << "\nHere is the inertia matrix:\n\n";
		std::cout << model.joint_inertia_matrix() << std::endl;
		
		std::cout << "\nHere is the joint Coriolis matrix:\n\n";
		std::cout << model.joint_coriolis_matrix() << std::endl;

        	std::cout << "\nHere is the joint Coriolis vector:\n\n";
       	 	std::cout << (model.joint_coriolis_matrix()*qdot).transpose() << std::endl;

		std::cout << "\nHere is the joint gravity torque vector:\n\n";
		std::cout << model.joint_gravity_vector().transpose() << std::endl;
		
		if(model.name() == "parody-mark1")
		{
			std::cout << "\nHere is the inertial coupling between the base and joints:\n\n";
			std::cout << model.joint_base_inertia_matrix() << std::endl;
		}
		
		/*
		clock_t timer;
		
		std::ofstream out("inverse_dynamics_data.csv");
		
		srand((unsigned int) time(0));
		
		for(int i = 0; i < 200; i++)
		{
			timer = clock();
			
			if(not model.update_state(0.1*Eigen::VectorXf::Random(numJoints),
                                                  0.1*Eigen::VectorXf::Random(numJoints)))
			{
				std::cout << "FAILURE.\n";
				
				return 1;
			}
			
			timer = clock() - timer;
			
			out << (float)timer/CLOCKS_PER_SEC << "\n";
		}*/
			
		return 0;
	}
	catch(std::exception &error)
	{
		std::cerr << "[ERROR] [URDF TEST] There was a problem constructing the KinematicTree object. "
		          << "See the error message below for details.\n";
		                    
		std::cerr << error.what() << std::endl;
	
		return 1;
	}
}
