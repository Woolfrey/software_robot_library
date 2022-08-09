    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                        A class for velocity control of a serial link robot                     //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef SERIALKINCONTROL_H_
#define SERIALKINCONTROL_H_

#include <algorithm>                                                                                // std::min, std::max
#include <array>                                                                                    // std::array
#include <QPSolver.h>
#include <SerialLink.h>                                                                             // Custom robot class
#include <vector>                                                                                   // std::vector

class SerialKinControl : public SerialLink,
                         public QPSolver
{
	public:
		SerialKinControl(const std::vector<RigidBody> links,
                                 const std::vector<Joint> joints,
                                 const float &controlFrequency);                                    // Constructor
		
		// Set Functions
		bool set_proportional_gain(const float &gain);                                      // Set the gain used in feedback control
		
		// Get Functions
		Eigen::VectorXf get_pose_error(const Eigen::Isometry3f &desired,                    // Get pose error as a 6x1 vector
		                               const Eigen::Isometry3f &actual);
		
		Eigen::VectorXf move_at_speed(const Eigen::VectorXf &vel);                          // Move endpoint at a given speed
		
		Eigen::VectorXf move_at_speed(const Eigen::VectorXf &vel,                           // Move endpoint at given speed...
		                              const Eigen::VectorXf &redundant);                    // ... specify the redundant task
		
		Eigen::VectorXf move_to_position(const Eigen::VectorXf &pos);                       // Move joints toward a desired position
		
		Eigen::VectorXf move_to_pose(const Eigen::Isometry3f &pose);                        // Move endpoint toward a desired pose
		
		Eigen::VectorXf move_to_pose(const Eigen::Isometry3f &pose,                         // Move endpoint toward a desired pose
		                             const Eigen::VectorXf &redundancy);                    // Specify redundant joint velocities
		                             
		Eigen::VectorXf track_cartesian_trajectory(const Eigen::Isometry3f &pose,           // Track Cartesian trajectory defined by pose...
		                                           const Eigen::VectorXf &vel);             // and instantaneous velocity
		
		Eigen::VectorXf track_cartesian_trajectory(const Eigen::Isometry3f &pose,           // Track Cartesian trajectory defined by pose...
		                                           const Eigen::VectorXf &vel,              // instantaneous velocity, and ...
		                                           const Eigen::VectorXf &redundancy);      // specify redundant task
		                                           
		Eigen::VectorXf track_joint_trajectory(const Eigen::VectorXf &pos,                  // Track trajectory defined by position...
		                                       const Eigen::VectorXf &vel);		    // and instantaneous joint velocity	
		
	private:
		// Functions
		bool get_speed_limit(float &lower, float &upper, const int &i);                     // Get instantaneous speed limit of a joint
		
		float get_joint_penalty(const int &i);                                              // For joint limit avoidance in redundant robot
		
		Eigen::VectorXf singularity_avoidance(const float &scalar);                         // Returns gradient of manipulability
		
		Eigen::VectorXf solve_joint_control(const Eigen::VectorXf &y,                       // Used in move_at_speed() function
		                                    const Eigen::MatrixXf &U);
		
		// Variables
		float k     = 1.0;                                                                  // Proportional gain
		float dt    = 1/100;                                                                // Discrete time step, for control purposes
		float hertz = 100;                                                                  // Control frequency
		std::vector<std::array<float,2>> pLim;                                              // Position limits for all the joints
		std::vector<float> vLim;                                                            // Velocity limits for all the joints
		std::vector<float> aLim;                                                            // Acceleration limits for all the joints
		
};                                                                                                  // Semicolon needed after a class declaration

#endif
