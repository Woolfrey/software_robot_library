    ////////////////////////////////////////////////////////////////////////////////////////////////////
   //                                                                                                //
  //                        A class for velocity control of a serial link robot                     //
 //                                                                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef SERIALKINCONTROL_H_
#define SERIALKINCONTROL_H_

#include <algorithm>                                                                                // std::min, std::max
#include <array>                                                                                    // std::array
#include <SerialLink.h>                                                                             // Custom robot class
#include <vector>                                                                                   // std::vector

class SerialKinControl : public SerialLink
{
	public:
		SerialKinControl(const std::vector<RigidBody> links,
                                const std::vector<Joint> joints,
                                const float &controlFrequency);                                     // Constructor
		
		// Set Functions
		bool set_proportional_gain(const float &gain);                                      // Set the gain used in feedback control
		
		// Get Functions
		
		Eigen::MatrixXf get_inverse(const Eigen::MatrixXf &A);                              // Get the inverse of a matrix
		
		Eigen::MatrixXf get_weighted_inverse(const Eigen::MatrixXf &A,                      // Get the weighted inverse of a matrix
                                                     const Eigen::MatrixXf &W);
		
		Eigen::VectorXf get_cartesian_control(const Eigen::VectorXf &vel);                  
		
		Eigen::VectorXf get_cartesian_control(const Eigen::VectorXf &vel,
		                                      const Eigen::VectorXf &secondaryTask);
		                                      
		Eigen::VectorXf get_cartesian_control(const Eigen::Isometry3f &pose,
                                                      const Eigen::VectorXf &vel);
		
		Eigen::VectorXf get_cartesian_control(const Eigen::Isometry3f &pose,
		                                      const Eigen::VectorXf &vel,
		                                      const Eigen::VectorXf &secondaryTask);
		                                      
		Eigen::VectorXf get_joint_control(const Eigen::VectorXf &pos,
                                                  const Eigen::VectorXf &vel);
		
		Eigen::VectorXf get_pose_error(const Eigen::Isometry3f &desired,
                                               const Eigen::Isometry3f &actual);				
		
	private:
		// Functions
		bool limit_joint_velocity(float &qdot, const int &i);                               // Ensure kinematic feasiblity of joint control
		
		bool primary_task_is_ok(const Eigen::VectorXf &task);                               // Ensure input vector is 6x1
		
		bool secondary_task_is_ok(const Eigen::VectorXf &task);                             // Ensure input vector is nx1
		
		Eigen::MatrixXf get_joint_weighting();                                              // Used for joint limit avoidance
		
		Eigen::VectorXf rmrc(const Eigen::VectorXf &xdot,                                   // Cartesian velocity control algorithm
                                     const Eigen::VectorXf &qdot_d);
		
		Eigen::VectorXf singularity_avoidance(const float &scalar);                         // Returns gradient of manipulability
		
		// Variables
		double k = 1.0;                                                                     // Proportional gain
		float dt = 1/100;                                                                   // Discrete time step, for control purposes
		std::vector<std::array<float,2>> pLim;                                              // Position limits for all the joints
		std::vector<float> vLim;                                                            // Velocity limits for all the joints
		std::vector<float> aLim;                                                            // Acceleration limits for all the joints
		
};                                                                                                  // Semicolon needed after a class declaration

#endif
