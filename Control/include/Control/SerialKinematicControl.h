/**
 * @file    SerialKinematicControl.h
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    February 2025
 * @version 1.0
 * @brief   Computes velocity (position) feedback control for a serial link robot arm.
 * 
 * @details This class contains methods for performing velocity control of a serial link robot arm
 *          in both Cartesian and joint space. The fundamental feedforward + feedback control is given by:
 *          control velocity = desired velocity + gain * (desired position - actual position).
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 */

#ifndef SERIALKINEMATICCONTROL_H_
#define SERIALKINEMATICCONTROL_H_

#include "RobotLibrary/Control/SerialLinkBase.h"

namespace RobotLibrary { namespace Control {

/**
 * @brief Algorithms for velocity control of a serial link robot arm.
 */
class SerialKinematicControl : public SerialLinkBase
{
	public:
		/**
		 * @brief Constructor.
		 * @param model A pointer to a KinematicTree object.
		 * @param endpointName The name of the reference frame in the KinematicTree to be controlled.
		 */
		SerialKinematicControl(RobotLibrary::Model::KinematicTree *model,
		                       const std::string &endpointName,
		                       const double &controlFrequency = 100.0)
		                       : SerialLinkBase(model, endpointName, controlFrequency){}
		
		/**
		 * @brief Solve the joint velocities required to move the endpoint at a given speed.
		 * @param endpointMotion A twist vector (linear & angular velocity).
		 * @return A nx1 vector of joint velocities.
		 */
		Eigen::VectorXd
		resolve_endpoint_motion(const Eigen::Vector<double,6> &endPointMotion);
		
		/**
		 * @brief Solve the joint velocities required to track a Cartesian trajectory.
		 * @param desiredPose The desired position & orientation (pose) for the endpoint.
		 * @param desiredVel The desired linear & angular velocity (twist) for the endpoint.
		 * @param desiredAcc Not used in velocity control.
		 * @return The joint velocities (nx1) required to track the trajectory.
		 */
		Eigen::VectorXd
		track_endpoint_trajectory(const RobotLibrary::Model::Pose &desiredPose,
					              const Eigen::Vector<double,6>   &desiredVelocity,
					              const Eigen::Vector<double,6>   &desiredAcceleration);

		/**
		 * @brief Solve the joint velocities required to track a joint space trajectory.
		 * @param desiredPos The desired joint position (nx1).
		 * @param desiredVel The desired joint velocity (nx1).
		 * @param desiredAcc Not used in velocity control.
		 * @return The control velocity (nx1).
           */			  
		Eigen::VectorXd
		track_joint_trajectory(const Eigen::VectorXd &desiredPosition,
		                       const Eigen::VectorXd &desiredVelocity,
		                       const Eigen::VectorXd &desiredAcceleration);
													   		
	protected:
	
		/**
		 * @brief Compute the instantaneous limits on the joint velocity control.
		 *        It computes the minimum between joint positions, speed, and acceleration limits.
		 * @param jointNumber The joint for which to compute the limits.
		 * @return The upper and lower joint speed at the current time.
		 */
		RobotLibrary::Model::Limits
		compute_control_limits(const unsigned int &jointNumber);
	
};                                                                                                  // Semicolon needed after a class declaration

} }

#endif
