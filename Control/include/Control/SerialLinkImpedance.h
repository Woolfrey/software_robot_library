/**
 * @file    SerialLinkImpedance.h
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    August 2025
 * @version 1.0
 *
 * @brief   Implements control laws for joint and Cartesian impedance control.
 * 
 * @details This class implements dynamic-free joint and Cartesian impedance control.
 *          It deliberately avoids the use of the robot inertia because of poor modeling in many
 *          robot arms. The SerialLinkDynamics should be used if a more accurate model is available.
 * 
 * @copyright (c) 2025 Jon Woolfrey
 *
 * @license   OSCL - Free for non-commercial open-source use only.
 *            Commercial use requires a license.
 *
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 * @see https://github.com/Woolfrey/software_simple_qp for the optimisation algorithm used in the control.
 */

#ifndef SERIAL_LINK_IMPEDANCE_H
#define SERIAL_LINK_IMPEDANCE_H

#include <Control/SerialLinkBase.h>

#include <memory>

namespace RobotLibrary { namespace Control {

/**
 * @brief Algorithms for velocity control of a serial link robot arm.
 */
class SerialLinkImpedance : public SerialLinkBase
{
	public:
		/**
		 * @brief Constructor.
		 * @param model A pointer to a KinematicTree object.
		 * @param endpointName The name of the reference frame in the KinematicTree to be controlled.
		 * @pram parameters Control gains and algorithm parameters.
		 */
		SerialLinkImpedance(std::shared_ptr<RobotLibrary::Model::KinematicTree> model,
		                    const std::string &endpointName,
		                    const RobotLibrary::Control::SerialLinkParameters &parameters = SerialLinkParameters());
		
		/**
		 * @brief Solve the joint torques required to move the endpoint at a given acceleration.
		 * @param endpointMotion A vector containing linear & angular acceleration.
		 * @return A vector of joint torques (nx1)
		 */
		Eigen::VectorXd
		resolve_endpoint_motion(const Eigen::Vector<double,6> &endPointMotion);
		
		/**
         * @brief Compute the joint torque required to execute the specified endpoint twist (linear & angular velocity).
                  This overrides the virtual method defined in the base class.
         * @param twist The desired linear & angular velocity, as a 6D vector.
         * @return The required joint torque.
         */
        Eigen::VectorXd
        resolve_endpoint_twist(const Eigen::Vector<double,6> &twist);
		
		/**
		 * @brief Solve the joint torques required to track a Cartesian trajectory.
		 * @param desiredPose The desired position & orientation (pose) for the endpoint.
		 * @param desiredVel The desired linear & angular velocity (twist) for the endpoint.
		 * @param desiredAcc The desired linear & angular acceleration for the endpoint.
		 * @return The joint torques (nx1) required to track the trajectory.
		 */
		Eigen::VectorXd
		track_endpoint_trajectory(const RobotLibrary::Model::Pose &desiredPose,
					              const Eigen::Vector<double,6>   &desiredVelocity,
					              const Eigen::Vector<double,6>   &desiredAcceleration);

		/**
		 * @brief Solve the joint torques required to track a joint space trajectory.
		 * @param desiredPos The desired joint position (nx1).
		 * @param desiredVel The desired joint velocity (nx1).
		 * @param desiredAcc The desired joint acceleration (nx1).
		 * @return The control torque (nx1).
         */			  
		Eigen::VectorXd
		track_joint_trajectory(const Eigen::VectorXd &desiredPosition,
		                       const Eigen::VectorXd &desiredVelocity,
		                       const Eigen::VectorXd &desiredAcceleration);
		                       
        /**
         * @brief Convert a desired endpoint wrench to the required joint torques.
         * @param wrench A 6x1 vector of forces (3x1) and moments (3x1).
         * @return The joint torque (nx1).
         */
        Eigen::VectorXd
        map_wrench_to_torque(const Eigen::Vector<double,6> &wrench);
        
        /**
         * @brief Set the desired configuration for the redundancy.
         */
        bool
        set_desired_configuration(const Eigen::VectorXd &configuration);
		                       												   		
	protected:
 
        Eigen::VectorXd _desiredConfiguration;                                                      ///< For redundancy
        
		/**
		 * @brief Compute the instantaneous limits on the joint acceleration control.
		 *        It computes the minimum between joint positions, speed, and acceleration limits.
		 * @param jointNumber The joint for which to compute the limits.
		 * @return The upper and lower joint speed at the current time.
		 */
		RobotLibrary::Model::Limits
		compute_control_limits(const unsigned int &jointNumber);
	
};                                                                                                  // Semicolon needed after a class declaration

} } // namespace

#endif
