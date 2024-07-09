/**
 * @file   SerialKinematicControl.h
 * @author Jon Woolfrey
 * @data   December 2023
 * @brief  A class for position/velocity control of a robot arm.
 */

#ifndef SERIALKINEMATICCONTROL_H_
#define SERIALKINEMATICCONTROL_H_

#include <SerialLinkBase.h>

template <class DataType>
class SerialKinematicControl : public SerialLinkBase<DataType>
{
	public:
		/**
		 * Constructor.
		 * @param model A pointer to a KinematicTree object.
		 * @param endpointName The name of the reference frame in the KinematicTree to be controlled.
		 */
		SerialKinematicControl(KinematicTree<DataType> *model,
		                       const std::string       &endpointName)
		:
		SerialLinkBase<DataType>(model, endpointName){}
		
		/**
		 * Solve the joint velocities required to move the endpoint at a given speed.
		 * @param endpointMotion A twist vector (linear & angular velocity).
		 * @return A nx1 vector of joint velocities.
		 */
		Eigen::Vector<DataType,Eigen::Dynamic>
		resolve_endpoint_motion(const Eigen::Vector<DataType,6> &endPointMotion);
		
		/**
		 * Solve the joint velocities required to track a Cartesian trajectory.
		 * @param desiredPose The desired position & orientation (pose) for the endpoint.
		 * @param desiredVel The desired linear & angular velocity (twist) for the endpoint.
		 * @param desiredAcc Not used in velocity control.
		 * @return The joint velocities (nx1) required to track the trajectory.
		 */
		Eigen::Vector<DataType,Eigen::Dynamic>
		track_endpoint_trajectory(const Pose<DataType>            &desiredPose,
                                  const Eigen::Vector<DataType,6> &desiredVelocity,
                                  const Eigen::Vector<DataType,6> &desiredAcceleration);

		/**
		 * Solve the joint velocities required to track a joint space trajectory.
		 * @param desiredPos The desired joint position (nx1).
		 * @param desiredVel The desired joint velocity (nx1).
		 * @param desiredAcc Not used in velocity control.
		 * @return The control velocity (nx1).
           */			  
		Eigen::Vector<DataType,Eigen::Dynamic>
		track_joint_trajectory(const Eigen::Vector<DataType,Eigen::Dynamic> &desiredPosition,
		                       const Eigen::Vector<DataType,Eigen::Dynamic> &desiredVelocity,
		                       const Eigen::Vector<DataType,Eigen::Dynamic> &desiredAcceleration);
													   		
	protected:
	
		/**
		 * Compute the instantaneous limits on the joint velocity control.
		 * It computes the minimum between joint positions, speed, and acceleration limits.
		 * @param jointNumber The joint for which to compute the limits.
		 * @return The upper and lower joint speed at the current time.
		 */
		Limits<DataType> compute_control_limits(const unsigned int &jointNumber);
	
};                                                                                                  // Semicolon needed after a class declaration

using SerialKinematicControl_f = SerialKinematicControl<float>;
using SerialKinematicControl_d = SerialKinematicControl<double>;

#endif
