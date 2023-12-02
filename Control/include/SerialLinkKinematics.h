/**
 * @file   SerialLinkKinematics.h
 * @author Jon Woolfrey
 * @data   December 2023
 * @brief  A class for velocity control of a robot arm.
 */

#ifndef SERIALLINKKINEMATICS_H_
#define SERIALLINKKINEMATICS_H_

#include <SerialLinkBase.h>

template <class DataType>
class SerialLinkKinematics : public SerialLinkBase<DataType>
{
	public:
		/**
		 * Constructor.
		 * @param model A pointer to a KinematicTree object.
		 * @param endpointName The name of the reference frame in the KinematicTree to be controlled.
		 */
		SerialLinkKinematics(KinematicTree<DataType> *model,
		                     const std::string &endpointName)
		:
		SerialLinkBase<DataType>(model, endpointName) {}
		
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
					  const Eigen::Vector<DataType,6> &desiredVel,
					  const Eigen::Vector<DataType,6> &desiredAcc);

		/**
		 * Solve the joint velocities required to track a joint space trajectory.
		 * @param desiredPos The desired joint position (nx1).
		 * @param desiredVel The desired joint velocity (nx1).
		 * @param desiredAcc Not used in velocity control.
		 * @return The control velocity (nx1).
                 */			  
		Eigen::Vector<DataType,Eigen::Dynamic>
		track_joint_trajectory(const Eigen::Vector<DataType,Eigen::Dynamic> &desiredPos,
		                       const Eigen::Vector<DataType,Eigen::Dynamic> &desiredVel,
		                       const Eigen::Vector<DataType,Eigen::Dynamic> &desiredAcc);
													   		
	protected:
	
		/**
		 * Compute the instantaneous limits on the joint velocity control.
		 * It computes the minimum between joint positions, speed, and acceleration limits.
		 * @param jointNumber The joint for which to compute the limits.
		 * @return The upper and lower joint speed at the current time.
		 */
		Limits<DataType> compute_control_limits(const unsigned int &jointNumber);
	
};                                                                                                  // Semicolon needed after a class declaration


  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //              Solve the endpoint motion required to achieve a given endpoint motion             //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
Eigen::Vector<DataType,Eigen::Dynamic>
SerialLinkKinematics<DataType>::resolve_endpoint_motion(const Eigen::Vector<DataType,6> &endpointMotion)
{
	using namespace Eigen;
	
	unsigned int numJoints = this->_model->number_of_joints();                                  // Makes things easier
	
	Vector<DataType,Dynamic> controlVelocity(numJoints); controlVelocity.setZero();             // Value to be returned
	
	Vector<DataType,Dynamic> startPoint = this->_model->joint_velocities();                     // For the QP solver
	
	Vector<DataType,Dynamic> lowerBound(numJoints), upperBound(numJoints);                      // On the joint control
	
	for(int i = 0; i < numJoints; i++)
	{
		// NOTE: Can't use structured binding on Eigen::Vector because its size is not
		//       known at compile time, so we have to assign to variable and transfer:
		const auto &[lower, upper] = compute_control_limits(i);                             // Get the control limits for the ith joint
	
		lowerBound(i) = lower;                                                              // Assign to vectors
		upperBound(i) = upper;
	
		     if(startPoint(i) <= lowerBound(i)) startPoint(i) = lowerBound(i) + 1e-03;      // Just above the lower limit
		else if(startPoint(i) >= upperBound(i)) startPoint(i) = upperBound(i) - 1e-03;      // Just below the upper limit
	}
	
	if(this->_manipulability < this->_minManipulability)                                        // Near-singular, solve damped-least squares problem
	{
		// Chiaverini, S., Egeland, O., & Kanestrom, R. K. (1991, June).
		// "Achieving user-defined accuracy with damped least-squares inverse kinematics."
		// International Conference on Advanced Robotics' Robots in Unstructured Environments (pp. 672-677). IEEE.
		
		DataType dampingFactor = pow((1 - this->_manipulability/this->_minManipulability),2) * 1e-03;

		try
		{
			controlVelocity =
			QPSolver<DataType>::constrained_least_squares(this->_jacobianMatrix.transpose()*this->_jacobianMatrix + dampingFactor*Matrix<DataType,Dynamic,Dynamic>::Identity(numJoints,numJoints),
				                                     -this->_jacobianMatrix.transpose()*endpointMotion,
				                                      lowerBound,
				                                      upperBound,
				                                      startPoint);
		}
		catch(const std::exception &exception)
		{
			std::cerr << exception.what() << "\n"; // Use std::endl to print immediately???
			
			controlVelocity = 0.9*this->_model->joint_velocities();                     // Slow down
		}
	}
	else
	{	
		// Formulate the constraints B*qdot < z
		
		// B = [    I   ]
		//     [   -I   ]
		//     [ dmu/dq ]
		
		Matrix<DataType,Dynamic,Dynamic> B(2*numJoints+1,numJoints);
		B.block(          0,0,numJoints,numJoints).setIdentity();
		B.block(  numJoints,0,numJoints,numJoints) = -B.block(0,0,numJoints,numJoints);
		B.block(2*numJoints,0,        1,numJoints) = this->manipulability_gradient().transpose(); // Need to use 'this->' or it won't compile
		
		// z = [       upperLimit      ]
		//     [       lowerLimit      ]
		//     [ -scalar*(mu - mu_min) ]
		
		Vector<DataType,Dynamic> z(2*numJoints+1);
		z.block(          0,0,numJoints,1) = upperBound;
		z.block(  numJoints,0,numJoints,1) = lowerBound;
		z.block(2*numJoints,1,        1,1) = this->_controlBarrierScalar*(this->_minManipulability - this->manipulability);
		
		if(numJoints <= 6)
		{
			controlVelocity =
			QPSolver<DataType>::solve(this->_jacobianMatrix.transpose()*this->_jacobianMatrix,
			                         -this->_jacobianMatrix.transpose()*endpointMotion,
			                          B, z, startPoint);
		}
		else                                                                                        // Redundant case
		{
			// NOTE TO FUTURE SELF: If redundant_task() is to maximise manipulability,
			// then we're computing it twice here, which is inefficient...
			
			try
			{
				controlVelocity =
				QPSolver<DataType>::constrained_least_squares(this->redundant_task(), // Need to use `this->` or it won't compile
					                                      this->_model->joint_inertia_matrix(),
					                                      this->_jacobianMatrix,
					                                      endpointMotion,
					                                      B, z, startPoint);
			}
			catch(const std::exception &exception)
			{
				std::cerr << exception.what() << "\n"; // Use std::endl to print immediately??
				
				controlVelocity = 0.9*this->_model->joint_velocities();             // Slow down
			}
		}
	}
	
	return controlVelocity;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //               Compute the endpoint velocity needed to track a given trajectory                //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
Eigen::Vector<DataType,Eigen::Dynamic>
SerialLinkKinematics<DataType>::track_endpoint_trajectory(const Pose<DataType>            &desiredPose,
                                                          const Eigen::Vector<DataType,6> &desiredVel,
                                                          const Eigen::Vector<DataType,6> &desiredAcc)
{
	return resolve_endpoint_motion(desiredVel + this->_cartesianStiffness*this->_endpointPose.error(desiredPose)); // Feedforward + feedback control
}
 
  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                  Compute the joint velocities needs to track a given trajectory               //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
Eigen::Vector<DataType,Eigen::Dynamic>
SerialLinkKinematics<DataType>::track_joint_trajectory(const Eigen::Vector<DataType,Eigen::Dynamic> &desiredPos,
                                                       const Eigen::Vector<DataType,Eigen::Dynamic> &desiredVel,
						       const Eigen::Vector<DataType,Eigen::Dynamic> &desiredAcc)
{
	unsigned int numJoints = this->_model->number_of_joints();                                  // Makes things easier
	
	if(desiredPos.size() != numJoints or desiredVel != numJoints)
	{
		throw std::invalid_argument("[ERROR] [SERIAL LINK] track_joint_trajectory(): "
		                            "Incorrect size for input arguments. This robot has "
		                            + std::to_string(numJoints) + " joints, but "
		                            "the position argument had " + std::to_string(desiredPos.size()) + " elements, and"
		                            "the velocity argument had " + std::to_string(desiredVel.size()) + " elements.");
	}
	
	Eigen::Vector<DataType,Eigen::Dynamic> velocityControl(numJoints);                          // Value to be returned
	
	for(int i = 0; i < numJoints; i++)
	{
		Limits<DataType> controlLimits = compute_control_limits(numJoints);                 // Get the instantaneous limits on the joint speed
		
		velocityControl(i) = desiredVel(i) + this->_jointPositionGain*(desiredPos(i) - this->_model->joint_position(i)); // Feedforward + feedback control
		                   
		     if(velocityControl(i) <= controlLimits.lower) velocityControl(i) = controlLimits.lower + 1e-03; // Just above the limit
		else if(velocityControl(i) >= controlLimits.upper) velocityControl(i) = controlLimits.upper - 1e-03; // Just below the limit
	}
	
	return velocityControl;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                   Compute the instantaneous limits on the joint velocities                    //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
Limits<DataType> SerialLinkKinematics<DataType>::compute_control_limits(const unsigned int &jointNumber)
{
	// Flacco, F., De Luca, A., & Khatib, O. (2015).
	// "Control of redundant robots under hard joint constraints: Saturation in the null space."
	// IEEE Transactions on Robotics, 31(3), 637-654.
	
	Limits<DataType> limits;                                                                    // Value to be returned

	DataType delta = this->_model->joint_position(jointNumber)
	               - this->_model->link(jointNumber).joint().position_limits().lower;           // Distance from the lower joint limit
	
	limits.lower = std::max(-delta*this->_controlFrequency,
	                        -this->_model->link(jointNumber).joint().speed_limit(),
	                        -2*sqrt(this->_maxJointAccel*delta));
	                        
	delta = this->_model->link(jointNumber).joint().position_limits().upper
	      - this->_model->joint_position(jointNumber);                                          // Distance to upper limit
	
	limits.upper = std::min(delta*this->_controlFrequency,
	                        this->_model->link(jointNumber).joint().speed_limit(),
	                        2*sqrt(this->_maxJointAccel*delta));
	                        
	return limits;
}

#endif
