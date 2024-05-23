/**
 * @file   SerialKinematicControl.h
 * @author Jon Woolfrey
 * @data   December 2023
 * @brief  A class for position/velocity control of a robot arm.
 */

#ifndef SERIALKINEMATICCONTROL_H_
#define SERIALKINEMATICCONTROL_H_

#include <Control/SerialLinkBase.h>

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

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //              Solve the endpoint motion required to achieve a given endpoint motion             //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
Eigen::Vector<DataType,Eigen::Dynamic>
SerialKinematicControl<DataType>::resolve_endpoint_motion(const Eigen::Vector<DataType,6> &endpointMotion)
{
	using namespace Eigen;                                                                         // Makes reading code below a bit simpler
	
	unsigned int numJoints = this->_model->number_of_joints();                                     // Makes things easier
	
	Vector<DataType,Dynamic> controlVelocity(numJoints); controlVelocity.setZero();                // Value to be returned
	
	Vector<DataType,Dynamic> startPoint = this->_model->joint_velocities();                        // For the QP solver
	
	Vector<DataType,Dynamic> lowerBound(numJoints), upperBound(numJoints);                         // On the joint control
	
	for(int i = 0; i < numJoints; i++)
	{
		// NOTE: Can't use structured binding on Eigen::Vector because its size is not
		//       known at compile time, so we have to assign to variable and transfer:
		const auto &[lower, upper] = compute_control_limits(i);                                   // Get the control limits for the ith joint
		lowerBound(i) = lower;                                                                    // Assign to vectors
		upperBound(i) = upper;
	
	     // Make sure the start point is feasible or the QP solver might fail
		     if(startPoint(i) <= lowerBound(i)) startPoint(i) = lowerBound(i) + 1e-03;            // Just above the lower limit
		else if(startPoint(i) >= upperBound(i)) startPoint(i) = upperBound(i) - 1e-03;            // Just below the upper limit
	}
	
	if(this->_manipulability > this->_minManipulability)                                           // Not singular
	{
	     if(this->_model->number_of_joints() <= 6)                                                 // Non redundant
	     {
	          // Solve problem of the form:
	          // min 0.5*(y - A*x)'*W*(y - A*x)
	          // subject to: x_min <= x <= x_max
	          
	          // See: github.com/Woolfrey/software_simple_qp
	          
               controlVelocity
               = QPSolver<DataType>::constrained_least_squares(endpointMotion,                      // y
                                                               this->_jacobianMatrix,               // A
                                                               this->_cartesianStiffness,           // W
                                                               lowerBound,                          // x_min
                                                               upperBound,                          // x_max
                                                               startPoint);                         // Initial guess for solution x
	     }
	     else                                                                                      // Redundant robot
	     {
	          // Solve a problem of the form:
	          // min (x_d - x)'*W*(x_d - x)
	          // subject to: A*x = y
	          //          x_min <= x <= x_max
	          
	          // See: github.com/Woolfrey/software_simple_qp
	          
	          
	          if(not this->_redundantTaskSet)
	          {
	               this->_redundantTask = (this->_controlFrequency/50)                             // NOTE: NEED TO EXPERIMENT TO DETERMINE SCALAR
	                                      *this->manipulability_gradient();                        // Autonomously reconfigure the robot away from singularities
               }
            
	          controlVelocity
	          = QPSolver<DataType>::constrained_least_squares(this->_redundantTask,                // x_d
	                                                          this->_model->joint_inertia_matrix(),// W
	                                                          endpointMotion,                      // y
	                                                          this->_jacobianMatrix,               // A
	                                                          lowerBound,                          // x_min
	                                                          upperBound,                          // x_max
	                                                          startPoint);                         // Initial guess for x
	     
	          this->_redundantTaskSet = false;                                                     // Reset for next control loop
	     }
	}
	else
	{
		// Solve a problem of the form:
		// min 0.5*x'*H*x + x'*f
		// subject to: B*x <= z
		
		// Pre-compute to speed up a little
		Eigen::Matrix<DataType, Eigen::Dynamic, Eigen::Dynamic> JtKd
		= this->_jacobianMatrix*this->_cartesianDamping;
		
		// H = J'*Kd*J + M
		Eigen::Matrix<DataType, Eigen::Dynamic, Eigen::Dynamic> H
		= JtKd*this->_jacobianMatrix + this->_model->joint_inertia_matrix();
		
		// f = -(J'*Kd*x + M*qdot)
		Eigen::Vector<DataType, Eigen::Dynamic> f
		= -JtKd*endpointMotion - this->_model->joint_inertia_matrix()*this->_model->joint_velocities();
		
		// Set up constraints:
		// B*qdot < z

          // B = [  I ]
          //     [ -I ]
		Eigen::Matrix<DataType,Eigen::Dynamic,Eigen::Dynamic> B(2*numJoints,numJoints);
		B.block(0,0,numJoints,numJoints).setIdentity();
		B.block(numJoints,0,numJoints,numJoints) = -B.block(0,0,numJoints,numJoints);
		
		// z = [  upperBound ]
		//     [ -lowerBound ]
		Eigen::Vector<DataType, Eigen::Dynamic> z(2*numJoints);
		z.head(numJoints) =  upperBound;
		z.tail(numJoints) = -lowerBound;
		
		controlVelocity = QPSolver<DataType>::solve(H,f,B,z,startPoint);
	}
	
	return controlVelocity;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //               Compute the endpoint velocity needed to track a given trajectory                //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
Eigen::Vector<DataType,Eigen::Dynamic>
SerialKinematicControl<DataType>::track_endpoint_trajectory(const Pose<DataType>            &desiredPose,
                                                            const Eigen::Vector<DataType,6> &desiredVelocity,
                                                            const Eigen::Vector<DataType,6> &desiredAcceleration)
{
     (void)desiredAcceleration;                                                                     // Not needed in velocity control
     
     // Compute feedforward + feedback control
	return resolve_endpoint_motion(desiredVelocity
	                             + this->_cartesianStiffness
	                             * this->_endpointPose.error(desiredPose));
}
 
  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                  Compute the joint velocities needed to track a given trajectory              //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
Eigen::Vector<DataType,Eigen::Dynamic>
SerialKinematicControl<DataType>::track_joint_trajectory(const Eigen::Vector<DataType,Eigen::Dynamic> &desiredPosition,
                                                         const Eigen::Vector<DataType,Eigen::Dynamic> &desiredVelocity,
						                           const Eigen::Vector<DataType,Eigen::Dynamic> &desiredAcceleration)
{
	unsigned int numJoints = this->_model->number_of_joints();                                     // Makes things easier
	
	if(desiredPosition.size() != numJoints or desiredVelocity.size() != numJoints)
	{
		throw std::invalid_argument("[ERROR] [SERIAL LINK] track_joint_trajectory(): "
		                            "Incorrect size for input arguments. This robot has "
		                            + std::to_string(numJoints) + " joints, but "
		                            "the position argument had " + std::to_string(desiredPosition.size()) + " elements, and"
		                            "the velocity argument had " + std::to_string(desiredVelocity.size()) + " elements.");
	}
	
	Eigen::Vector<DataType,Eigen::Dynamic> velocityControl(numJoints);                             // Value to be returned
	
	for(int i = 0; i < numJoints; i++)
	{
		velocityControl(i) = desiredVelocity(i)                                                                  // Feedforward control
		                   + this->_jointPositionGain*(desiredPosition(i) - this->_model->joint_positions()[i]); // Feedback control
		
		Limits<DataType> controlLimits = compute_control_limits(i);                               // Get the instantaneous limits on the joint speed
		                   
		     if(velocityControl(i) <= controlLimits.lower) velocityControl(i) = controlLimits.lower + 1e-03; // Just above the limit
		else if(velocityControl(i) >= controlLimits.upper) velocityControl(i) = controlLimits.upper - 1e-03; // Just below the limit
	}
	
	return velocityControl;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                   Compute the instantaneous limits on the joint velocities                    //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
Limits<DataType> SerialKinematicControl<DataType>::compute_control_limits(const unsigned int &jointNumber)
{
	// Flacco, F., De Luca, A., & Khatib, O. (2015).
	// "Control of redundant robots under hard joint constraints: Saturation in the null space."
	// IEEE Transactions on Robotics, 31(3), 637-654.
	
	Limits<DataType> limits;                                                                       // Value to be returned

	DataType delta = this->_model->joint_positions()[jointNumber]
	               - this->_model->link(jointNumber)->joint().position_limits().lower;             // Distance from lower limit
	
	limits.lower = std::max(-delta*this->_controlFrequency,
	               std::max(-this->_model->link(jointNumber)->joint().speed_limit(),
	                        -2*sqrt(this->_maxJointAcceleration*delta)));
	                        
	delta = this->_model->link(jointNumber)->joint().position_limits().upper
	      - this->_model->joint_positions()[jointNumber];                                          // Distance to upper limit
	
	limits.upper = std::min(delta*this->_controlFrequency,
	               std::min(this->_model->link(jointNumber)->joint().speed_limit(),
	                        2*sqrt(this->_maxJointAcceleration*delta)));
	                        
	return limits;
}

#endif
