/**
 * @file    SerialKinematicControl.cpp
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
 * @see https://github.com/Woolfrey/software_simple_qp for the optimisation algorithm used in the control.
 */
 
#include "Control/SerialKinematicControl.h"

namespace RobotLibrary { namespace Control {

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //               Compute the endpoint velocity needed to track a given trajectory                //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd
SerialKinematicControl::track_endpoint_trajectory(const RobotLibrary::Model::Pose &desiredPose,
                                                  const Eigen::Vector<double,6>   &desiredVelocity,
                                                  const Eigen::Vector<double,6>   &desiredAcceleration)
{
     (void)desiredAcceleration;                                                                     // Not needed in velocity control
     
	return resolve_endpoint_motion(desiredVelocity                                                  // Feedforward term
	                             + _cartesianStiffness * _endpointPose.error(desiredPose));         // Feedback term
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                       This doesn't do much...!                                //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd
SerialKinematicControl::resolve_endpoint_twist(const Eigen::Vector<double,6> &twist)
{
    return resolve_endpoint_motion(twist);
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //              Solve the endpoint motion required to achieve a given endpoint motion             //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd
SerialKinematicControl::resolve_endpoint_motion(const Eigen::Vector<double,6> &endpointMotion)
{
    using namespace Eigen;                                                                          // Improves readability
    
    // Variables used in this scope
    unsigned int numJoints = _model->number_of_joints();                                            // Makes referencing easier       
    VectorXd startPoint = _model->joint_velocities();                                               // Needed for the QP solver
    VectorXd lowerBound(numJoints), upperBound(numJoints);                                          // Limits on joint control

    // Compute joint velocity limits and ensure the starting point is within bounds
    for (unsigned int i = 0; i < numJoints; ++i)
    {
        // NOTE: Size of bounds is not known at compile time,
        // so we must manually transfer values
        const auto &[lower, upper] = compute_control_limits(i);
        lowerBound[i] = lower;
        upperBound[i] = upper;

        startPoint[i] = std::clamp(startPoint[i], lower + 1e-03, upper - 1e-03);                    // Ensure within bounds of QP solver might fail
    }

    // Compute manipulability gradient once and update constraints
    const VectorXd manipulabilityGradient = manipulability_gradient();                              // Used in a few places, so compute it once here
    _constraintMatrix.row(2 * numJoints) = -manipulabilityGradient.transpose();                     // Part of the control barrier function
    _constraintVector.head(numJoints) = upperBound;
    _constraintVector.segment(numJoints, numJoints) = -lowerBound;
    _constraintVector(2 * numJoints) = (_manipulability - _minManipulability) * 100 * sqrt(_controlFrequency);

    VectorXd controlVelocity = VectorXd::Zero(numJoints);                                           // We need to compute this

    if (not is_singular())
    {
        if (numJoints <= 6)                                                                         // Fully actuated or underactuated robots
        {
            // Solve a problem of the form:
            // min 0.5*x'*H*x + x'*f
            // subject to: B*x <= z

            // See: github.com/Woolfrey/software_simple_qp
            
            controlVelocity = QPSolver<double>::solve(
                _jacobianMatrix.transpose() * _jacobianMatrix,                                      // H
               -_jacobianMatrix.transpose() * endpointMotion,                                       // f
                _constraintMatrix,                                                                  // B
                _constraintVector,                                                                  // z
                startPoint                                                                          // Initial guess
            );
        }
        else                                                                                        // Redundant robot
        {
            if (not _redundantTaskSet)
            {
                _redundantTask = manipulabilityGradient * sqrt(_controlFrequency) / 5.0;   
                _redundantTaskSet = false;                                                          // Set false for next control loop
            }

            // Solve a problem of the form:
            // min (x_d - x)'*W*(x_d - x)
            // subject to: A*x = y
            //             B*x < z

            // See: github.com/Woolfrey/software_simple_qp
            
            controlVelocity = QPSolver<double>::constrained_least_squares(
                _redundantTask,                                                                     // x_d
                _model->joint_inertia_matrix(),                                                     // W
                _jacobianMatrix,                                                                    // A
                endpointMotion,                                                                     // y
                _constraintMatrix,                                                                  // B
                _constraintVector,                                                                  // z
                startPoint                                                                          // Initial guess
            );
        }
    }
    else                                                                                            // Singular case
    {
        // Control barrier function must have been violated, so apply damped least squares:
        // Chiaverini, S., Egeland, O., & Kanestrom, R. K. (1991, June).
        // Achieving user-defined accuracy with damped least-squares inverse kinematics.
        // In Fifth International Conference on Advanced Robotics Robots in Unstructured Environments
        // (pp. 672-677). IEEE.
       
        double dampingFactor = pow(1.0 - _manipulability/_minManipulability, 2.0) * 0.01;           // Attenuate damping based on proximity to singularity

        // Solve a problem of the form:
        // min 0.5*x'*H*x + x'*f
        // subject to: B*x <= z
        
        MatrixXd H = _jacobianMatrix.transpose() * _jacobianMatrix; 
        
        H.diagonal().array() += dampingFactor;
        
        controlVelocity = QPSolver<double>::solve(
            H,
           -_jacobianMatrix.transpose() * endpointMotion,
            _constraintMatrix.block(0, 0, 2 * numJoints, numJoints),                                // Use only first 2*numJoints rows
            _constraintVector.head(2 * numJoints),                                                  // Use only first 2*numJoints elements
            startPoint
        );
    }

    return controlVelocity;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                  Compute the joint velocities needed to track a given trajectory              //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd
SerialKinematicControl::track_joint_trajectory(const Eigen::VectorXd &desiredPosition,
                                               const Eigen::VectorXd &desiredVelocity,
						                       const Eigen::VectorXd &desiredAcceleration)
{
	unsigned int numJoints = _model->number_of_joints();                                            // Makes things easier
	
	if(desiredPosition.size() != numJoints or desiredVelocity.size() != numJoints)
	{
		throw std::invalid_argument("[ERROR] [SERIAL LINK] track_joint_trajectory(): "
		                            "Incorrect size for input arguments. This robot has "
		                            + std::to_string(numJoints) + " joints, but "
		                            "the position argument had " + std::to_string(desiredPosition.size()) + " elements, and"
		                            "the velocity argument had " + std::to_string(desiredVelocity.size()) + " elements.");
	}
	
	Eigen::VectorXd velocityControl(numJoints);                                                     // Value to be returned
	
	for(int i = 0; i < numJoints; ++i)
	{
		velocityControl(i) = desiredVelocity(i)                                                      // Feedforward control
		                   + _jointPositionGain*(desiredPosition(i) - _model->joint_positions()[i]); // Feedback control
		
		RobotLibrary::Model::Limits controlLimits = compute_control_limits(i);                      // Get the instantaneous limits on the joint speed
		                   
		     if(velocityControl(i) <= controlLimits.lower) velocityControl(i) = controlLimits.lower + 1e-03; // Just above the limit
		else if(velocityControl(i) >= controlLimits.upper) velocityControl(i) = controlLimits.upper - 1e-03; // Just below the limit
	}
	
	return velocityControl;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                   Compute the instantaneous limits on the joint velocities                    //
///////////////////////////////////////////////////////////////////////////////////////////////////
RobotLibrary::Model::Limits
SerialKinematicControl::compute_control_limits(const unsigned int &jointNumber)
{
	// Flacco, F., De Luca, A., & Khatib, O. (2015).
	// "Control of redundant robots under hard joint constraints: Saturation in the null space."
	// IEEE Transactions on Robotics, 31(3), 637-654.
	
	RobotLibrary::Model::Limits limits;                                                             // Value to be returned

	double delta = _model->joint_positions()[jointNumber]
	             - _model->link(jointNumber)->joint().position_limits().lower;                      // Distance from lower limit
	
	limits.lower = std::max(-delta*_controlFrequency,
	               std::max(-_model->link(jointNumber)->joint().speed_limit(),
	                        -2*sqrt(_maxJointAcceleration*delta)));
	                        
	delta = _model->link(jointNumber)->joint().position_limits().upper
	      - _model->joint_positions()[jointNumber];                                                 // Distance to upper limit
	
	limits.upper = std::min(delta*_controlFrequency,
	               std::min(_model->link(jointNumber)->joint().speed_limit(),
	                        2*sqrt(_maxJointAcceleration*delta)));
	                        
	if(limits.lower > limits.upper)
	{
	    throw std::runtime_error(
	        "[ERROR] [SERIAL KINEMATIC CONTROL] compute_control_limits():"
	        "Lower limit for the '" + _model->link(jointNumber)->joint().name() + "' joint is greater than "
	        "upper limit (" + std::to_string(limits.lower) + " > " + std::to_string(limits.upper) + "). "
	        "How did that happen???");
    }
	                        
	return limits;
}

} }
