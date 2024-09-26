/**
 * @file   SerialKinematicControl.cpp
 * @author Jon Woolfrey
 * @data   July 2024
 * @brief  Source files for the Kinematic Control class.
 */
 
#include <SerialKinematicControl.h>

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //               Compute the endpoint velocity needed to track a given trajectory                //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd
SerialKinematicControl::track_endpoint_trajectory(const Pose                    &desiredPose,
                                                  const Eigen::Vector<double,6> &desiredVelocity,
                                                  const Eigen::Vector<double,6> &desiredAcceleration)
{
     (void)desiredAcceleration;                                                                     // Not needed in velocity control
     
	return resolve_endpoint_motion(desiredVelocity                                                  // Feedforward term
	                             + _cartesianStiffness * _endpointPose.error(desiredPose));         // Feedback term
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //              Solve the endpoint motion required to achieve a given endpoint motion             //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd
SerialKinematicControl::resolve_endpoint_motion(const Eigen::Vector<double,6> &endpointMotion)
{
    using namespace Eigen;                                                                          // Makes reading code below a bit simpler
    
    unsigned int numJoints = _model->number_of_joints();                                            // Makes things easier
    
    VectorXd startPoint = _model->joint_velocities();                                               // Used in the QP solver
    
    // Get the instantaneous limits on the joint velocities
    
    VectorXd lowerBound(numJoints), upperBound(numJoints);
    
    for(int i = 0; i < numJoints; i++)
    {
        const auto &[lower, upper] = compute_control_limits(i);
        
        lowerBound[i] = lower;
        upperBound[i] = upper;
        
        // Ensure start point is within limits, or QP solver might fail
             if(startPoint[i] <= lower) startPoint[i] = lower + 1e-03;
        else if(startPoint[i] >= upper) startPoint[i] = upper - 1e-03;
    }
    
    VectorXd manipulabilityGradient = this->manipulability_gradient();                              // Used in a couple of places, so compute once here
    
    // Update the constraints for the QP solver
    _constraintMatrix.row(2*numJoints) = -manipulabilityGradient.transpose();
    _constraintVector.block(        0, 0, numJoints, 1) =  upperBound;
    _constraintVector.block(numJoints, 0, numJoints, 1) = -lowerBound;
    _constraintVector(2*numJoints) = (_manipulability - _minManipulability) * _controlFrequency / 10.0; // ADJUST THIS?
    
    VectorXd controlVelocity(numJoints); controlVelocity.setZero();                                 // We want to compute this
    
    if(_manipulability > _minManipulability)                                                        // Not singular
    {
        if(_model->number_of_joints() <= 6)                                                         // Fully actuated or underactuated robots
        {
            // Solve a problem of the form:
            // min 0.5*x'*H*x + x'*f
            // subject to: B*x <= z

            // See: github.com/Woolfrey/software_simple_qp

            controlVelocity = QPSolver<double>::solve(_jacobianMatrix.transpose()*_jacobianMatrix,  // H
                                                     -_jacobianMatrix.transpose()*endpointMotion,   // f
                                                      _constraintMatrix,                            // B
                                                      _constraintVector,                            // z
                                                      startPoint);                             
        }
        else // REDUNDANT CASE
        {
            // Solve a problem of the form:
            // min (x_d - x)'*W*(x_d - x)
            // subject to: A*x = y
            //             B*x < z

            // See: github.com/Woolfrey/software_simple_qp

            if(not _redundantTaskSet)
            {
                _redundantTask = manipulabilityGradient * _controlFrequency / 20.0;                 // Autonomously reconfigure away from singularities
            }

            controlVelocity = QPSolver<double>::constrained_least_squares(
                _redundantTask,                                                                     // x_d
                _model->joint_inertia_matrix(),                                                     // W
                _jacobianMatrix,                                                                    // A
                endpointMotion,                                                                     // y
                _constraintMatrix,                                                                  // B
                _constraintVector,                                                                  // z
                startPoint                                                                          // Initial guess for x
            );                                                                        

            _redundantTaskSet = false;                                                              // Reset for next control loop
        }
    }
    else // SINGULAR
    {
        // Control barrier function must have been violated, so apply damped least squares:
        // Chiaverini, S., Egeland, O., & Kanestrom, R. K. (1991, June).
        // Achieving user-defined accuracy with damped least-squares inverse kinematics.
        // In Fifth International Conference on Advanced Robotics Robots in Unstructured Environments
        // (pp. 672-677). IEEE.
       
        double dampingFactor = pow(1.0 - _manipulability/_minManipulability, 2.0) * 0.10;           // Attenuate damping based on proximity to singularity

        // Solve a problem of the form:
        // min 0.5*x'*H*x + x'*f
        // subject to: B*x <= z
           
        MatrixXd H = _jacobianMatrix.transpose()*_jacobianMatrix;
        
        for(int i = 0; i < numJoints; i++) H(i,i) += dampingFactor;
        
        controlVelocity = QPSolver<double>::solve
        (
            H,
            -_jacobianMatrix.transpose()*endpointMotion,
            _constraintMatrix.block(0,0,2*numJoints,numJoints),                                     // Ignore last row
            _constraintVector.head(2*numJoints),                                                    // Ignore last element
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
	unsigned int numJoints = _model->number_of_joints();                                      // Makes things easier
	
	if(desiredPosition.size() != numJoints or desiredVelocity.size() != numJoints)
	{
		throw std::invalid_argument("[ERROR] [SERIAL LINK] track_joint_trajectory(): "
		                            "Incorrect size for input arguments. This robot has "
		                            + std::to_string(numJoints) + " joints, but "
		                            "the position argument had " + std::to_string(desiredPosition.size()) + " elements, and"
		                            "the velocity argument had " + std::to_string(desiredVelocity.size()) + " elements.");
	}
	
	Eigen::VectorXd velocityControl(numJoints);                                                     // Value to be returned
	
	for(int i = 0; i < numJoints; i++)
	{
		velocityControl(i) = desiredVelocity(i)                                                                  // Feedforward control
		                   + _jointPositionGain*(desiredPosition(i) - _model->joint_positions()[i]); // Feedback control
		
		Limits controlLimits = compute_control_limits(i);                                           // Get the instantaneous limits on the joint speed
		                   
		     if(velocityControl(i) <= controlLimits.lower) velocityControl(i) = controlLimits.lower + 1e-03; // Just above the limit
		else if(velocityControl(i) >= controlLimits.upper) velocityControl(i) = controlLimits.upper - 1e-03; // Just below the limit
	}
	
	return velocityControl;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                   Compute the instantaneous limits on the joint velocities                    //
///////////////////////////////////////////////////////////////////////////////////////////////////
Limits
SerialKinematicControl::compute_control_limits(const unsigned int &jointNumber)
{
	// Flacco, F., De Luca, A., & Khatib, O. (2015).
	// "Control of redundant robots under hard joint constraints: Saturation in the null space."
	// IEEE Transactions on Robotics, 31(3), 637-654.
	
	Limits limits;                                                                                  // Value to be returned

	double delta = _model->joint_positions()[jointNumber]
	             - _model->link(jointNumber)->joint().position_limits().lower;                // Distance from lower limit
	
	limits.lower = std::max(-delta*_controlFrequency,
	               std::max(-_model->link(jointNumber)->joint().speed_limit(),
	                        -2*sqrt(_maxJointAcceleration*delta)));
	                        
	delta = _model->link(jointNumber)->joint().position_limits().upper
	      - _model->joint_positions()[jointNumber];                                           // Distance to upper limit
	
	limits.upper = std::min(delta*_controlFrequency,
	               std::min(_model->link(jointNumber)->joint().speed_limit(),
	                        2*sqrt(_maxJointAcceleration*delta)));
	                        
	return limits;
}
