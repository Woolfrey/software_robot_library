/**
 * @file    SerialDynamicControl.cpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    May 2025
 * @version 1.0
 * @brief   Computes joint torques required to perform Cartesian, or joint feedback control for a serial link robot arm.
 * 
 * @details This class contains methods for performing torque control of a serial link robot arm
 *          in both Cartesian and joint space.
 * 
 * @copyright Copyright (c) 2025 Jon Woolfrey
 * 
 * @license GNU General Public License V3
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 * @see https://github.com/Woolfrey/software_simple_qp for the optimisation algorithm used in the control.
 */

#include <Control/SerialDynamicControl.h>

namespace RobotLibrary { namespace Control {

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //               Compute the endpoint acceleration needed to track a given trajectory            //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd
SerialDynamicControl::track_endpoint_trajectory(const RobotLibrary::Model::Pose &desiredPose,
                                                const Eigen::Vector<double,6>   &desiredVelocity,
                                                const Eigen::Vector<double,6>   &desiredAcceleration)
{
    return resolve_endpoint_motion(desiredAcceleration                                              // Feedforward acceleration
                                 + _cartesianDamping * (desiredVelocity - endpoint_velocity())      // Velocity feedback
                                 + _cartesianStiffness * _endpointPose.error(desiredPose));         // Pose feedback
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                        Velocity feedback                                      //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd
SerialDynamicControl::resolve_endpoint_twist(const Eigen::Vector<double,6> &twist)
{
    return resolve_endpoint_motion(_cartesianDamping*(twist - endpoint_velocity()));
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //              Solve the endpoint motion required to achieve a given endpoint motion             //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd
SerialDynamicControl::resolve_endpoint_motion(const Eigen::Vector<double,6> &endpointMotion)
{
    // See:
    // Bruyninckx, H., & Khatib, O. (2000, April).
    // Gauss' principle and the dynamics of redundant and constrained manipulators.
    // In Proceedings 2000 ICRA. Millennium Conference. IEEE International Conference on Robotics and Automation.
    // Symposia Proceedings (Cat. No. 00CH37065) (Vol. 3, pp. 2563-2568). IEEE.
    // 
    // Ferraguti, F., Landi, C. T., Singletary, A., Lin, H. C., Ames, A., Secchi, C., & Bonfe, M. (2022).
    // Safety and efficiency in robotics: The control barrier functions approach.
    // IEEE Robotics & Automation Magazine, 29(3), 139-151.

    using namespace Eigen;                                                                          // Improves readability
    
    // Variables used in this scope
    unsigned int numJoints = _model->number_of_joints();                                            // Makes referencing easier   
        
    VectorXd lowerBound(numJoints), upperBound(numJoints);                                          // Limits on joint control

    VectorXd startPoint = (QPSolver<double>::results().solution.size() == 0)
                        ? VectorXd::Zero(numJoints)
                        : QPSolver<double>::results().solution;

    // Compute joint acceleration limits and ensure the starting point is within bounds
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
    const VectorXd jointVelocities        = _model->joint_velocities();
    const VectorXd coriolisTorques        = _model->joint_coriolis_matrix() * jointVelocities;    
    const MatrixXd inertiaMatrix          = _model->joint_inertia_matrix();
    
    _constraintMatrix.row(2 * numJoints) = (inertiaMatrix * jointVelocities).transpose();           // Part of the dynamic control barrier function
    _constraintVector.head(numJoints) = upperBound;
    _constraintVector.segment(numJoints, numJoints) = -lowerBound;
    _constraintVector(2 * numJoints) = (_manipulability - _minManipulability) * 100 * sqrt(_controlFrequency)
                                     + (manipulabilityGradient - coriolisTorques).dot(jointVelocities);

    VectorXd controlAcceleration = VectorXd::Zero(numJoints);
    
    if (not is_singular())
    {
        if (numJoints <= 6)
        {
            // Solve a problem of the form:
            // min 0.5*x'*H*x + x'*f
            // subject to: B*x <= z

            // See: github.com/Woolfrey/software_simple_qp
            
            controlAcceleration = QPSolver<double>::solve(
                _jacobianMatrix.transpose() * _jacobianMatrix,                                      // H
               -_jacobianMatrix.transpose() * (endpointMotion - _model->time_derivative(_jacobianMatrix) * jointVelocities), // f
                _constraintMatrix,                                                                  // B
                _constraintVector,                                                                  // z
                startPoint                                                                          // Initial guess
            );
        }
        else                                                                                        // Redundant robot
        {
            if (not _redundantTaskSet)
            {
                _redundantTask = manipulabilityGradient * sqrt(_controlFrequency) / 10.0            // This term moves away from a singularity
                               - coriolisTorques                                                    // This term minimises kinetic energy
                               - _model->joint_damping_vector();                                    // This term ensures stability
                               
                _redundantTaskSet = false;                                                          // Set false for next control loop
            }

            // Solve a problem of the form:
            // min (x_d - x)'*W*(x_d - x)
            // subject to: A*x = y
            //             B*x < z

            // See: github.com/Woolfrey/software_simple_qp
            
            controlAcceleration = QPSolver<double>::constrained_least_squares(
                _redundantTask,                                                                     // x_d
                inertiaMatrix,                                                                      // W
                _jacobianMatrix,                                                                    // A
                endpointMotion - _model->time_derivative(_jacobianMatrix) * jointVelocities,        // y
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
        
        controlAcceleration = QPSolver<double>::solve(
            H,
           -_jacobianMatrix.transpose() * (endpointMotion - _model->time_derivative(_jacobianMatrix) * _model->joint_velocities()) ,
            _constraintMatrix.block(0, 0, 2 * numJoints, numJoints),                                // Use only first 2*numJoints rows
            _constraintVector.head(2 * numJoints),                                                  // Use only first 2*numJoints elements
            startPoint
        );
    }

    return inertiaMatrix * controlAcceleration + coriolisTorques;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                  Compute the joint torques needed to track a given joint trajectory           //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd
SerialDynamicControl::track_joint_trajectory(const Eigen::VectorXd &desiredPosition,
                                             const Eigen::VectorXd &desiredVelocity,
						                     const Eigen::VectorXd &desiredAcceleration)
{
	unsigned int numJoints = _model->number_of_joints();                                            // Makes things easier
	
	if(desiredPosition.size()     != numJoints
	or desiredVelocity.size()     != numJoints
	or desiredAcceleration.size() != numJoints)
	{
		throw std::invalid_argument("[ERROR] [SERIAL DYNAMIC CONTROL] track_joint_trajectory(): "
		                            "Incorrect size for input arguments. This robot has "
		                            + std::to_string(numJoints) + " joints, but "
		                            "the position argument had " + std::to_string(desiredPosition.size()) + " elements,"
		                            "the velocity argument had " + std::to_string(desiredVelocity.size()) + " elements, and "
		                            "the acceleration argument had " + std::to_string(desiredAcceleration.size()) + " elements.");
	}
	
	Eigen::VectorXd controlAcceleration = Eigen::VectorXd::Zero(numJoints);                         // We want to compute this
	
	for (int i = 0; i < numJoints; ++i)
	{
	    
	    controlAcceleration(i) = desiredAcceleration(i)                                                    // Feedforward term
	                           + _jointVelocityGain * (desiredVelocity(i) - _model->joint_velocities()[i]) // Velocity feedback
	                           + _jointPositionGain * (desiredPosition(i) - _model->joint_positions()[i]); // Position feedback
        
        const auto &[lower, upper] = compute_control_limits(i);                                     // Get instantaneous limits on the joint acceleration
         
        controlAcceleration(i) = std::clamp(controlAcceleration(i), lower + 1e-03, upper - 1e-03);  // Ensure within limits
    }
	
	return _model->joint_inertia_matrix() * controlAcceleration + _model->joint_coriolis_matrix() * _model->joint_velocities();
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                   Compute the instantaneous limits on the joint velocities                    //
///////////////////////////////////////////////////////////////////////////////////////////////////
RobotLibrary::Model::Limits
SerialDynamicControl::compute_control_limits(const unsigned int &jointNumber)
{
	// Flacco, F., De Luca, A., & Khatib, O. (2015).
	// "Control of redundant robots under hard joint constraints: Saturation in the null space."
	// IEEE Transactions on Robotics, 31(3), 637-654.

	RobotLibrary::Model::Limits limits;                                                             // Value to be returned
	
	limits.lower = std::max(2 * _controlFrequency * _controlFrequency * ( _model->link(jointNumber)->joint().position_limits().lower - _model->joint_positions()[jointNumber] - _model->joint_velocities()[jointNumber] / _controlFrequency ),
	               std::max(-_controlFrequency * (_model->link(jointNumber)->joint().speed_limit() + _model->joint_velocities()[jointNumber]),
	                        -_maxJointAcceleration ));
	                           
	limits.upper = std::min(2 * _controlFrequency * _controlFrequency * (_model->link(jointNumber)->joint().position_limits().upper - _model->joint_positions()[jointNumber] - _model->joint_velocities()[jointNumber] / _controlFrequency),
	               std::min(_controlFrequency * (_model->link(jointNumber)->joint().speed_limit() - _model->joint_velocities()[jointNumber]),
	                        _maxJointAcceleration ));	
	                        
	if(limits.lower > limits.upper)
	{
	    throw std::logic_error("[ERROR] [SERIAL DYNAMIC CONTROL] compute_control_limits(): "
	                           "Lower limit for the '" + _model->link(jointNumber)->joint().name() + "' joint is greater than "
	                           "upper limit (" + std::to_string(limits.lower) + " > " + std::to_string(limits.upper) + "). "
	                           "How did that happen???");
    }
	                        
	return limits;
}

} } // namespace
