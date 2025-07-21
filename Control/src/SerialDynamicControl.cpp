/**
 * @file    SerialDynamicControl.cpp
 * @author  Jon Woolfrey
 * @email   jonathan.woolfrey@gmail.com
 * @date    July 2025
 * @version 1.0
 * @brief   Computes joint torques required to perform Cartesian, or joint feedback control for a serial link robot arm.
 * 
 * @details This class contains methods for performing torque control of a serial link robot arm
 *          in both Cartesian and joint space.
 *
 * @copyright (c) 2025 Jon Woolfrey
 *
 * @license   OSCL - Free for non-commercial open-source use only.
 *            Commercial use requires a license.
 * 
 * @see https://github.com/Woolfrey/software_robot_library for more information.
 * @see https://github.com/Woolfrey/software_simple_qp for the optimisation algorithm used in the control.
 */

#include <Control/SerialDynamicControl.h>

namespace RobotLibrary { namespace Control {


  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                            Constructor                                        //
///////////////////////////////////////////////////////////////////////////////////////////////////
SerialDynamicControl::SerialDynamicControl(std::shared_ptr<RobotLibrary::Model::KinematicTree> model,
                                           const std::string &endpointName,
                                           const RobotLibrary::Control::SerialLinkParameters &parameters)
: SerialLinkBase(model, endpointName, parameters)
{
    // Worker bees can leave.
    // Even drones can fly away.
    // The Queen is their slave.
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //               Compute the endpoint acceleration needed to track a given trajectory            //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd
SerialDynamicControl::track_endpoint_trajectory(const RobotLibrary::Model::Pose &desiredPose,
                                                const Eigen::Vector<double,6>   &desiredVelocity,
                                                const Eigen::Vector<double,6>   &desiredAcceleration)
{
    // NOTE: This method saves the magnitude of position and orientation error internally,
    //       so it can be queried after for analysing performance
    Eigen::Vector<double,6> poseError = pose_error(desiredPose);
    
    return resolve_endpoint_motion(desiredAcceleration
                                 + _cartesianVelocityGain * (desiredVelocity - endpoint_velocity())
                                 + _cartesianPoseGain * poseError);
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                                        Velocity feedback                                      //
///////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd
SerialDynamicControl::resolve_endpoint_twist(const Eigen::Vector<double,6> &twist)
{
    return resolve_endpoint_motion(_cartesianVelocityGain * (twist - endpoint_velocity()));
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

    using namespace Eigen;                                                                          // Improves readability
    
    // Variables used in this scope
    unsigned int numJoints = _model->number_of_joints();                                            // Makes referencing easier   
    VectorXd manipulabilityGradient = manipulability_gradient();                                    // Used in a few places, so compute it once here
    VectorXd jointVelocities        = _model->joint_velocities();
    VectorXd coriolisTorques        = _model->joint_coriolis_matrix() * jointVelocities;    
    MatrixXd inertiaMatrix          = _model->joint_inertia_matrix();        
    VectorXd lowerBound(numJoints);
    VectorXd upperBound(numJoints);

    VectorXd startPoint = (QPSolver<double>::results().solution.size() == 0)
                        ? VectorXd::Zero(numJoints)
                        : QPSolver<double>::results().solution;                                     // This is needed for the QP solver

    // Compute joint acceleration limits
    for (unsigned int i = 0; i < numJoints; ++i)
    {
        // NOTE: Size of bounds is not known at compile time,
        // so we must manually transfer values
        const auto &[lower, upper] = compute_control_limits(i);
        lowerBound[i] = lower;
        upperBound[i] = upper;

        startPoint[i] = std::clamp(startPoint[i], lower + 1e-03, upper - 1e-03);                    // Ensure within bounds of QP solver might fail
    }
    
    _constraintVector.head(numJoints) = upperBound;
    _constraintVector.segment(numJoints, numJoints) = -lowerBound;

    // Singularity avoidance, assume second derivative of manipulability is zero
    _constraintMatrix.row(2 * numJoints) = -manipulabilityGradient.transpose();
    
    double alpha = 10 * sqrt(_controlFrequency);
    
    _constraintVector(2 * numJoints) = 2 * alpha * manipulabilityGradient.dot(jointVelocities)
                                     + alpha * alpha * (_manipulability - _minManipulability);

    // Now we solve the control
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
                _redundantTask = manipulabilityGradient * sqrt(_controlFrequency) / 2.0             // This term moves away from a singularity
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

    return inertiaMatrix * controlAcceleration;
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
    
	return _model->joint_inertia_matrix() * controlAcceleration;
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
	
    // NOTE: If I include _maxJointAcceleration as a limit, then eventually the lower limit will
	//       be greater than the upper limit, which throws an error ¯\_(⊙︿⊙)_/¯          

	RobotLibrary::Model::Limits limits;
	
	limits.lower = std::max(2 * _controlFrequency * _controlFrequency * ( _model->link(jointNumber)->joint().position_limits().lower - _model->joint_positions()[jointNumber] - _model->joint_velocities()[jointNumber] / _controlFrequency ),
	                        -_controlFrequency * (_model->link(jointNumber)->joint().speed_limit() + _model->joint_velocities()[jointNumber]));
	                           
	limits.upper = std::min(2 * _controlFrequency * _controlFrequency * (_model->link(jointNumber)->joint().position_limits().upper - _model->joint_positions()[jointNumber] - _model->joint_velocities()[jointNumber] / _controlFrequency),
	                        _controlFrequency * (_model->link(jointNumber)->joint().speed_limit() - _model->joint_velocities()[jointNumber]));
	                   
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
