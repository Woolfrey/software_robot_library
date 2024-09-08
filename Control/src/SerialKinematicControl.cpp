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
     
     // Compute feedforward + feedback control
	return resolve_endpoint_motion(desiredVelocity
	                             + this->_cartesianStiffness
	                             * this->_endpointPose.error(desiredPose));
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //              Solve the endpoint motion required to achieve a given endpoint motion             //
////////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXd
SerialKinematicControl::resolve_endpoint_motion(const Eigen::Vector<double,6> &endpointMotion)
{
    using namespace Eigen;                                                                          // Makes reading code below a bit simpler

    unsigned int numJoints = this->_model->number_of_joints();                                      // Makes things easier

    VectorXd controlVelocity(numJoints); controlVelocity.setZero();                                 // Value to be returned

    VectorXd startPoint = this->_model->joint_velocities();                                         // For the QP solver

    VectorXd lowerBound(numJoints), upperBound(numJoints);                                          // On the joint control

    VectorXd manipulabilityGradient = this->manipulability_gradient();                              // Used in multiple places, so compute it once here

    // Update constraints on the joint motion
    for(int i = 0; i < numJoints; i++)
    {
        // NOTE: Can't use structured binding on Eigen::Vector because its size is not
        //       known at compile time, so we have to assign to variable and transfer:
        const auto &[lower, upper] = compute_control_limits(i);                                     // Get the control limits for the ith joint
        lowerBound(i) = lower;                                                                      // Assign to vectors
        upperBound(i) = upper;

        // Make sure the start point is feasible or the QP solver might fail
             if(startPoint(i) <= lowerBound(i)) startPoint(i) = lowerBound(i) + 1e-03;
        else if(startPoint(i) >= upperBound(i)) startPoint(i) = upperBound(i) - 1e-03;
    }

    this->_constraintMatrix.row(2*numJoints) = -manipulabilityGradient.transpose();                 // For singularity avoidance

    this->_constraintVector.block(       0 , 0, numJoints, 1) =  upperBound;
    this->_constraintVector.block(numJoints, 0, numJoints, 1) = -lowerBound;
    //  this->_constraintVector(2*numJoints)                                                        // Needs to be set on a case-by-case basis

    if(this->_manipulability > this->_minManipulability)                                            // Not singular
    {
        this->_constraintVector(2*numJoints) = 
        (this->_controlFrequency)/10 * (this->_manipulability - this->_minManipulability);          // Limit motion toward singularity

        if(this->_model->number_of_joints() <= 6)                                                   // Fully actuated or underactuated robots
        {
            // Solve a problem of the form:
            // min 0.5*x'*H*x + x'*f
            // subject to: B*x <= z

            // See: github.com/Woolfrey/software_simple_qp

            // Weight the solution by the Cartesian inertia matrix:
            Matrix<double,6,6> A = this->_jacobianMatrix
                                 * this->_model->joint_inertia_matrix().ldlt().solve(this->_jacobianMatrix.transpose());

            MatrixXd H = this->_jacobianMatrix.transpose()*A*this->_jacobianMatrix;

            VectorXd f = -this->_jacobianMatrix.transpose()*A*endpointMotion;

            controlVelocity = QPSolver<double>::solve(H, f,
                                                      this->_constraintMatrix,                      // B
                                                      this->_constraintVector,                      // z
                                                      startPoint);                             
        }
        else // REDUNDANT CASE
        {
            // Solve a problem of the form:
            // min (x_d - x)'*W*(x_d - x)
            // subject to: A*x = y
            //             B*x < z

            // See: github.com/Woolfrey/software_simple_qp

            if(not this->_redundantTaskSet)
            {
                this->_redundantTask = (this->_controlFrequency/10)*manipulabilityGradient;         // Autonomously reconfigure the robot away from singularities
            }

            controlVelocity =
            QPSolver<double>::constrained_least_squares(this->_redundantTask,                       // x_d
                                                        this->_model->joint_inertia_matrix(),       // W
                                                        this->_jacobianMatrix,                      // A
                                                        endpointMotion,                             // y
                                                        this->_constraintMatrix,                    // B
                                                        this->_constraintVector,                    // z
                                                        startPoint);                                // Initial guess for x

            this->_redundantTaskSet = false;                                                        // Reset for next control loop
        }
    }
    else // SINGULAR
    {
        // Solve a problem of the form:
        // min 0.5*x'*H*x + x'*f
        // subject to: B*x <= z

        controlVelocity =
        QPSolver<double>::solve(Eigen::MatrixXd::Zero(numJoints,numJoints),
                                Eigen::VectorXd::Zero(numJoints),
                                this->_constraintMatrix,
                                this->_constraintVector,
                                startPoint); 
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
	unsigned int numJoints = this->_model->number_of_joints();                                      // Makes things easier
	
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
		                   + this->_jointPositionGain*(desiredPosition(i) - this->_model->joint_positions()[i]); // Feedback control
		
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

	double delta = this->_model->joint_positions()[jointNumber]
	             - this->_model->link(jointNumber)->joint().position_limits().lower;                // Distance from lower limit
	
	limits.lower = std::max(-delta*this->_controlFrequency,
	               std::max(-this->_model->link(jointNumber)->joint().speed_limit(),
	                        -2*sqrt(this->_maxJointAcceleration*delta)));
	                        
	delta = this->_model->link(jointNumber)->joint().position_limits().upper
	      - this->_model->joint_positions()[jointNumber];                                           // Distance to upper limit
	
	limits.upper = std::min(delta*this->_controlFrequency,
	               std::min(this->_model->link(jointNumber)->joint().speed_limit(),
	                        2*sqrt(this->_maxJointAcceleration*delta)));
	                        
	return limits;
}
