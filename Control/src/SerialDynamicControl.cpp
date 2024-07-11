#include <SerialDynamicControl.h>

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //               Solve the joint torques required to achieve a given endpoint motion              //
////////////////////////////////////////////////////////////////////////////////////////////////////
inline
Eigen::VectorXd
SerialDynamicControl::resolve_endpoint_motion(const Eigen::Vector<double,6> &endpointMotion)
{
    using namespace Eigen;                                                                          // Makes reading code below a bit simpler

    unsigned int numJoints = this->_model->number_of_joints();                                      // Makes things easier

    Eigen::VectorXd controlTorque(numJoints); controlTorque.setZero();                              // Value to be returned

    Eigen::VectorXd controlAcceleration(numJoints); controlAcceleration.setZero();                  // Value to be calculated

    Eigen::VectorXd startPoint = QPSolver<double>::last_solution();                                 // For the QP solver

    Eigen::VectorXd lowerBound(numJoints), upperBound(numJoints);                                   // On the joint control

    for(int i = 0; i < numJoints; i++)
    {
        // NOTE: Can't use structured binding on Eigen::Vector because its size is not
        //       known at compile time, so we have to assign to variable and transfer:
        const auto &[lower, upper] = compute_control_limits(i);                                     // Get the control limits for the ith joint
        lowerBound(i) = lower;                                                                      // Assign to vectors
        upperBound(i) = upper;

        // Make sure the start point is feasible or the QP solver might fail
             if(startPoint(i) <= lowerBound(i)) startPoint(i) = lowerBound(i) + 1e-03;              // Just above the lower limit
        else if(startPoint(i) >= upperBound(i)) startPoint(i) = upperBound(i) - 1e-03;              // Just below the upper limit
    }

    if(this->_manipulability > this->_minManipulability)                                            // Not singular
    {
        if(this->_model->number_of_joints() < 7)                                                    // Non redundant
        { 
            // Solve problem of the form:
            // min 0.5*(y - A*x)'*W*(y - A*x)
            // subject to: x_min <= x <= x_max

            // See: github.com/Woolfrey/software_simple_qp

            controlAcceleration
            = QPSolver<double>::constrained_least_squares(endpointMotion - this->_model->time_derivative(this->_jacobianMatrix)*this->_model->joint_velocities(), // y
                                                          this->_jacobianMatrix,                    // A
                                                          this->_cartesianStiffness,                // W
                                                          lowerBound,                               // x_min
                                                          upperBound,                               // x_max
                                                          startPoint);                              // Initial guess for solution x
        }
        else                                                                                        // Redundant robot
        {
            // Solve a problem of the form:
            // min 0.5*x'*H*x + x'*f
            // subject to: B*x <= z

            // See: github.com/Woolfrey/software_simple_qp

            if(not this->_redundantTaskSet)
            {
                this->_redundantTask = (this->_controlFrequency/50)                                 // NOTE: NEED TO EXPERIMENT TO DETERMINE SCALAR
                                     * this->manipulability_gradient()                              // This autonomously reconfigures the robot away from singularities
                                     - this->_model->joint_damping_vector();                        // This ensures energy is dissipated
            }

            controlAcceleration
            = QPSolver<double>::solve(this->_model->joint_inertia_matrix(),                         // H
                                        this->_redundantTask,                                       // f
                                        this->_jacobianMatrix,                                      // B
                                        endpointMotion - this->_model->time_derivative(this->_jacobianMatrix)*this->_model->joint_velocities(), // z
                                        startPoint);                                                // Initial guess for solution x

            this->_redundantTaskSet = false;                                                        // Reset for next control loop
        }
    }
    else                                                                                            // Apply damped least squares
    {

    }

    controlTorque = this->_model->joint_inertia_matrix()*controlAcceleration
                  + this->_model->joint_coriolis_matrix()*this->_model->joint_velocities()
                  + this->_model->joint_damping_vector();

    return controlTorque;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                  Compute the joint torques needed to track a given trajectory                 //
///////////////////////////////////////////////////////////////////////////////////////////////////
inline
Eigen::VectorXd
SerialDynamicControl::track_endpoint_trajectory(const Pose                    &desiredPose,
                                                const Eigen::Vector<double,6> &desiredVelocity,
                                                const Eigen::Vector<double,6> &desiredAcceleration)
{
     // NOTE: I just put Pose _endpointPose as a member in the base class
     
     return resolve_endpoint_motion(desiredAcceleration
                                  + this->_cartesianStiffness*this->_endpointPose.error(desiredPose)
                                  + this->_cartesianDamping*(desiredVelocity - this->_jacobianMatrix*this->_model->joint_velocities()));
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                    Compute the joint torques needed to track a given trajectory               //
///////////////////////////////////////////////////////////////////////////////////////////////////
inline
Eigen::VectorXd
SerialDynamicControl::track_joint_trajectory(const Eigen::VectorXd &desiredPosition,
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

    Eigen::VectorXd torqueControl(numJoints);                                                       // Value to be returned

    for(int i = 0; i < numJoints; i++)
    {   /*
        torqueControl(i) = this->_model->joint_inertia_matrix().row(i)*(desiredAcceleration(i)                                                                  // Feedforward control
                         + this->_jointPositionGain*(desiredPosition(i) - this->_model->joint_positions()[i])
                         + this->_jointDerivativeGain*(desiredVelocity(i) - this->_model->joint_velocities()[i]))
                         + this->_model->joint_coriolis_matrix().row(i)*this->_model->joint_velocities()
                         + this->_model->joint_damping_vector()(; // Feedback control
         */

        Limits controlLimits = compute_control_limits(numJoints);                                   // Get the instantaneous limits on the joint speed

             if(torqueControl(i) <= controlLimits.lower) torqueControl(i) = controlLimits.lower + 1e-03; // Just above the limit
        else if(torqueControl(i) >= controlLimits.upper) torqueControl(i) = controlLimits.upper - 1e-03; // Just below the limit
    }

    return torqueControl;
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                   Compute the instantaneous limits on the joint acceleration                  //
///////////////////////////////////////////////////////////////////////////////////////////////////
inline
Limits
SerialDynamicControl::compute_control_limits(const unsigned int &jointNumber)
{
    // Flacco, F., De Luca, A., & Khatib, O. (2012).
    // "Motion control of redundant robots under joint constraints: Saturation in the null space."
    // IEEE International Conference on Robotics and Automation, 285-292.

    Limits limits;                                                                                  // Value to be returned

    double delta = this->_model->joint_positions()[jointNumber]
                 + this->_model->joint_velocities()[jointNumber]*(1.0/this->_controlFrequency)
                 - this->_model->link(jointNumber)->joint().position_limits().lower;                // Distance from lower limit

    limits.lower = std::max( -2*delta*(1.0/pow(this->_controlFrequency,2)),
                  (std::max((-this->_model->link(jointNumber)->joint().speed_limit() + this->_model->joint_velocities()[jointNumber])*(1.0/this->_controlFrequency),
                             -this->_maxJointAcceleration)));

    delta = this->_model->link(jointNumber)->joint().position_limits().upper
          - this->_model->joint_velocities()[jointNumber]*(1/this->_controlFrequency)
          - this->_model->joint_positions()[jointNumber];                                           // Distance to upper limit

    limits.upper = std::min( 2*delta*(1.0/pow(this->_controlFrequency,2)),
                  (std::min((this->_model->link(jointNumber)->joint().speed_limit() - this->_model->joint_velocities()[jointNumber])*(1.0/this->_controlFrequency),
                             this->_maxJointAcceleration)));

    return limits;
}
