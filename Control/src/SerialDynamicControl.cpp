#include <SerialDynamicControl.h>

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                          Constructor                                           //
////////////////////////////////////////////////////////////////////////////////////////////////////
SerialDynamicControl::SerialDynamicControl(KinematicTree *model,
                                           const std::string &endpointName)
                                           : SerialLinkBase(model, endpointName)
{
    // NOTE: We should make these scale with the control frequency.
    
    _jointPositionGain = 225.0;
    _jointVelocityGain = 30.0;
    
    
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //               Solve the joint torques required to achieve a given endpoint motion              //
////////////////////////////////////////////////////////////////////////////////////////////////////
inline
Eigen::VectorXd
SerialDynamicControl::resolve_endpoint_motion(const Eigen::Vector<double,6> &endpointMotion)
{
    // NOTE: In future we need to include a control barrier function for singularity avoidance.
    
    using namespace Eigen;                                                                          // Makes reading code below a bit simpler

    unsigned int numJoints = _model->number_of_joints();                                            // Makes things easier

    Vector<double,6> cartesianAcceleration = endpointMotion
                                           - _model->time_derivative(_jacobianMatrix)*_model->joint_velocities();
                                           
    // Compute instantaneous limits on joint acceleration
    VectorXd lowerBound(numJoints), upperBound(numJoints);
    
    for(int i = 0; i < numJoints; i++)
    {
        const auto &[lower, upper] = compute_control_limits(i);
        
        lowerBound[i] = lower;
        upperBound[i] = upper;
    }
                                           
    // Determine the start point for the QP solver
    VectorXd startPoint; 
    if(QPSolver<double>::last_solution().size() == numJoints)
    {
        startPoint = QPSolver<double>::last_solution();
        
        for(int i = 0; i < numJoints; i++)
        {
                 if(startPoint[i] <= lowerBound[i]) startPoint[i] = lowerBound[i] + 1e-03;
            else if(startPoint[i] >= upperBound[i]) startPoint[i] = upperBound[i] - 1e-03;
        }
    }
    else startPoint = (lowerBound + upperBound) / 2.0;                                              // Start in the middle
    
    VectorXd jointAcceleration(numJoints); jointAcceleration.setZero();                             // We want to solve for this
    
    if(_manipulability > _minManipulability)                                                        // Not singular
    {
        if(_model->number_of_joints() <= 6)                                                         // Fully actuated or underactuated robots
        {
            // Solve a problem of the form:
            // min_x 1/2*(y - A*x)^T*W*(y - A*x)
            // subject to: x_min <= x <= x_max

            // See: github.com/Woolfrey/software_simple_qp

            jointAcceleration = QPSolver<double>::constrained_least_squares
            (
                cartesianAcceleration,                                                              // y
                _jacobianMatrix,                                                                    // A
                MatrixXd::Identity(6,6),                                                            // W
                lowerBound,                                                                         // x_min
                upperBound,                                                                         // x_max
                startPoint
            );                             
        }
        else // REDUNDANT CASE
        {
            // Solve a problem of the form:
            // min (x_d - x)'*W*(x_d - x)
            // subject to: A*x = y
            //        x_min <= x <= xMax

            // See: github.com/Woolfrey/software_simple_qp

            if(not _redundantTaskSet)
            {
                _redundantTask = this->manipulability_gradient() * _controlFrequency / 100.0         // Autonomously reconfigure away from singularities
                               - _model->joint_damping_vector();                                    // Necessary for stability
            }

            jointAcceleration = QPSolver<double>::constrained_least_squares
            (
                _redundantTask,                                                                     // x_d
                _model->joint_inertia_matrix(),                                                     // W
                _jacobianMatrix,                                                                    // A
                cartesianAcceleration,                                                              // y
                lowerBound,                                                                         // x_min
                upperBound,                                                                         // x_max
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
        
        jointAcceleration = QPSolver<double>::solve
        (
            H,
            -_jacobianMatrix.transpose()*cartesianAcceleration,
            _constraintMatrix.block(0,0,2*numJoints,numJoints),                                     // Ignore last row
            _constraintVector.head(2*numJoints),                                                    // Ignore last element
            startPoint
        );   
    }
    
    return _model->joint_inertia_matrix()*jointAcceleration;
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
     return resolve_endpoint_motion(desiredAcceleration
                                  + _cartesianStiffness*_endpointPose.error(desiredPose)
                                  + _cartesianDamping*(desiredVelocity - _jacobianMatrix*_model->joint_velocities()));
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
    using namespace Eigen;
    
    unsigned int numJoints = _model->number_of_joints();                                            // Makes things easier

    if(desiredPosition.size() != numJoints or desiredVelocity.size() != numJoints)
    {
        throw std::invalid_argument("[ERROR] [SERIAL LINK] track_joint_trajectory(): "
                                    "Incorrect size for input arguments. This robot has "
                                    + std::to_string(numJoints) + " joints, but "
                                    "the position argument had " + std::to_string(desiredPosition.size()) + " elements, and"
                                    "the velocity argument had " + std::to_string(desiredVelocity.size()) + " elements.");
    }

    VectorXd jointAcceleration = desiredAcceleration                                                 // Feedforward
                               + _jointVelocityGain * (desiredVelocity - _model->joint_velocities()) // Velocity feedback
                               + _jointPositionGain * (desiredPosition - _model->joint_positions()); // Position feedback

    // Compute instantaneous limits on joint acceleration
    VectorXd lowerBound(numJoints), upperBound(numJoints);
    
    for(int i = 0; i < numJoints; i++)
    {
        const auto &[lower, upper] = compute_control_limits(i);
        
        lowerBound[i] = lower;
        upperBound[i] = upper;
    }
                                           
    // Determine the start point for the QP solver
    VectorXd startPoint; 
    if(QPSolver<double>::last_solution().size() == numJoints)
    {
        startPoint = QPSolver<double>::last_solution();
        
        for(int i = 0; i < numJoints; i++)
        {
                 if(startPoint[i] <= lowerBound[i]) startPoint[i] = lowerBound[i] + 1e-03;
            else if(startPoint[i] >= upperBound[i]) startPoint[i] = upperBound[i] - 1e-03;
        }
    }
    else startPoint = (lowerBound + upperBound) / 2.0;                                              // Start in the middle
    
    // Due to inertial coupling, if halt 1 joint at the limit, this will cause
    // spikes in joint torque on others. Therefore, solve a QP problem weighted by inertia to
    // minimise tracking error.
    
    // Solve a problem of the form:
    // min_x 1/2(y - A*x)^T*W*(y - A*x)
    // subject to: x_min <= x <= x_max
    
    // see: github.com/Woolfrey/simple_qp_solver
    
    jointAcceleration = QPSolver<double>::constrained_least_squares
    (
        jointAcceleration,                                                                          // y
        MatrixXd::Identity(numJoints,numJoints),                                                    // A
        _model->joint_inertia_matrix(),                                                             // W
        lowerBound,                                                                                 // x_min
        upperBound,                                                                                 // x_max
        startPoint                                                                                  // For the interior point algorithm
    );
    
    return _model->joint_inertia_matrix() * jointAcceleration;
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
    // Calculating the limits given the maximum acceleration is achieved by using the QP solver, outside this function

    Limits limits;                                                                                  // Value to be returned

    double p    = _model->joint_positions()[jointNumber];
    double v    = _model->joint_velocities()[jointNumber];
    double vMax = _model->link(jointNumber)->joint().speed_limit();
    
    double delta = p + v / _controlFrequency
                 - _model->link(jointNumber)->joint().position_limits().lower;                      // Distance from lower limit
                 
    limits.lower = std::max(-2*delta*_controlFrequency*_controlFrequency,
                   std::max(-(vMax + v) * _controlFrequency,
                            -_maxJointAcceleration));
                            
    delta = _model->link(jointNumber)->joint().position_limits().upper
          - (p + v / _controlFrequency);                                                            // Distance to upper limit

    limits.upper = std::min(2*delta*_controlFrequency*_controlFrequency,
                   std::min((vMax - v) / _controlFrequency,
                            _maxJointAcceleration));
                           
    return limits;
}
