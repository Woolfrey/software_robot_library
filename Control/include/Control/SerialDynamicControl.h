#ifndef SERIALDYNAMICCONTROL_H
#define SERIALDYNAMICCONTROL_H

#include "Control/SerialLinkBase.h"

template <class DataType>
class SerialDynamicControl : public SerialLinkBase <DataType>
{
    public:
        /**
         * Constructor.
         * @param model A pointer to a KinematicTree object.
         * @param endpointName The name of the reference frame in the KinematicTree to be controlled.
         */
        SerialDynamicControl(KinematicTree<DataType> *model,
                             const std::string &endpointName);

        /**
         * Solve the joint torques required to move the endpoint at a given acceleration.
         * @param endpointMotion An acceleration vector (linear & angular acceleration).
         * @return A nx1 vector of joint torques.
         */
        Eigen::Vector<DataType,Eigen::Dynamic>
        resolve_endpoint_motion(const Eigen::Vector<DataType,6> &endpointMotion);

        /**
         * Solve the joint torques required to track a Cartesian trajectory.
         * @param desiredPose The desired position & orientation (pose) for the endpoint.
         * @param desiredVelocity The desired linear & angular velocity (twist) for the endpoint.
         * @param desiredAcceleration The desired linear & angular acceleration for the endpoint.
         * @return The joint torques (nx1) required to track the trajectory.
         */
        Eigen::Vector<DataType,Eigen::Dynamic>
        track_endpoint_trajectory(const Pose<DataType>            &desiredPose,
                                  const Eigen::Vector<DataType,6> &desiredVelocity,
                                  const Eigen::Vector<DataType,6> &desiredAcceleration);

        /**
         * Solve the joint torques required to track a joint space trajectory.
         * @param desiredPosition The desired joint position (nx1).
         * @param desiredVelocity The desired joint velocity (nx1).
         * @param desiredAcceleration The desired joint accelerations (nx1).
         * @return The control torque (nx1).
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

};

//////////////////////////////////////////////////////////////////////////////////////////////////////
//                                          Constructor                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
SerialDynamicControl<DataType>::SerialDynamicControl(KinematicTree<DataType> *model, const std::string &endpointName)
    : SerialLinkBase<DataType>(model, endpointName)

{
    this->_jointPositionGain = 225.0;
    this->_jointDerivativeGain = 30.0;
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //               Solve the joint torques required to achieve a given endpoint motion              //
////////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
Eigen::Vector<DataType,Eigen::Dynamic>
SerialDynamicControl<DataType>::resolve_endpoint_motion(const Eigen::Vector<DataType,6> &endpointMotion)
{
    using namespace Eigen;                                                                          // Makes reading code below a bit simpler

    unsigned int numJoints = this->_model->number_of_joints();                                      // Makes things easier

    Vector<DataType,Dynamic> controlTorque(numJoints); controlTorque.setZero();                     // Value to be returned

    Vector<DataType,Dynamic> controlAcceleration(numJoints); controlAcceleration.setZero();         // Value to be calculated

    Vector<DataType,Dynamic> startPoint = QPSolver<DataType>::last_solution();                      // For the QP solver

    Vector<DataType,Dynamic> lowerBound(numJoints), upperBound(numJoints);                          // On the joint control

    for(int i = 0; i < numJoints; ++i)
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
            = QPSolver<DataType>::constrained_least_squares(endpointMotion - this->_model->time_derivative(this->_jacobianMatrix)*this->_model->joint_velocities(), // y
                                                            this->_jacobianMatrix,                  // A
                                                            this->_cartesianStiffness,              // W
                                                            lowerBound,                             // x_min
                                                            upperBound,                             // x_max
                                                            startPoint);                            // Initial guess for solution x
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
            = QPSolver<DataType>::solve(this->_model->joint_inertia_matrix(),                       // H
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

using SerialDynamicControl_f = SerialDynamicControl<float>;
using SerialDynamicControl_d = SerialDynamicControl<double>;

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                  Compute the joint torques needed to track a given trajectory                 //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
Eigen::Vector<DataType,Eigen::Dynamic>
SerialDynamicControl<DataType>::track_endpoint_trajectory(const Pose<DataType>            &desiredPose,
                                                          const Eigen::Vector<DataType,6> &desiredVelocity,
                                                          const Eigen::Vector<DataType,6> &desiredAcceleration)
{
     // NOTE: I just put Pose<DataType> _endpointPose as a member in the base class
     
     return resolve_endpoint_motion(desiredAcceleration
                                  + this->_cartesianStiffness*this->_endpointPose.error(desiredPose)
                                  + this->_cartesianDamping*(desiredVelocity - this->_jacobianMatrix*this->_model->joint_velocities()));
}

  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                    Compute the joint torques needed to track a given trajectory               //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
Eigen::Vector<DataType,Eigen::Dynamic>
SerialDynamicControl<DataType>::track_joint_trajectory(const Eigen::Vector<DataType,Eigen::Dynamic> &desiredPosition,
                                                       const Eigen::Vector<DataType,Eigen::Dynamic> &desiredVelocity,
                                                       const Eigen::Vector<DataType,Eigen::Dynamic> &desiredAcceleration)
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

    Eigen::Vector<DataType,Eigen::Dynamic> accelerationControl(numJoints);
    Eigen::Vector<DataType,8> torqueControl(numJoints);                                // Value to be returned
    accelerationControl = desiredAcceleration                                                                   // Feedforward control
                          +  this->_jointPositionGain*(desiredPosition - this->_model->joint_positions())
                          +  this->_jointDerivativeGain*(desiredVelocity - this->_model->joint_velocities());

    // Obtain the minimum and maximum limits based on the joint position and velocity
    std::vector<Limits<DataType>> controlLimits(numJoints);
    for(int i = 0; i < numJoints; ++i)
    {
        controlLimits[i] = compute_control_limits(i);                         // get the instantaneous limits on the joint acceleration
    }

    // Obtain the effort limits imposed by the motors
    Eigen::Vector<DataType, Eigen::Dynamic> effort_limits(numJoints); effort_limits.setZero();              // Set the initial joint torques
    for(int i = 0; i < numJoints; ++i)
    {
        effort_limits(i) =  this->_model->link(i)->joint().effort_limit();
    }

    Eigen::Vector<DataType,Eigen::Dynamic> lowerBound(numJoints), upperBound(numJoints);                         // On the joint control
    Eigen::Matrix<DataType,Eigen::Dynamic, Eigen::Dynamic> constraintMatrix;
    Eigen::Vector<DataType,Eigen::Dynamic> constraintVector;
    Eigen::Vector<DataType,Eigen::Dynamic> start;

    constraintMatrix.resize(4*numJoints,numJoints);
    constraintMatrix.setZero();
    constraintMatrix.block(0,0,numJoints,numJoints) = this->_model->joint_inertia_matrix();
    constraintMatrix.block(numJoints,0,numJoints,numJoints) = -constraintMatrix.block(0,0,numJoints,numJoints);
    constraintMatrix.block(2*numJoints,0,numJoints,numJoints).setIdentity();
    constraintMatrix.block(3*numJoints,0,numJoints,numJoints).setIdentity();
    constraintMatrix.block(3*numJoints,0,numJoints,numJoints) = -constraintMatrix.block(3*numJoints,0,numJoints,numJoints);

    constraintVector.resize(4*numJoints);
    constraintVector.setZero();
    constraintVector.segment(0,numJoints) = effort_limits+this->_model->joint_gravity_vector();
    constraintVector.segment(numJoints,numJoints) = -(-effort_limits+this->_model->joint_gravity_vector());

    for (int i = 0; i < numJoints; ++i)
    {
        constraintVector(2*numJoints+i) = controlLimits[i].upper;
        constraintVector(3*numJoints+i) = -controlLimits[i].lower;
    }

    start.resize(numJoints);
    start.setZero();
    start = accelerationControl;

    Eigen::Matrix<DataType,Eigen::Dynamic,Eigen::Dynamic> H; H.setIdentity(numJoints,numJoints);            // Solution is unweighted
    Eigen::Vector<DataType,Eigen::Dynamic> f;  f.setZero(numJoints) ;

    f = -H*accelerationControl;

    accelerationControl = QPSolver<DataType>::solve(H, f,
                                                constraintMatrix,                 // B
                                                constraintVector,                 // z
                                                start);

    torqueControl = this->_model->joint_inertia_matrix()*(accelerationControl)
                    + this->_model->joint_coriolis_matrix()*this->_model->joint_velocities()
                    + this->_model->joint_damping_vector(); // Feedback control

    return torqueControl;
}


  ///////////////////////////////////////////////////////////////////////////////////////////////////
 //                   Compute the instantaneous limits on the joint acceleration                  //
///////////////////////////////////////////////////////////////////////////////////////////////////
template <class DataType> inline
Limits<DataType> SerialDynamicControl<DataType>::compute_control_limits(const unsigned int &jointNumber)
{
    // Flacco, F., De Luca, A., & Khatib, O. (2012).
    // "Motion control of redundant robots under joint constraints: Saturation in the null space."
    // IEEE International Conference on Robotics and Automation, 285-292.
    // Calculating the limits given the maximum acceleration is achieved by using the QP solver, outside this function

    Limits<DataType> limits;                                                                        // Value to be returned

    DataType delta = this->_model->joint_positions()[jointNumber]
                     + this->_model->joint_velocities()[jointNumber]*(1.0/this->_controlFrequency)
                     - this->_model->link(jointNumber)->joint().position_limits().lower;             // Distance from lower limit

    limits.lower = std::max((-2*delta)*pow(this->_controlFrequency,2),
                            (-this->_model->link(jointNumber)->joint().speed_limit() + this->_model->joint_velocities()[jointNumber])*(this->_controlFrequency));


    delta = this->_model->link(jointNumber)->joint().position_limits().upper
            - this->_model->joint_velocities()[jointNumber]*(1.0/this->_controlFrequency)
            - this->_model->joint_positions()[jointNumber];                                          // Distance to upper limit

    limits.upper = std::min(abs(2*delta*pow(this->_controlFrequency,2)),
                            (this->_model->link(jointNumber)->joint().speed_limit() - this->_model->joint_velocities()[jointNumber])*(this->_controlFrequency));
    return limits;
}


#endif //SERIALDYNAMICCONTROL_H
