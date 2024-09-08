#ifndef SERIALDYNAMICCONTROL_H
#define SERIALDYNAMICCONTROL_H

#include <SerialLinkBase.h>

class SerialDynamicControl : public SerialLinkBase
{
    public:
        /**
         * Constructor.
         * @param model A pointer to a KinematicTree object.
         * @param endpointName The name of the reference frame in the KinematicTree to be controlled.
         */
        SerialDynamicControl(KinematicTree *model, const std::string &endpointName);

        /**
         * Solve the joint torques required to move the endpoint at a given acceleration.
         * @param endpointMotion An acceleration vector (linear & angular acceleration).
         * @return A nx1 vector of joint torques.
         */
        Eigen::VectorXd
        resolve_endpoint_motion(const Eigen::Vector<double,6> &endpointMotion);

        /**
         * Solve the joint torques required to track a Cartesian trajectory.
         * @param desiredPose The desired position & orientation (pose) for the endpoint.
         * @param desiredVelocity The desired linear & angular velocity (twist) for the endpoint.
         * @param desiredAcceleration The desired linear & angular acceleration for the endpoint.
         * @return The joint torques (nx1) required to track the trajectory.
         */
        Eigen::VectorXd
        track_endpoint_trajectory(const Pose                    &desiredPose,
                                  const Eigen::Vector<double,6> &desiredVelocity,
                                  const Eigen::Vector<double,6> &desiredAcceleration);

        /**
         * Solve the joint torques required to track a joint space trajectory.
         * @param desiredPosition The desired joint position (nx1).
         * @param desiredVelocity The desired joint velocity (nx1).
         * @param desiredAcceleration The desired joint accelerations (nx1).
         * @return The control torque (nx1).
           */
        Eigen::VectorXd
        track_joint_trajectory(const Eigen::VectorXd &desiredPosition,
                               const Eigen::VectorXd &desiredVelocity,
                               const Eigen::VectorXd &desiredAcceleration);

    protected:

        /**
         * Compute the instantaneous limits on the joint velocity control.
         * It computes the minimum between joint positions, speed, and acceleration limits.
         * @param jointNumber The joint for which to compute the limits.
         * @return The upper and lower joint speed at the current time.
         */
        Limits compute_control_limits(const unsigned int &jointNumber);
};

#endif
