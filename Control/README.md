# :control_knobs: Control

[:back: Back to the Foyer](../README.md)

This sublibrary contains control classes for robots.

- [Serial Link Base](#serial-link-base)
- [Serial Kinematic Control](#serial-kinematic-control)
- [Release Notes](#package-release-notes---v100-april-2025)

> [!NOTE]
> The first release has velocity control for serial link robot arms. Torque control is coming soon.

## Serial Link Base

The purpose of this class is to provide a common structure to all serial link control classes, and polymorphism to implement different controllers for the same task.

The class requires a _pointer_ to a [Kinematic Tree](../Model/README.md) object, and the name of a valid reference frame (i.e. the endpoint of the robot) in said model:
```
SerialLinkBase(std::shared_ptr<RobotLibrary::Model::KinematicTree> model,
               const std::string &endpointName,
               const RobotLibrary::Control::Parameters &parameters = Parameters());
```
This is done deliberately so that:
1. Two serial link controllers can control different endpoints on the same robot (e.g. a controller for each arm on a humanoid robot), and
2. The model can update the kinematics & dynamics independently in a separate thread.


## Serial Kinematic Control

This class inherits the `SerialLinkBase` class. It provides methods for velocity-level feedback control on joint trajectories, real-time Cartesian control, and feedback control on Cartesian trajectories.

> [!TIP]
> You can check out [my ROS2 action server](https:://github.com/Woolfrey/server_serial_link) to see this class in action.

### Joint Velocity Control

The method:
```
Eigen::VectorXd
track_joint_trajectory(const Eigen::VectorXd &desiredPosition,
                       const Eigen::VectorXd &desiredVelocity,
                       const Eigen::VectorXd desiredAcceleration);
```
computes the control equation:
```math
\dot{\mathbf{q}} = \dot{\mathbf{q}}_d + k_p\left(\mathbf{q}_d - \mathbf{q} \right)
```
where:
- $\dot{\mathbf{q}}\in\mathbb{R}^n$ are the joint velocities to command the robot,
- $\mathbf{q}_d,\dot{\mathbf{q}}_d\in\mathbb{R}^n$ is the desired joint position and joint velocity,
- $\mathbf{q}\in\mathbb{R}^n$ is the current joint position, and
- $k_p\in\mathbb{R}^+$ is the joint position gain.

### Cartesian Velocity Control

The method:
```
Eigen::VectorXd
track_cartesian_trajectory(const RobotLibrary::Model::Pose &desiredPose,
                           const Eigen::Vector<double,6> &desiredVelocity,
                           const Eigen::Vector<double,6> &desiredAcceleration);
```
computes:
```math
\dot{\mathbf{x}} = \dot{\mathbf{x}}_d + \mathbf{K}\left(\mathbf{x}_d - \mathbf{x}\right)
```
where:
- $\mathbf{x}\in\mathbb{R}^6$ is the command twist (Cartesian velocity) for the endpoint of the robot,
- $\mathbf{x}_d,\dot{\mathbf{x}}_d$ is the desired pose & twist,
- $\mathbf{x}\in\mathbb{SE}(3)$ is the current pose of the endpoint, and
- $\mathbf{K}\in\mathbb{R}^{6\times 6}$ is the Cartesian stiffness matrix.

It then immediately calls:
```
Eigen::VectorXd
resolve_endpoint_motion(const Eigen::Vector<double,6> &endpointMotion)
```
which solves a different problem depending on if the robot is redundant or not.

For a non-redundant robot the problem is:
```math
\begin{align}
  \min_\dot{\mathbf{q}} \frac{1}{2}\|\dot{\mathbf{x}} - \mathbf{J}\dot{\mathbf{q}} \|^2 \\
  \text{subject to: } \dot{\mathbf{q}}_{min} \le \dot{\mathbf{q}} \le \dot{\mathbf{q}}_{max} \\
                      \dot{\mu} \ge \gamma \left(\mu - \mu_{min}\right)
\end{align}
```
where:
- $\mathbf{J}\in\mathbb{R}^{6\times n}$ is the Jacobian matrix,
- $\dot{\mathbf{q}}min$ and $\dot{\mathbf{q}}_{max}\in\mathbb{R}^n$ are lower and upper bounds on the joint speed based on joint limits (both position and speed),
- $\mu$ is the measure of manipulability (proximity to a singularity), and
- $\mu_{min}$ is the minimum manipulability threshold.

If the robot is redundant it solves,
```math
\begin{align}
  \min_\dot{\mathbf{q}} \frac{1}{2}\|\dot{\mathbf{q}}_{\varnothing} - \dot{\mathbf{q}} \|_M^2 \\
  \text{subject to: } \dot{\mathbf{x}} = \mathbf{J}\dot{\mathbf{q}} \\
               \dot{\mathbf{q}}_{min} \le \dot{\mathbf{q}} \le \dot{\mathbf{q}}_{max} \\
                      \dot{\mu} \ge -\gamma \left(\mu - \mu_{min}\right)
\end{align}
```
where:
- $\dot{\mathbf{q}}_{\varnothing}\in\mathbb{R}^n$ is a redundant task, and
- the solution is weighted by the inertia matrix $\mathbf{M}(\mathbf{q})\in\mathbb{R}^{n\times n}$.

> [!TIP]
> You can set the redundant task with the `set_redundant_task()` method. If none is provided, it automatically uses the gradient projection method to move away from singularities (where possible).

> [!NOTE]
> If the singularity avoidance fails and the manipulability $\mu$ is below $\mu_{min}$, then the algorithm automatically reverts to a damped-least-squares method to avoid numerical instability.

## :package: Release Notes - v1.0.0 (April 2025)

Tested and implemented:
- `SerialLinkBase`
- `SerialKinematicControl`

