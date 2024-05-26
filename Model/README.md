# Model

>[!WARNING]
> Still under construction :construction:

The model classes are used for computing the kinematics and dynamics of robotic systems.

[:rewind: Back to the foyer.](../README.md)

### Contents:
- [Pose](#pose)
- [RigidBody](#rigidbody)
- [Joint](#joint)
- [Link](#link)
- [KinematicTree](#kinematictree)
     - [Kinemaitcs](#kinematics)
     - [Dynamics](#dynamics)
        - [Fixed-base Mechanisms](#fixed-base-mechanisms)
        - [Floating-base Mechanisms](#floating-base-mechanisms)

## Pose
This class is used to represent the relative position and orientation of an object and/or coordinate frame with respect to another; i.e. the Special Euclidean group $\mathbb{SE}(3)$. It is composed of a 3D `Eigen::Vector` for the translation component, and an `Eigen::Quaternion` for the rotation.

[PLACEHOLDER FOR FIGURE]

- Inverse:
  ```
  pose.inverse();
  ```
- Propagation:
  ```
  RobotLibrary::Pose poseAtoC = poseAtoB * poseBtoC;
  ```
- As a 4x4 homogeneous transformation matrix
  ```
  Eigen::Matrix<type,4,4> T = pose.as_matrix();
  ```
- Error (useful for control):
  ```
  RobotLibrary::Pose actualPose, desiredPose;
  ...
  Eigen::Vector<type,6> poseError = actualPose.error(desiredPose);
  ```
  The first 3 elements is the position error, and the last 3 elements are the orientation error:
  ```
  Eigen::Vector<type,3> positionError = poseError.head(3);
  Eigen::Vector<type,3> orientationError = poseError.tail(3);
  ```


## RigidBody
This class contains the mass and inertia parameters of a single solid object.

## Joint
This class contains information about an actuator on robot.

## Link
This class describes a rigid body that is connected to a joint.

## KinematicTree

This class solves the kinematics and dynamics of branching, serial link mechanisms.

>[!WARNING]
> The KinematicTree can only model open-link chains. It cannot model parallel mechanisms.

You can generate a kinematic & dynamic model of a multi-limb robot from a `.urdf` file:
```
KinematicTree<type> model("path/to/file.urdf");
```
There are 2 convenient typedefs:
1. `KinematicTree_f = KinematicTree<float>`
2. `KinematicTree_d = KinematicTree<double>`

>[!NOTE]
> Before computing any kinematics or dynamics, it is necessary to update the state of the model:
> `model.update_state(jointPositionVector, jointVelocityVector);`

### Kinematics
Forward kinematics:
```math
\mathbf{x = f(q)}
```
```
RobotLibrary::Pose<type> = model.frame_pose("frame_name");
```
Differential kinematics:
```math
\begin{equation}
\mathbf{\dot{x}} = \mathbf{J(q)\dot{q}}
\end{equation}
```
Using the name:
```
Eigen::Matrix<type,6,Eigen::Dynamic> jacobian = model.jacobian("frameName");
```
Using a `ReferenceFrame` data structure:
```
Eigen::Matrix<type,6,Eigen::Dynamic> jacobian = model.jacobian(referenceFrame);
```
Acceleration level:
```math
\mathbf{\ddot{x} = J(q)\ddot{q} + \dot{J}(q,\dot{q})\dot{q}}
```
Time derivative of the Jacobian:
```
Eigen::Matrix<type,6,Eigen::Dynamic> jacobianTimeDerivative = model.time_derivative(jacobian);
```
Partial derivative $\partial\mathbf{J}/\partial\mathrm{q_j}$ where $\mathrm{j}$ is the joint number:
```
Eigen::Matrix<type,6,Eigen::Dynamic> jacobianPartialDerivative = model.partial_derivative(jacobian,j);
```

### Dynamics
#### Fixed-base Mechanisms:
```math
\boldsymbol{\tau} = \mathbf{M(q)\ddot{q} + \left(C(q,\dot{q}) + D\right)\dot{q} + g(q)}
```
```
Eigen::Matrix<type,Eigen::Dynamic,Eigen::Dynamic> M = model.joint_inertia_matrix();
Eigen::Matrix<type,Eigen::Dynamic,Eigen::Dynamic> C = model.joint_coriolis_matrix();
Eigen::Vector<type,Eigen::Dynamic> d = model.joint_damping_vector();
Eigen::Vector<type,Eigen::Dynamic> g = model.joint_gravity_vector();
```
#### Floating-base Mechanisms:

>[!WARNING]
> The accuracy of these calculations have not yet been validated.

```math
\begin{bmatrix}
     \boldsymbol{\tau} \\
     \mathbf{w}
\end{bmatrix}
=
\begin{bmatrix}
     \mathbf{M}_\mathrm{qq} && \mathbf{M}_\mathrm{qb} \\
     \mathbf{M}_\mathrm{bq} && \mathbf{M}_\mathrm{bb}
\end{bmatrix}
\begin{bmatrix}
     \mathbf{\ddot{q}} \\
     \mathbf{\ddot{b}}
\end{bmatrix}
+
\begin{bmatrix}
     \mathbf{C}_\mathrm{qq} & \mathbf{C}_\mathrm{qb} \\
     \mathbf{C}_\mathrm{bq} & \mathbf{C}_\mathrm{bb}
\end{bmatrix}
\begin{bmatrix}
     \mathbf{\dot{q}} \\
     \mathbf{\dot{b}}
\end{bmatrix}
+
\begin{bmatrix}
     \mathbf{g}_\mathrm{q} \\
     \mathbf{g}_\mathrm{b}
\end{bmatrix}
```

The joint-space dynamic properties for the joint-space, e.g. the joint-space inertia matrix $\mathbf{M}_\mathrm{qq}$, is the same as the example above for fixed-base mechanisms.

The robot base is a public class member:
```
RobotLibrary::RigidBody base;
```
so you can use the methods in [RigidBody](#rigidbody) to obtain its dynamic properties.

For the dynamic coupling:
```
Eigen::Matrix<type,Eigen::Dynamic,6> Mqb = joint_base_inertia_matrix();
Eigen::Matrix<type,Eigen::Dynamic,6> Cqb = joint_base_coriolis_matrix();
Eigen::Matrix<type,6,Eigen::Dynamic> Mbq = base_joint_inertia_matrix();
Eigen::Matrix<type,6,Eigen::Dynamic> Cbq = base_joint_coriolis_matrix();
```
>[!TIP]
> Due to symmetry of the inertia matrix `Mbq = Mqb.transpose()`,
> and skew symmetry properties of the Coriolis matrix `Cbq = -Cqb.transpose()`.
> The above functions have been included for completeness.

>[!NOTE]
> To update the dynamics for a floating base mechanism you should call
> `model.update_state(jointPositionVector, jointVelocityVector, basePose, baseVelocity)`
> where `basePose` is a `RobotLibrary::Pose` object, and `baseVelocity` is a 6D `Eigen::Vector` object of the linear and angular velocities (i.e. a twist).
