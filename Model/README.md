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
     - [Forward and Differential Kinematics](#forward--differential-kinematics)
     - [Inverse Dynamics of Fixed-base Mechanisms](#inverse-dynamics-of-fixed-base-mechanisms)

## Pose
This class is used to represent the relative position and orientation of an object and/or coordinate frame with respect to another.

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
#include <Model/KinematicTree.h>

int main(int argc, char **argv)
{
     std::string filePath = "/path/to/file.urdf";    // Location of the urdf file
     KinematicTree_f model(filePath);                // This creates a model using float data type

     ...
}
```
There are 2 convenient typedefs:
1. `KinematicTree_f = KinematicTree<float>`
2. `KinematicTree_d = KinematicTree<double>`

### Forward & Differential Kinematics:
```math
\mathbf{x = f(q)}
```

```math
\begin{equation}
\mathbf{\dot{x}} = \mathbf{J(q)\dot{q}}
\end{equation}
```

### Inverse Dynamics of Fixed-base Mechanisms:
```math
\boldsymbol{\tau} = \mathbf{M(q)\ddot{q} + \left(C(q,\dot{q}) + D\right)\dot{q} + g(q)}
```
