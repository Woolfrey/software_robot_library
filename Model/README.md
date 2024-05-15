# Model

>[!WARNING]
> Still under construction :construction:

The model classes are used for computing the kinematics and dynamics of robotic systems.

[:rewind: Back to the foyer.](../README.md)

### Contents:
- Pose
- RigidBody
- Joint
- Link
- [KinematicTree](#kinematictree)

## KinematicTree

The `KinematicTree` class solves the kinematics and dynamics of branching, serial link mechanisms.

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
