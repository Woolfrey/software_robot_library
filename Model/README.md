# Model

:construction: This page is under construction. :construction:

The model classes are used for computing the kinematics and dynamics of robotic systems.

[:rewind: Back to the foyer.](../README.md)

### Contents:
- Pose
- RigidBody
- Joint
- Link
- KinematicTree

## KinematicTree

You can generate a kinematic & dynamic model of a multi-limb robot from a `.urdf` file:

```
#include <KinematicTree.h>

int main(int argc, char **argv)
{
     std::string filePath = "/path/to/file.urdf";
     KinematicTree<double> model(filePath);

     ...
}
```
You can also use floats `KinematicTree<float>`.
