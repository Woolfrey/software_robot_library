# :robot: RobotLibrary

RobotLibrary is a C++ package for modeling, trajectory generation, and control of robots. The [initial release](#package-release-notes---v100-april-2025) supports real-time velocity control of serial link robot arms (work on torque control is underway). The interfaces are designed to provide simple inputs and outputs, without needing to know anything about the algorithms underneath.

The modular design means you can utilize different components to develop your own controllers. For example, you can inherit the `SerialLinkBase` class in the [Control sub-library](Control/README.md) and implement your own algorithms for the joint and Cartesian control methods. Or, you can use the `KinematicTree` in the [Model sub-library](Model/README.md) for the inverse dynamics and write your own controller from scratch.

- [Sections of the Library](#classical_building-sections-of-the-library)
- [Installation](#clipboard-installation)
    - [Requirements](#requirements)
    - [Installing Eigen](#installing-eigen)
    - [Installing RobotLibrary](#installing-robotlibrary)
- [Using RobotLibrary](#rocket-using-robotlibrary)
    - [In Another Project](#in-another-project)
    - [Examples](#examples)
- [Release Notes](#package-release-notes---v100-april-2025)
- [Contributing](#handshake-contributing)
- [License](#scroll-license)

## :classical_building: Sections of the Library

- [Control](Control/README.md): Classes for real-time, feedback control.
- [Math](Math/README.md): Supporting functions & classes for other parts of the library.
- [Model](Model/README.md): Classes for computing the kinematics & dynamics of rigid-body structures.
- [Trajectory](Trajectory/README.md): Classes for generating paths through space & time.

[:top: Back to Top.](#robot-robotlibrary)

## :clipboard: Installation

### Requirements:

- CMake 3.14 or higher
- C++17 or higher 
- Eigen3 v3.4 or higher

### Installing Eigen:

> [!NOTE]
> You need to manually install Eigen 3.4 if you're using Ubuntu 20.04.

#### Ubuntu 20.04

1. First ensure prerequisites are installed:

    `sudo apt update`
    
    `sudo apt install -y build-essential cmake git`
    
2. Download version 3.4 directly (or from the webpage):

    `wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz`

3. Extract the downloaded file:

    `tar -xvf eigen-3.4.0.tar.gz`
    
    `cd eigen-3.4.0`
    
4. Build and install:

    `mkdir build && cd build`
    
    `cmake ../`
    
    `sudo make install`

#### Ubuntu 22.04 & Later

Eigen 3.4 is automatically installed on later versions of Ubuntu. In the command line you can run:

  `sudo apt install libeigen3-dev`

### Installing RobotLibrary:

1. Clone this repository in to your working directory:

   `git clone https://github.com/Woolfrey/software_robot_library.git`
   
2. Navigate in to the folder:

   `cd ~/<your_working_directory>/software_robot_library`

3. Create a build directory and navigate in to it:

   `mkdir build && cd build`

4. Run the following commands in the `build` directory:

   `cmake ..`
   
   `sudo make install`

You should now be able to include different parts of the library in your C++ files.

[:top: Back to Top.](#robot-robotlibrary)

## :rocket: Using RobotLibrary

### In Another Project:

When using `RobotLibrary` classes in another project, it is necessary to link both `Eigen` and `RobotLibrary` when compiling executables. For example, we may want to use the `KinematicTree` class in the `example.cpp` of the following example project:

```
example_project/
├── build/
├── src/
|   └── example.cpp
└── CMakeLists.txt
```

In the `example.cpp` file we can include the `KinematicTree` header file under `RobotLibrary`:

```
#include <RobotLibrary/Model/KinematicTree.h>
...
int main(int argc, char **argv)
{
     RobotLibrary::Model::KinematicTree model("path/to/robot.urdf");
}
```

Then, in the `CMakeLists.txt` file, we must:
1. Tell the compiler to find both `Eigen3` and `RobotLibrary`, and
2. Link `RobotLibrary` and `Eigen3` to the executable that uses any `RobotLibrary` classes:

```
cmake_minimum_required(VERSION 3.8)
project(example)
...
find_package(Eigen3 REQUIRED)
find_package(RobotLibrary REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR}) 
...
add_executable(example src/example.cpp)
target_link_libraries(example RobotLibrary::RobotLibrary Eigen3::Eigen)
```

Inside the `example_project/build` folder it should  be possible to compile the project:

```
cmake ..
make
```

### Examples:

If you would like to see examples where `RobotLibrary` has been applied, you can check out:

- [Serial Link Action Server](https://github.com/Woolfrey/server_serial_link) : My own ROS2 action servers for control.
- [TestingRobotLibrary](https://github.com/Woolfrey/testing_robot_library) : C++ executables I use for numerical validation of RobotLibrary.

[:top: Back to Top.](#robot-robotlibrary)

## :package: Release Notes - v1.0.0 (April 2025)

### :tada: Initial Release:
- Control:
     - SerialLinkBase : Base class providing common structure for all child classes.
     - SerialKinematicControl : Joint velocity & Cartesian velocity control algorithms.
- Math:
     - MathFunctions : Helper functions for other classes.
     - Polynomial : Generates a scalar, polynomial function.
     - SkewSymmetric: Converted an `Eigen::Vector3d` object to an anti-symmetric `Eigen::Matrix3d` object.
     - Spline : Connects multiple points using polynomials, whilst ensuring continuity.
- Model:
     - Joint : Models an actuated joint on a robot.
     - KinematicTree : Kinematic & dynamic modeling of branching, serial-link structures.
     - Link : Represents a combined `Joint` and `RigidBody` object.
     - Pose : Position and orientation; $\mathbb{SE}(3)$.
     - RigidBody : Dynamics for a single, solid object.
- Trajectory:
     - CartesianSpline : Generates smooth trajectories over a series of poses.
     - SplineTrajectory : Generates smooth trajectories over a series of points.
     - TrajectoryBase : Provides common structure to all trajectory classes.
     - TrapezoidalVelocity : Trajectory with constant sections, and ramps up and down.
  
[:top: Back to Top.](#robot-robotlibrary)
    
## :handshake: Contributing

Contributions are welcome! Feel free to fork the library, make your own improvements, and issue a pull request.

[:top: Back to Top.](#robot-robotlibrary)

## :scroll: License

[:top: Back to Top.](#robot-robotlibrary)
