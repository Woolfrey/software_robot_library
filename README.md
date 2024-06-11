# :robot: RobotLibrary
>[!WARNING]
> _RobotLibrary is still under construction_ :construction:

RobotLibrary is a C++ library for modeling and control of robots. It is an ongoing project with more features to come soon.

### Contents:
- Installation Instructions
     - [Installing Eigen](#installing-eigen)
     - [Installing RobotLibrary](#installing-robotlibrary)
     - [Using RobotLibrary in Another Project](#using-robotlibrary-in-another-project)
- Sections of the Library
     - Control :construction:
     - [Math](Math/README.md)
     - [Model](Model/README.md)
     - [Trajectory](Trajectory/README.md)

## Installation Instructions

### Installing Eigen:

RobotLibrary requires the Eigen libraries for linear algebra. If you're using Linux you can install it from the command line:

  `sudo apt install libeigen3-dev`

Otherwise you can go to the [Eigen main page](https://eigen.tuxfamily.org/index.php?title=Main_Page) to see how you can install it.

[:arrow_backward: Go Back.](#contents)

### Installing RobotLibrary:

1. Clone this repository in to your working directory:

   `git clone https://github.com/Woolfrey/software_robot_library.git`
   
2. Navigate in to the folder:

   `cd ~/<your_working_directory>/software_robot_library`

3. Create a build directory and navigate in to it:

   `mkdir build && cd build`

4. Run the following commands in the `build` directory:

   `cmake ../`
   
   `sudo make install`

You should now be able to include different parts of the library in your C++ files.

[:arrow_backward: Go Back.](#contents)

### Using RobotLibrary in Another Project:
When using `RobotLibrary` classes in another project, it is necessary to link both `Eigen` and `RobotLibrary` when compiling executables. For example, we may want to use the `KinematicTree` class `source_file.cpp` of the following example project:
```
/example_project
   |
   -- CMakeLists.txt
   |
   -- /build
   |
   -- /include
   |
   -- /src
      |
      -- example.cpp
```
In the `example.cpp` file we can `#include` the `KinematicTree` header file from the `Model` sublibrary:
```
#include <Model/KinematicTree.h>
...
int main(int argc, char **argv)
{
     KinematicTree model("path/to/robot.urdf");
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
...
add_executable(example src/example.cpp)
target_link_libraries(example RobotLibrary::RobotLibrary Eigen3::Eigen)
```
Inside the `/example_project/build` folder it should  be possible to compile the project:
```
cmake ../
make
```
[:arrow_backward: Go Back.](#contents)
