# :robot: RobotLibrary

RobotLibrary is a C++ library for modeling and control of robots arms.

It is an ongoing project and we hope to add more features as time goes on.

### Contents:
- Installation Instructions
     - [Installing Eigen](#installing-eigen)
     - [Installing RobotLibrary](#installing-robotlibrary)
     - [Using RobotLibrary in Another Project](#using-robotlibrary-in-another-project)
- Sections of the Library
     - [Control](Control/README.md)
     - [Math](Math/README.md)
     - [Model](Model/README.md)
     - [Trajectory](Trajectory/README.md)

## Installation Instructions

### Installing Eigen:

RobotLibrary uses Eigen 3.4. Installation procedure depends on the version of Ubuntu. For detailed information on installation instructions you can visit Otherwise you can go to the [Eigen main page](https://eigen.tuxfamily.org/index.php?title=Main_Page).

#### Ubuntu 20.04

If you're still using Ubuntu 20.04, you need to manually install Eigen 3.4.

First ensure prerequisites are installed:
```
sudo apt update
sudo apt install -y build-essential cmake git
```

Download version 3.4 directly (or from the webpage):
```
wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
```

Extract the downloaded file:
```
tar -xvf eigen-3.4.0.tar.gz
cd eigen-3.4.0
```

Now build and install:
```
mkdir build
cd build
cmake ..
sudo make install
```

#### Ubuntu 22.04 and later

Eigen 3.4 is automatically installed on later versions of Ubuntu. In the command line you can run:

  `sudo apt install libeigen3-dev`

[:arrow_backward: Go Back.](#contents)

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

[:arrow_backward: Go Back.](#contents)

### Using RobotLibrary in Another Project:
When using `RobotLibrary` classes in another project, it is necessary to link both `Eigen` and `RobotLibrary` when compiling executables. For example, we may want to use the `KinematicTree` class in the `example.cpp` of the following example project:
```
example_project/
├── CMakeLists.txt
├── build/
└── src/
    └── example.cpp
```
In the `example.cpp` file we can include the `KinematicTree` header file under `RobotLibrary`:
```
#include <RobotLibrary/KinematicTree.h>
...
int main(int argc, char **argv)
{
     RobotLibrary::KinematicTree model("path/to/robot.urdf");
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
[This repository](https://github.com/Woolfrey/testing_robot_library) has some simple test code that demonstrates the use of different classes.

[:arrow_backward: Go Back.](#contents)
