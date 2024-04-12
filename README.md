# :robot: RobotLibrary

RobotLibrary is a C++ library for modeling and control of robots. It is an ongoing project with more features to come soon.

### Contents:
- Installation Instructions :clipboard: 
     - [Installing Eigen](#installing-eigen)
     - [Installing RobotLibrary](#installing-robotlibrary)
- Sections of the Library :books:
     - Control :construction: _Under Construction_ :construction:
     - [Math](Math/README.md)
     - [Model](Model/README.md)
     - [Trajectory](Trajectory/README.md)

## Installation Instructions :clipboard:

### Installing Eigen
RobotLibrary requires the Eigen libraries for linear algebra. If you're using Linux you can install it from the command line:

  `sudo apt install libeigen3-dev`

Otherwise you can go to the [Eigen main page](https://eigen.tuxfamily.org/index.php?title=Main_Page) to see how you can install it.

[:arrow_backward: Go Back.](#contents)

### Installing RobotLibrary

1. Clone this repository in to your working directory:

   `git clone https://github.com/Woolfrey/software_robot_library.git`
   
2. Navigate in to the folder:

   `cd ~/<your_working_directory>/software_robot_library`

3. Create a build directory and navigate in to it:

   `mkdir build && cd build`

4. Run the following commands in the `build` directory:

   `cmake ../`
   
   `make`

You should now be able to include different parts of the library in your C++ files, for example:

  `#include <KinematicTree.h>`

[:arrow_backward: Go Back.](#contents)
