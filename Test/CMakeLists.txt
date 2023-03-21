# This program is used to test and debug code.
cmake_minimum_required(VERSION 3.8)					                            # Minimum cmake requirement
project(RobotLibraryTest)

find_package(Eigen3 REQUIRED)                                                                       # As it says on the label

include_directories( ${EIGEN3_INCLUDE_DIRS}   )                                                     # Needed for linear algebra

########## List of executables to be compiled ##########

#add_executable(serial_link_test serial_link_test.cpp)                                              # Create name of exe from source file
#add_dependencies(serial_link_test RobotLibrary)                                                    # Depends on RobotLibrary (obviously)
#target_link_libraries(serial_link_test RobotLibrary)                                               # Link RobotLibrary

#add_executable(math_test math_test.cpp)
#add_dependencies(math_test RobotLibrary)
#target_link_libraries(math_test RobotLibrary)

add_executable(qp_test qp_test.cpp)
add_dependencies(qp_test RobotLibrary)
target_link_libraries(qp_test RobotLibrary)

add_executable(urdf_test urdf_test.cpp)
add_dependencies(urdf_test RobotLibrary)
target_link_libraries(urdf_test RobotLibrary tinyxml2)
