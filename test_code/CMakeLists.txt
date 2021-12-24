cmake_minimum_required(VERSION 3.8)					# Minimum cmake requirement

project(RobotLibrary)							# Can use ${PROJECT_NAME} as a reference

find_package(Eigen3 3.3 REQUIRED)					# Needed for linear algebra


include_directories(
	../Control							# Include these folders from the parent directory
	../Robot
	../Trajectory
	)						


# This program is used to test and debug code.
add_executable(robot_library_test robot_library_test.cpp)
target_link_libraries(robot_library_test Eigen3::Eigen)
