# This program is used to test and debug code.
cmake_minimum_required(VERSION 3.8)					                            # Minimum cmake requirement
project(RobotLibraryTest)

# if(NOT TARGET RobotLibrary::RobotLibrary)
#     find_package(RobotLibrary CONFIG REQUIRED)
# endif()

find_package(Eigen3 REQUIRED)

# IF (${RobotLibrary_FOUND})
#    MESSAGE(STATUS "Found RobotLibrary")
# ELSE (${RobotLibrary_FOUND})
#    MESSAGE(STATUS "Could not locate RobotLibrary, attempting build step")
# ENDIF (${RobotLibrary_FOUND})

# if(NOT ${RobotLibrary_FOUND})
#    execute_process(COMMAND ${CMAKE_COMMAND} --build .
#      RESULT_VARIABLE result
#      WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
#    if(result)
#      message(FATAL_ERROR "Build step for RobotLibrary failed: ${result}")
#       endif()
# endif()

include_directories( ${EIGEN3_INCLUDE_DIRS})


#include_directories(${ROBOTLIBRARY_INCLUDE_DIRS} ${RobotLibrary_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/../Model/include ${CMAKE_CURRENT_SOURCE_DIR}/../Control/include ${CMAKE_CURRENT_SOURCE_DIR}/../Trajectory/include)

add_executable(robot_library_test robot_library_test.cpp)
add_dependencies(robot_library_test RobotLibrary)
target_link_libraries(robot_library_test RobotLibrary)

#install(TARGETS robot_library_test                                                                  # Specify the source #file#
#  	# IMPORTANT: Add the bar executable to the "export-set"#
#  	EXPORT RobotLibrary
#  	RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin)

