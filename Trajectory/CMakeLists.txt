####################################################################################################
#                                          TRAJECTORY                                              #
####################################################################################################
project(Trajectory)                                                                                 # Refered to by ${PROJECT_NAME} below

# Generate target reference,
# list associated headers containing class definitions
add_library(${PROJECT_NAME} SHARED
            include/Trajectory/Polynomial.h
            include/Trajectory/MultiTrapezoid.h
            include/Trajectory/Spline.h
            include/Trajectory/TrajectoryBase.h
            include/Trajectory/TrapezoidalVelocity.h
            include/Trajectory/Waypoints.h
)

set_target_properties(${PROJECT_NAME} PROPERTIES CXX_VISIBILITY_PRESET "hidden"                     # Helps reduce size of library?
                                                 LINKER_LANGUAGE CXX                                # Need to tell the linker this is C++
                                                 VISIBILITY_INLINES_HIDDEN TRUE                     # Hide inline functions to reduce size of library
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)                                          # Define the C++ standard needed to compile this library

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src                  # Set source files as private
        	                                         ${CMAKE_CURRENT_SOURCE_DIR}/test                 # Set test files as private
                                           PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> # Reference the local include directory when building this project
                                                  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>       # Reference install/include directory after the fact
)

####################################################################################################

add_executable(polynomial_test test/polynomial_test.cpp)                                            # Create name of exe from source file
add_dependencies(polynomial_test RobotLibrary)                                                      # Depends on RobotLibrary (obviously)
target_link_libraries(polynomial_test RobotLibrary)                                                 # Link RobotLibrary

add_executable(trapezoidal_test test/trapezoidal_test.cpp)                                          # Create name of exe from source file
add_dependencies(trapezoidal_test RobotLibrary)                                                     # Depends on RobotLibrary (obviously)
target_link_libraries(trapezoidal_test RobotLibrary)                                                # Link RobotLibrary

add_executable(spline_test test/spline_test.cpp)                                                    # Create name of exe from source file
add_dependencies(spline_test RobotLibrary)                                                          # Depends on RobotLibrary (obviously)
target_link_libraries(spline_test RobotLibrary)                                                     # Link RobotLibrary

add_executable(multi_trapezoid_test test/multi_trapezoid_test.cpp)                                  # Create name of exe from source file
add_dependencies(multi_trapezoid_test RobotLibrary)                                                 # Depends on RobotLibrary (obviously)
target_link_libraries(multi_trapezoid_test RobotLibrary)                                            # Link RobotLibrary
