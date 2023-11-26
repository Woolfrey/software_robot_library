####################################################################################################
#                                          TRAJECTORY                                              #
####################################################################################################
find_package(Eigen3 REQUIRED)                                                                       # For linear algebra

include_directories(${EIGEN3_INCLUDE_DIRS})                                                         # I don't know what this does ¯\_(ツ)_/¯

add_library(Trajectory SHARED test/polynomial_test.cpp)                                             # This is a header-only library

# Specify where to find header files
target_include_directories(Trajectory
	PUBLIC
		$<INSTALL_INTERFACE:include>
    		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	PRIVATE
        	${CMAKE_CURRENT_SOURCE_DIR}/src
        	${CMAKE_CURRENT_SOURCE_DIR}/test		
)

########################################## Executables #############################################
add_executable(polynomial_test test/polynomial_test.cpp)                                            # Create name of exe from source file
add_dependencies(polynomial_test RobotLibrary)                                                      # Depends on RobotLibrary (obviously)
target_link_libraries(polynomial_test RobotLibrary)                                                 # Link RobotLibrary
