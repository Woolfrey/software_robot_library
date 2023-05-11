find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_library(Trajectory SHARED src/CartesianTrajectory.cpp
							  src/CubicSpline.cpp
							  src/Polynomial.cpp
							  src/TrajectoryBase.cpp
							  src/TrapezoidalVelocity.cpp)

target_include_directories(Trajectory
	PUBLIC 
		$<INSTALL_INTERFACE:include>
        	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>                              # Location of header files
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/src                                                     # Location of source files
)

target_link_libraries(Trajectory Model)                                                                   # Specify dependent library here
