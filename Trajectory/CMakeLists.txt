find_package(Eigen3 REQUIRED)

include_directories(include SHARED ${EIGEN3_INCLUDE_DIRS})

add_library(Trajectory SHARED src/TrajectoryBase.cpp src/Polynomial.cpp src/CubicSpline.cpp)

target_include_directories(Trajectory
	PUBLIC 
		$<INSTALL_INTERFACE:include>
        	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>                              # Location of header files
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/src                                                     # Location of source files
)

target_link_libraries(Trajectory)                                                                   # Specify dependent library here
