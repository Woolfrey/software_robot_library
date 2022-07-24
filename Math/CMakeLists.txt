find_package(Eigen3 REQUIRED)

include_directories(include SHARED ${EIGEN3_INCLUDE_DIRS})


add_library(Math SHARED src/Math.cpp src/QPSolver.cpp)                                              # Location of source files to add to library

target_include_directories(Math
	PUBLIC
		$<INSTALL_INTERFACE:include>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>                              # Location of header files
	PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/src                                                     # Location of source files
)

target_link_libraries(Math)                                                                        # Specify dependent libraries here
