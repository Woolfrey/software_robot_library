find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_library(Trajectory INTERFACE)

target_link_libraries(Math)

#target_include_directories(Trajectory
#	PUBLIC 
#		$<INSTALL_INTERFACE:include>
#        	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>                              # Location of header files
#	PRIVATE
#		${CMAKE_CURRENT_SOURCE_DIR}/src                                                     # Location of source files
#)

#target_link_libraries(Trajectory Math)                                                              # Specify dependent library here
