add_library(Trajectory INTERFACE)                                                                   # This is a header-only library

# Specify where to find header files
target_include_directories(Trajectory
	INTERFACE
    		$<INSTALL_INTERFACE:include>
    		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
