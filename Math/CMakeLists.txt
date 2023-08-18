add_library(Math INTERFACE)                                                                         # Declare Math as a header-only library

# Specify where to find header files
target_include_directories(Math
	INTERFACE
    	$<INSTALL_INTERFACE:include>
    	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
