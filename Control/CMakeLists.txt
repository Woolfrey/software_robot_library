add_library(Control INTERFACE)

# Specify where to find header files
target_include_directories(Control
	INTERFACE
    	$<INSTALL_INTERFACE:include>
    	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
