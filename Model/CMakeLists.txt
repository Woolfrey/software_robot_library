####################################################################################################
#                                              MODEL                                               #
####################################################################################################
find_package(Eigen3 REQUIRED)                                                                       # For linear algebra

include_directories(${EIGEN3_INCLUDE_DIRS})

add_library(Model SHARED src/tinyxml2.cpp)

target_include_directories(Model
	PUBLIC 
		$<INSTALL_INTERFACE:include>    
 		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	PRIVATE
        	${CMAKE_CURRENT_SOURCE_DIR}/src
        	${CMAKE_CURRENT_SOURCE_DIR}/test
)

########################################## Executables #############################################
add_executable(urdf_test test/urdf_test.cpp)                                                        # Create name of exe from source file
add_dependencies(urdf_test RobotLibrary)                                                            # Depends on RobotLibrary (obviously)
target_link_libraries(urdf_test RobotLibrary)                                                       # Link RobotLibrary
