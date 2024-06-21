####################################################################################################
#                                         ROBOT LIBRARY                                            #
#                                       └[∵┌] └|∵|┘ [┐∵]┘                                         #
####################################################################################################

cmake_minimum_required(VERSION 3.8)					                                        # As it says on the label

project(RobotLibrary                                                                                # Reference to ${PROJECT_NAME} below
        DESCRIPTION "Modeling and control classes for robots [┐∵]┘"
        LANGUAGES CXX
        VERSION 1.0
)

set(CMAKE_CXX_STANDARD 17)                                                                          # For structured bindings
set(CMAKE_DEBUG_POSTFIX _debug)                                                                     # Adds '_debug' to all .so and .dll files
set(CMAKE_BUILD_TYPE Release)                                                                       # Release = optimised, debug = detailed info, no optimisation

find_package(Eigen3 REQUIRED)                                                                       # Necessary for linear algebra
include_directories(${EIGEN3_INCLUDE_DIRS})

#################################### Download QPSolver #############################################

if(EXISTS "${CMAKE_SOURCE_DIR}/Math/include/Math/QPSolver.h")
	message([STATUS] " Found the QPSolver class in ${CMAKE_SOURCE_DIR}/Math/include/Math")
else()
	message([WARNING] " QPSolver class not found in ${CMAKE_SOURCE_DIR}/Math/include/Math")
	
	message([STATUS] " Downloading from https://github.com/Woolfrey/software_simple_qp ...")
	
	file(DOWNLOAD
	    https://raw.githubusercontent.com/Woolfrey/software_simple_qp/master/include/QPSolver.h
	    ${CMAKE_SOURCE_DIR}/Math/include/Math/QPSolver.h)
	    
	if(EXISTS "${CMAKE_SOURCE_DIR}/Math/include/Math/QPSolver.h")
		message([STATUS] " ... Done!")
	else()
		message([FATAL ERROR] " Could not download QPSolver.")
	endif()
endif()

###################################### Declare library #############################################

# Location of other CMakeLists.txt files
add_subdirectory(Control)
add_subdirectory(Math)
add_subdirectory(Model)
add_subdirectory(Trajectory)

add_library(${PROJECT_NAME} INTERFACE)                                                              # Refered to by all `target` commands below 

target_link_libraries(RobotLibrary INTERFACE Control Math Model Trajectory)
#target_link_libraries(RobotLibrary INTERFACE Control Math Model Trajectory)                       # Link the subdirectories to RobotLibrary

################################### Installation Instructions ######################################

include(GNUInstallDirs)                                                                             # Defines canonical install paths
include(CMakePackageConfigHelpers)                                                                  # Functions for creating a package

set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})                                # Create variable for install path

install(TARGETS ${PROJECT_NAME} Control Math Model Trajectory
#install(TARGETS ${PROJECT_NAME} Control Math Model Trajectory                                       # Specify targets to be installed
	    EXPORT  ${PROJECT_NAME}Targets                                                              # Name of target exports
	    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}                                                # For STATIC libraries
	    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}                                            # Installs the `include` directories
	    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}                                                # For SHARED libraries
	    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR}                                                # For executables?
)

install(EXPORT      ${PROJECT_NAME}Targets                                                          # The thing to be exported
        FILE        ${PROJECT_NAME}Targets.cmake                                                    # What we're exporting it into
        NAMESPACE   ${PROJECT_NAME}::
        DESTINATION ${INSTALL_CONFIGDIR})                                                           # Location

# Create version file concurrent with this project version
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake VERSION ${PROJECT_VERSION}
                                                                                                COMPATIBILITY AnyNewerVersion)
# Generate config files
configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/${PROJECT_NAME}Config.cmake.in        # Instructions located here?
	                         ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake               # Config file to be created
	                         INSTALL_DESTINATION ${INSTALL_CONFIGDIR})                             # As says
	                         
# Install config files
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
	         ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
	   DESTINATION ${INSTALL_CONFIGDIR})               

# Export the package
export(EXPORT    ${PROJECT_NAME}Targets
	  FILE      ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake
	  NAMESPACE ${PROJECT_NAME}::)
	  
export(PACKAGE ${PROJECT_NAME})
