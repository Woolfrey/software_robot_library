cmake_minimum_required(VERSION 3.8)					                            # Minimum cmake requirement

project(RobotLibrary)							                            # Can use ${PROJECT_NAME} as a reference

find_package(Eigen3 REQUIRED)

# Other directories to add to the build
include_directories(
	Control
	Robot
	Trajectory
 )

add_subdirectory(Test)                                                                              # Location of other CMakeLists files


###################################################################################################

# Make relative paths absolute (to prevent CMake warning when building)
foreach(p LIB BIN INCLUDE CMAKE)
  set(var INSTALL_${p}_DIR)
  if(NOT IS_ABSOLUTE "${${var}}")
    set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
  endif()
endforeach()

export(TARGETS robot_library_test                                                                   # Target source files
       FILE "${PROJECT_BINARY_DIR}/RobotLibraryTargets.cmake")                                      # Location

export(PACKAGE RobotLibrary)                                                                        # Register with CMake so others can find it

# Create the RobotLibraryConfig.cmake and RobotLibraryConfigVersion files
file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}" "${INSTALL_INCLUDE_DIR}")
  
set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}")                              # ... for the build tree
configure_file(RobotLibraryConfig.cmake.in "${PROJECT_BINARY_DIR}/RobotLibraryConfig.cmake" @ONLY)
  
set(CONF_INCLUDE_DIRS "\${ROBOTLIBRARY_CMAKE_DIR}/${REL_INCLUDE_DIR}")                              # ... for the install tree
configure_file(RobotLibraryConfig.cmake.in "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/RobotLibraryConfig.cmake" @ONLY)
  
configure_file(RobotLibraryConfigVersion.cmake.in "${PROJECT_BINARY_DIR}/RobotLibraryConfigVersion.cmake" @ONLY) # ... for both

# Install the RobotLibraryConfig.cmake and RobotLibraryConfigVersion.cmake
install(FILES
	"${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/RobotLibraryConfig.cmake"
	"${PROJECT_BINARY_DIR}/RobotLibraryConfigVersion.cmake"
	DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)

# Install the export set for use with the install-tree
install(EXPORT RobotLibraryTargets DESTINATION
  	"${INSTALL_CMAKE_DIR}" COMPONENT dev)

