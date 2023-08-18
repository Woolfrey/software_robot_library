cmake_minimum_required(VERSION 3.8)					                            # Minimum cmake requirement

project(RobotLibrary VERSION 1.0.0 LANGUAGES CXX)                                                   # Can use ${PROJECT_NAME} as a reference

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_BUILD_TYPE Release)

# Default build is Release. To build in other modes use:
#	Debug		(Information, not optimised)
#	Release		(No information, optimised)
#	RelWithDebInfo	(Information & optimised)
#	MinSizeRel	(Same as Release, but optimised for size)

#################################### Download QPSolver #############################################

if(EXISTS "${CMAKE_SOURCE_DIR}/Math/include/QPSolver.h")
	message([STATUS] " Found the QPSolver class in ${CMAKE_SOURCE_DIR}/Math/include/")
else()
	message([WARNING] " QPSolver class not found in ${CMAKE_SOURCE_DIR}/Math/include/")
	
	message([STATUS] " Downloading from https://github.com/Woolfrey/SimpleQPSolver ...")
	
	file(DOWNLOAD
	    https://raw.githubusercontent.com/Woolfrey/SimpleQPSolver/master/include/QPSolver.h
	    ${CMAKE_SOURCE_DIR}/Math/include/QPSolver.h)
	    
	if(EXISTS "${CMAKE_SOURCE_DIR}/Math/include/QPSolver.h")
		message([STATUS] " ... Done!")
	else()
		message([FATAL ERROR] " Could not download QPSolver.")
	endif()
endif()

####################################################################################################
add_library(RobotLibrary INTERFACE)                                                                 # Declaration of RobotLibrary  

# Specify location of other CMakeLists.txt files
add_subdirectory(Control)
add_subdirectory(Math)
add_subdirectory(Model)
add_subdirectory(Trajectory)

target_link_libraries(RobotLibrary INTERFACE Control Math Model Trajectory)                         # Link the subdirectories

################################### Installation Instructions ######################################

include(GNUInstallDirs)

set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/RobotLibrary)                                   # Declare the location for installing config files

install(TARGETS             RobotLibrary Control Math Model Trajectory                              # Specify targets to be installed
	EXPORT              RobotLibrary-targets
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# Export the targets to a script
install(EXPORT      RobotLibrary-targets
	FILE        RobotLibraryTargets.cmake
	NAMESPACE   RobotLibrary::
	DESTINATION ${INSTALL_CONFIGDIR}
)

# Create a ConfigVersion.cmake file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
	${CMAKE_CURRENT_BINARY_DIR}/RobotLibraryConfigVersion.cmake
	VERSION ${PROJECT_VERSION}
	COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/RobotLibraryConfig.cmake.in
	${CMAKE_CURRENT_BINARY_DIR}/RobotLibraryConfig.cmake
	INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
)

# Install the config, config version and custom find modules
install(FILES
	${CMAKE_CURRENT_BINARY_DIR}/RobotLibraryConfig.cmake
	${CMAKE_CURRENT_BINARY_DIR}/RobotLibraryConfigVersion.cmake
	DESTINATION ${INSTALL_CONFIGDIR}
)

################################# Export from the Build Tree #######################################
export(EXPORT RobotLibrary-targets
	FILE ${CMAKE_CURRENT_BINARY_DIR}/RobotLibraryTargets.cmake
	NAMESPACE RobotLibrary::
)

export(PACKAGE RobotLibrary)                                                                        # Register package so other projects can find it
