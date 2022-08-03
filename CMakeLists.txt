cmake_minimum_required(VERSION 3.8)					                               # Minimum cmake requirement

project(RobotLibrary VERSION 1.0.0 LANGUAGES CXX)                                                   # Can use ${PROJECT_NAME} as a reference

# Other directories to add to the build

add_library(RobotLibrary INTERFACE)                                                                 # Declaration of RobotLibrary

add_subdirectory(Control)
add_subdirectory(Math)
add_subdirectory(Model)
add_subdirectory(Trajectory)

target_link_libraries(RobotLibrary INTERFACE Control Math Model Trajectory)                         # Add these to the library

##############################################
# Installation instructions

include(GNUInstallDirs)

set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/RobotLibrary)

install(TARGETS RobotLibrary Control Math Model Trajectory                                          # Specify targets to be installed
	EXPORT RobotLibrary-targets
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

#Export the targets to a script
install(EXPORT RobotLibrary-targets
	FILE
		RobotLibraryTargets.cmake
	NAMESPACE
		RobotLibrary::
	DESTINATION
		#${CMAKE_INSTALL_LIBDIR}/cmake/RobotLibrary
		${INSTALL_CONFIGDIR}
)

#Create a ConfigVersion.cmake file
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

#Install the config, config version and custom find modules
install(FILES
	${CMAKE_CURRENT_BINARY_DIR}/RobotLibraryConfig.cmake
	${CMAKE_CURRENT_BINARY_DIR}/RobotLibraryConfigVersion.cmake
	DESTINATION ${INSTALL_CONFIGDIR}
)

##############################################
## Exporting from the build tree
export(EXPORT RobotLibrary-targets
	FILE ${CMAKE_CURRENT_BINARY_DIR}/RobotLibraryTargets.cmake
	NAMESPACE RobotLibrary::
)

#Register package in the User Package Registry
export(PACKAGE RobotLibrary)

add_subdirectory(Test)
