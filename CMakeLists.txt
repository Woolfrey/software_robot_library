cmake_minimum_required(VERSION 3.8)					                            # Minimum cmake requirement

project(RobotLibrary VERSION 1.0.0 LANGUAGES CXX)                                                   # Can use ${PROJECT_NAME} as a reference

# Default build is Release. To build in other modes use:
#
#	cmake -DCMAKE_BUILD_TYPE=Type ../
#
# where Type is:
#	Debug		(Information, not optimised)
#	Release		(No information, optimised)
#	RelWithDebInfo	(Information & optimised)
#	MinSizeRel	(Same as Release, but optimised for size)
#
# Then run `make` as usual.

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_library(RobotLibrary INTERFACE Math Trajectory)                                                 # Declaration of RobotLibrary

########## Other directories to add to the build ##########
#add_subdirectory(Control)
add_subdirectory(Math)
#add_subdirectory(Model)
add_subdirectory(Trajectory)

target_link_libraries(RobotLibrary INTERFACE Math Trajectory)                                       # Link the subdirectories

##############################################
# Installation instructions

include(GNUInstallDirs)

set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/RobotLibrary)

install(TARGETS RobotLibrary Math Trajectory                                                        # Specify targets to be installed
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

export(PACKAGE RobotLibrary)                                                                        # Register package so otherr projects can find it

########## Location of Other CMakeLists.txt files ##########
add_subdirectory(Test)                                                                              # Should this be here?
