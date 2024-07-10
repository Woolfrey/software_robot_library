cmake_minimum_required(VERSION 3.12)

# Define the project
project(RobotLibrary                                                                                        # ${PROJECT_NAME} refers to this
        DESCRIPTION "Modeling and control of robots."
        VERSION 1.0
        LANGUAGES CXX)

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
####################################################################################################

# Add subdirectories
add_subdirectory(Control)
add_subdirectory(Math)
add_subdirectory(Model)
add_subdirectory(Trajectory)

########################################### INSTALL ################################################
include(GNUInstallDirs)

include(CMakePackageConfigHelpers)                                                                  # Create and install the package config file

write_basic_package_version_file("${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
                                 VERSION ${PROJECT_VERSION}
                                 COMPATIBILITY AnyNewerVersion
)

configure_package_config_file("${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in"
                              "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
                              INSTALL_DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/cmake
)

install(FILES      "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
                   "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
        DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/cmake
)

############################################ EXPORT ################################################

export(PACKAGE ${PROJECT_NAME})                                                                     # Register the package in the user's package registry

enable_testing()                                                                                    # Optionally, if using CTest for testing
