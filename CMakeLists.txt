cmake_minimum_required(VERSION 3.14)

# Define the project
project(RobotLibrary
        DESCRIPTION "Modeling and control of robots."
        VERSION 1.0
        LANGUAGES CXX)

#################################### Download QPSolver #############################################

if(EXISTS "${CMAKE_SOURCE_DIR}/Math/include/Math/QPSolver.h")
    message(STATUS "Found the QPSolver class in ${CMAKE_SOURCE_DIR}/Math/include/Math")
else()
    message(WARNING "QPSolver class not found in ${CMAKE_SOURCE_DIR}/Math/include/Math")
    
    message(STATUS "Downloading from https://github.com/Woolfrey/software_simple_qp ...")
    
    file(DOWNLOAD
        https://raw.githubusercontent.com/Woolfrey/software_simple_qp/master/include/QPSolver.h
        ${CMAKE_SOURCE_DIR}/Math/include/Math/QPSolver.h)
        
    if(EXISTS "${CMAKE_SOURCE_DIR}/Math/include/Math/QPSolver.h")
        message(STATUS "... Done!")
    else()
        message(FATAL_ERROR "Could not download QPSolver.")
    endif()
endif()

####################################################################################################

# Add subdirectories
add_subdirectory(Control)
add_subdirectory(Math)
add_subdirectory(Model)
add_subdirectory(Trajectory)

# Create an interface library that combines all the sub-libraries
add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(${PROJECT_NAME} INTERFACE Control Math Model Trajectory)

####################################################################################################

include(CMakePackageConfigHelpers)                                                                  # Enable CMake's support for exporting targets

# Configure the installation of the package
install(TARGETS  ${PROJECT_NAME} Control Math Model Trajectory
        EXPORT   ${PROJECT_NAME}Targets
        LIBRARY  DESTINATION lib
        ARCHIVE  DESTINATION lib
        RUNTIME  DESTINATION bin
        INCLUDES DESTINATION include
)

# Generate the package configuration file
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in
                              ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
                              INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
)

# Install the package configuration file
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
        DESTINATION lib/cmake/${PROJECT_NAME}
)

# Install the export target
install(EXPORT      ${PROJECT_NAME}Targets
        FILE        ${PROJECT_NAME}Targets.cmake
        NAMESPACE   ${PROJECT_NAME}::
        DESTINATION lib/cmake/${PROJECT_NAME}
)
