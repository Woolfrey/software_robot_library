####################################################################################################
#                                              MODEL                                               #
####################################################################################################
project(Model)                                                                                      # Refered to by ${PROJECT_NAME} below

# Generate target reference,
# list associated headers containing class definitions
add_library(${PROJECT_NAME}
            include/Joint.h
            include/Link.h
            include/KinematicTree.h
            include/RigidBody.h
)

set_target_properties(${PROJECT_NAME} PROPERTIES CXX_VISIBILITY_PRESET "hidden"                     # Helps reduce size of library?
                                                 LINKER_LANGUAGE CXX                                # Need to tell the linker this is C++
                                                 VISIBILITY_INLINES_HIDDEN TRUE                     # Hide inline functions to reduce size of library
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)                                          # Define the C++ standard needed to compile this library

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src                  # Set source files as private
        	                                         ${CMAKE_CURRENT_SOURCE_DIR}/test                 # Set test files as private
                                           PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> # Reference the local include directory when building this project
                                                  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>       # Reference install/include directory after the fact
)

####################################################################################################

add_executable(urdf_test test/urdf_test.cpp src/tinyxml2.cpp)                                       # Create name of exe from source file
add_dependencies(urdf_test RobotLibrary)                                                            # Depends on RobotLibrary (obviously)
target_link_libraries(urdf_test RobotLibrary)                                                       # Link RobotLibrary
