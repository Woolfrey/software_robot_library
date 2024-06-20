####################################################################################################
#                                               MATH                                               #
####################################################################################################

project(Math)                                                                                       # Refered to by ${PROJECT_NAME} below

add_library(${PROJECT_NAME} include/Math/MathFunctions.h
                            include/Math/QPSolver.h)                                                # Generate targets and list associated header files

set_target_properties(${PROJECT_NAME} PROPERTIES CXX_VISIBILITY_PRESET "hidden"                     # Helps reduce size of library?
                                                 LINKER_LANGUAGE CXX                                # Need to tell the linker this is C++
                                                 VISIBILITY_INLINES_HIDDEN TRUE                     # Hide inline functions to reduce size of library
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)                                          # Define the C++ standard needed to compile this library

target_include_directories(${PROJECT_NAME} PRIVATE src                                                   # Set the source files as private
                                           PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> # Reference the local include directory when building this project
                                                  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>       # Reference install/include directory after the fact
)
