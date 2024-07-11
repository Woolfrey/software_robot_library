
# List the source files for this library
add_library(Model src/Joint.cpp
                  src/KinematicTree.cpp
                  src/Link.cpp
                  src/Pose.cpp
                  src/RigidBody.cpp
                  src/tinyxml2.cpp
)

# Specify targets to be built
target_include_directories(Model PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(Model PRIVATE Math Eigen3::Eigen)                                             # Other libraries needed to compile this one

# Installation instructions
install(TARGETS  Model
        EXPORT   ModelTargets
        LIBRARY  DESTINATION lib
        ARCHIVE  DESTINATION lib
        RUNTIME  DESTINATION bin
        INCLUDES DESTINATION include
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/                                              # The directory we want to install
        DESTINATION include/RobotLibrary                                                      # Where to install it
        FILES_MATCHING PATTERN "*.h"
)
