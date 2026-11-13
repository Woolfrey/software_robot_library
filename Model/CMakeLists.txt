
# List the source files for this library

set(MODEL_SOURCES "")

if(BUILD_SERIAL_LINK)
    list(APPEND MODEL_SOURCES src/Joint.cpp
                              src/KinematicTree.cpp
                              src/Link.cpp
                              src/Pose.cpp
                              src/RigidBody.cpp
                              src/tinyxml2.cpp)                                                     # URDF parsing for KinematicTree
endif()

if(BUILD_MOBILE_ROBOT)
    list(APPEND MODEL_SOURCES src/Obstacle2D.cpp
                              src/Pose2D.cpp
                              src/RigidBody2D.cpp
                              src/Unicycle.cpp)
endif()

add_library(Model ${MODEL_SOURCES})

# Specify targets to be built
target_include_directories(Model PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/RobotLibrary>
)

target_link_libraries(Model PUBLIC Math Eigen3::Eigen)                                              # Other libraries needed to compile this one

# Installation instructions
install(TARGETS  Model
        EXPORT   ModelTargets
        LIBRARY  DESTINATION lib
        ARCHIVE  DESTINATION lib
        RUNTIME  DESTINATION bin
        INCLUDES DESTINATION include
)

# Only install headers for the parts that were built
set(MODEL_HEADER_EXCLUDES "")

if(NOT BUILD_SERIAL_LINK)
    list(APPEND MODEL_HEADER_EXCLUDES PATTERN "Joint.h"         EXCLUDE
                                      PATTERN "KinematicTree.h" EXCLUDE
                                      PATTERN "Link.h"          EXCLUDE
                                      PATTERN "Pose.h"          EXCLUDE
                                      PATTERN "RigidBody.h"     EXCLUDE
                                      PATTERN "tinyxml2.h"      EXCLUDE)
endif()

if(NOT BUILD_MOBILE_ROBOT)
    list(APPEND MODEL_HEADER_EXCLUDES PATTERN "Obstacle2D.h"    EXCLUDE
                                      PATTERN "Pose2D.h"        EXCLUDE
                                      PATTERN "RigidBody2D.h"   EXCLUDE
                                      PATTERN "Unicycle.h"      EXCLUDE)
endif()

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/                                              # The directory we want to install
        DESTINATION include/RobotLibrary                                                            # Where to install it
        FILES_MATCHING PATTERN "*.h"
        ${MODEL_HEADER_EXCLUDES}
)
