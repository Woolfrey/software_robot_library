# Trajectory/CMakeLists.txt

set(TRAJECTORY_SOURCES src/SplineTrajectory.cpp                                                     # Shared: no dependence on Pose or Pose2D
                       src/TrajectoryBase.cpp
                       src/TrapezoidalVelocity.cpp)

if(BUILD_SERIAL_LINK)
    list(APPEND TRAJECTORY_SOURCES src/CartesianSpline.cpp)
endif()

if(BUILD_MOBILE_ROBOT)
    list(APPEND TRAJECTORY_SOURCES src/HermiteTrajectory.cpp
                                   src/MinimumArcLength.cpp)
endif()

add_library(Trajectory ${TRAJECTORY_SOURCES})

target_include_directories(Trajectory PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/RobotLibrary>
)

target_link_libraries(Trajectory PUBLIC Math Model Eigen3::Eigen)

install(TARGETS  Trajectory
        EXPORT   TrajectoryTargets
        LIBRARY  DESTINATION lib
        ARCHIVE  DESTINATION lib
        RUNTIME  DESTINATION bin
        INCLUDES DESTINATION include
)

# Only install headers for the parts that were built
set(TRAJECTORY_HEADER_EXCLUDES "")

if(NOT BUILD_SERIAL_LINK)
    list(APPEND TRAJECTORY_HEADER_EXCLUDES PATTERN "CartesianSpline.h"   EXCLUDE)
endif()

if(NOT BUILD_MOBILE_ROBOT)
    list(APPEND TRAJECTORY_HEADER_EXCLUDES PATTERN "HermiteTrajectory.h" EXCLUDE
                                           PATTERN "MinimumArcLength.h"  EXCLUDE)
endif()

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
        DESTINATION include/RobotLibrary
        FILES_MATCHING PATTERN "*.h"
        ${TRAJECTORY_HEADER_EXCLUDES}
)
