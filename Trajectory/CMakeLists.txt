# Trajectory/CMakeLists.txt
add_library(Trajectory src/CartesianSpline.cpp
                       src/SplineTrajectory.cpp
                       src/TrajectoryBase.cpp
                       src/TrapezoidalVelocity.cpp
)

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

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
        DESTINATION include/RobotLibrary
        FILES_MATCHING PATTERN "*.h"
)
