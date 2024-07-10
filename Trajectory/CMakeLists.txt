# Trajectory/CMakeLists.txt

# Create a library target for Trajectory
add_library(Trajectory INTERFACE)

# Specify the include directories
target_include_directories(Trajectory INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Optionally, install headers
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/Trajectory/CartesianSpline.h
              ${CMAKE_CURRENT_SOURCE_DIR}/include/Trajectory/SplineTrajectory.h
              ${CMAKE_CURRENT_SOURCE_DIR}/include/Trajectory/TrajectoryBase.h
              ${CMAKE_CURRENT_SOURCE_DIR}/include/Trajectory/TrapezoidalVelocity.h
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/RobotLibrary/Trajectory
)

