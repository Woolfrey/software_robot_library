add_library(Trajectory src/CartesianSpline.cpp
                       src/SplineTrajectory.cpp
                       src/TrapezoidalVelocity.cpp
)

# Specify the include directories
target_include_directories(Trajectory PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/Trajectory>
                           $<INSTALL_INTERFACE:include/Trajectory>
)

include_directories(${CMAKE_SOURCE_DIR}/Math/include)                                               # This is so the Trajectory library can find the Math library
include_directories(${CMAKE_SOURCE_DIR}/Model/include)                                              # This is so the Trajectory library can find the Model library

# Installation settings for Trajectory
install(TARGETS  Trajectory
        EXPORT   RobotLibraryTargets
        LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/Trajectory/CartesianSpline.h
              ${CMAKE_CURRENT_SOURCE_DIR}/include/Trajectory/SplineTrajectory.h
              ${CMAKE_CURRENT_SOURCE_DIR}/include/Trajectory/TrapezoidalVelocity.h
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/RobotLibrary/Trajectory)
       
