# Math/CMakeLists.txt

# Create a library target for Math
add_library(Math INTERFACE)

# Specify the include directories
target_include_directories(Math INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Optionally, install headers
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/Math/MathFunctions.h
              ${CMAKE_CURRENT_SOURCE_DIR}/include/Math/Polynomial.h
              ${CMAKE_CURRENT_SOURCE_DIR}/include/Math/QPSolver.h
              ${CMAKE_CURRENT_SOURCE_DIR}/include/Math/SkewSymmetric.h
              ${CMAKE_CURRENT_SOURCE_DIR}/include/Math/Spline.h
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/RobotLibrary/Math
)

