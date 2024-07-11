# Math/CMakeLists.txt
add_library(Math src/MathFunctions.cpp
                 src/Polynomial.cpp
                 src/SkewSymmetric.cpp
                 src/Spline.cpp
)

target_include_directories(Math PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(Math PRIVATE Eigen3::Eigen)

install(TARGETS  Math
        EXPORT   MathTargets
        LIBRARY  DESTINATION lib
        ARCHIVE  DESTINATION lib
        RUNTIME  DESTINATION bin
        INCLUDES DESTINATION include
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
        DESTINATION include/RobotLibrary
        FILES_MATCHING PATTERN "*.h"
)
