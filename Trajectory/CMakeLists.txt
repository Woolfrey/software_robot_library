# Trajectory/CMakeLists.txt
add_library(Trajectory INTERFACE)

target_include_directories(Trajectory
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Optionally, if Trajectory requires specific C++ features or flags:
# target_compile_features(Trajectory INTERFACE cxx_std_11)
# target_compile_options(Trajectory INTERFACE -Wall -Wextra)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
    DESTINATION include/RobotLibrary
    FILES_MATCHING PATTERN "*.h"
)

