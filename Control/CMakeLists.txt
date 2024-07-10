# Control/CMakeLists.txt
add_library(Control INTERFACE)

target_include_directories(Control
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Optionally, if Control requires specific C++ features or flags:
# target_compile_features(Control INTERFACE cxx_std_11)
# target_compile_options(Control INTERFACE -Wall -Wextra)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
    DESTINATION include/RobotLibrary
    FILES_MATCHING PATTERN "*.h"
)

