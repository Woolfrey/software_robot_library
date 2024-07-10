# Math/CMakeLists.txt
add_library(Math INTERFACE)

target_include_directories(Math
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Optionally, if Math requires specific C++ features or flags:
# target_compile_features(Math INTERFACE cxx_std_11)
# target_compile_options(Math INTERFACE -Wall -Wextra)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
    DESTINATION include/RobotLibrary
    FILES_MATCHING PATTERN "*.h"
)

