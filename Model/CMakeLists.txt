# Model/CMakeLists.txt
add_library(Model
    src/tinyxml2.cpp
)

target_include_directories(Model
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Optionally, if Model requires specific C++ features or flags:
# target_compile_features(Model PUBLIC cxx_std_11)
# target_compile_options(Model PRIVATE -Wall -Wextra)

install(TARGETS Model
    EXPORT ModelTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
    DESTINATION include/RobotLibrary
    FILES_MATCHING PATTERN "*.h"
)

