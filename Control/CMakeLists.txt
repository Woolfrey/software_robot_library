# Control/CMakeLists.txt

# Create a library target for Control
add_library(Control INTERFACE)

# Specify the include directories
target_include_directories(Control INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Optionally, install headers
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/Control/SerialLinkBase.h
              ${CMAKE_CURRENT_SOURCE_DIR}/include/Control/SerialKinematicControl.h
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/RobotLibrary/Control
)
