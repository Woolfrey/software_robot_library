# Model/CMakeLists.txt

# Create a library target for Model
add_library(Model INTERFACE)

# Specify the include directories
target_include_directories(Model INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Optionally, install headers
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/Model/Joint.h
              ${CMAKE_CURRENT_SOURCE_DIR}/include/Model/KinematicTree.h
              ${CMAKE_CURRENT_SOURCE_DIR}/include/Model/Link.h
              ${CMAKE_CURRENT_SOURCE_DIR}/include/Model/Pose.h
              ${CMAKE_CURRENT_SOURCE_DIR}/include/Model/RigidBody.h
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/RobotLibrary/Model
)

