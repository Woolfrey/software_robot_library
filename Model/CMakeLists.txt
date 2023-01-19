find_package(Eigen3 REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_library(Model SHARED src/Joint.cpp src/RigidBody.cpp src/SerialLink.cpp src/Pose.cpp)

#target_include_directories (Model PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_include_directories(Model
    PUBLIC 
        $<INSTALL_INTERFACE:include>    
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)
