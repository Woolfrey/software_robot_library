find_package(Eigen3 REQUIRED)

include_directories(include SHARED ${EIGEN3_INCLUDE_DIRS})

add_library(Control SHARED src/SerialKinControl.cpp src/SerialDynControl.cpp)

target_include_directories(Control
    PUBLIC 
        $<INSTALL_INTERFACE:include>    
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(Control Model)