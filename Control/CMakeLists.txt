
# List the source files for this library

set(CONTROL_SOURCES "")

if(BUILD_SERIAL_LINK)
    list(APPEND CONTROL_SOURCES src/SerialLinkBase.cpp
                                src/SerialLinkImpedance.cpp
                                src/SerialLinkDynamic.cpp
                                src/SerialLinkKinematic.cpp)
endif()

if(BUILD_MOBILE_ROBOT)
    list(APPEND CONTROL_SOURCES src/UnicycleBase.cpp
                                src/UnicycleFeedback.cpp
                                src/UnicyclePredictive.cpp)
endif()

add_library(Control ${CONTROL_SOURCES})

# Specify targets to be built
target_include_directories(Control PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/RobotLibrary>
)

target_link_libraries(Control PUBLIC Math Model Eigen3::Eigen)                                      # Other libraries needed to compile this one

# Installation instructions
install(TARGETS  Control
        EXPORT   ControlTargets
        LIBRARY  DESTINATION lib
        ARCHIVE  DESTINATION lib
        RUNTIME  DESTINATION bin
        INCLUDES DESTINATION include
)

# Only install headers for the parts that were built
set(CONTROL_HEADER_EXCLUDES "")

if(NOT BUILD_SERIAL_LINK)
    list(APPEND CONTROL_HEADER_EXCLUDES PATTERN "SerialLink*" EXCLUDE)
endif()

if(NOT BUILD_MOBILE_ROBOT)
    list(APPEND CONTROL_HEADER_EXCLUDES PATTERN "Unicycle*" EXCLUDE)
endif()

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/                                              # The directory we want to install
        DESTINATION include/RobotLibrary/                                                           # Where to install it
        FILES_MATCHING PATTERN "*.h"
        ${CONTROL_HEADER_EXCLUDES}
)
