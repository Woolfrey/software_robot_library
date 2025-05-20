
# List the source files for this library
add_library(Control src/DifferentialDriveFeedback.cpp
                    src/DifferentialDrivePredictive.cpp
                    src/SerialDynamicControl.cpp
                    src/SerialKinematicControl.cpp
                    src/SerialLinkBase.cpp
)

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

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/                                              # The directory we want to install
        DESTINATION include/RobotLibrary/                                                           # Where to install it
        FILES_MATCHING PATTERN "*.h"
)
