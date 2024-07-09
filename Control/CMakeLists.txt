add_library(Control src/SerialLinkBase.cpp
                    src/SerialKinematicControl.cpp
)

# Specify the include directories
target_include_directories(Control PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/Control>
                           $<INSTALL_INTERFACE:include/Control>
)

include_directories(${CMAKE_SOURCE_DIR}/Math/include)
include_directories(${CMAKE_SOURCE_DIR}/Model/include)                                              # This is so the Control library can find the Model library

# Link with the Model library
target_link_libraries(Control PUBLIC Math)                                                          # Need this or Control won't build
target_link_libraries(Control PUBLIC Model)                                                         # Need this or Control won't build

# Installation settings for Control
install(TARGETS  Control
        EXPORT   RobotLibraryTargets
        LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/Control/SerialLinkBase.h
              ${CMAKE_CURRENT_SOURCE_DIR}/include/Control/SerialKinematicControl.h
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/RobotLibrary/Control)
