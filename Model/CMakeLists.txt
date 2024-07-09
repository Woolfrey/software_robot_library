add_library(Model src/Joint.cpp
                  src/KinematicTree.cpp
                  src/Link.cpp
                  src/Pose.cpp
                  src/RigidBody.cpp
)

# Specify the include directories
target_include_directories(Model PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/Model>
                           $<INSTALL_INTERFACE:include/Model>
)

include_directories(${CMAKE_SOURCE_DIR}/Math/include)                                               # This is so the Model library can find the Math library

# Installation settings for Model
install(TARGETS  Model
        EXPORT   RobotLibraryTargets
        LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/Model/Joint.h
              ${CMAKE_CURRENT_SOURCE_DIR}/include/Model/KinematicTree.h
              ${CMAKE_CURRENT_SOURCE_DIR}/include/Model/Link.h
              ${CMAKE_CURRENT_SOURCE_DIR}/include/Model/Pose.h
              ${CMAKE_CURRENT_SOURCE_DIR}/include/Model/RigidBody.h
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/RobotLibrary/Model)
       
