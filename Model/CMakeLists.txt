add_library(Model src/Pose.cpp
)

# Specify the include directories
target_include_directories(Model PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/Model>
                           $<INSTALL_INTERFACE:include/Model>
)

# Installation settings for Math
install(TARGETS  Model
        EXPORT   RobotLibraryTargets
        LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/Model/Pose.h    
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/RobotLibrary/Model)
       
