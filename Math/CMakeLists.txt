add_library(Math src/MathFunctions.cpp
                 src/Polynomial.cpp
                 src/SkewSymmetric.cpp
                 src/Spline.cpp
)

# Specify the include directories
target_include_directories(Math PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/Math>
                           $<INSTALL_INTERFACE:include/Math>
)

# Installation settings for Math
install(TARGETS  Math
        EXPORT   RobotLibraryTargets
        LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/Math/MathFunctions.h
              ${CMAKE_CURRENT_SOURCE_DIR}/include/Math/Polynomial.h
              ${CMAKE_CURRENT_SOURCE_DIR}/include/Math/SkewSymmetric.h
              ${CMAKE_CURRENT_SOURCE_DIR}/include/Math/Spline.h    
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/RobotLibrary/Math)
       
