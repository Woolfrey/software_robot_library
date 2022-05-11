# This program is used to test and debug code.
add_executable(robot_library_test robot_library_test.cpp)
target_link_libraries(robot_library_test Eigen3::Eigen)

install(TARGETS robot_library_test                                                                  # Specify the source file
  	# IMPORTANT: Add the bar executable to the "export-set"
  	EXPORT RobotLibraryTargets
  	RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin)

