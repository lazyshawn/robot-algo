# robot_prototype_lib
add_library(robot_prototype
  src/robot_base.cpp
  src/kinematics.cpp
)

# set_target_properties(user_interface PROPERTIES
#   INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/include"
#   LINK_DIRECTORIES "${CMAKE_SOURCE_DIR}/lib"
# )

# Then, link libraries
target_link_libraries(robot_prototype
  PUBLIC user_interface transform
)

# Then, carefully add your include directories. All of your `target_include_directories` must look like this
target_include_directories(robot_prototype PUBLIC
  # include directory in your build tree
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  # include directory when installed
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

###############################################################################
# Install package
###############################################################################
# Install public include files
install(DIRECTORY include/robot_prototype
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

# We add `yourlibrary` target into the export set.
# The export set will contain all targets to be imported by the other project.
# It also installs the library to the install script so they are installed:
set(robot_prototype_libs robot_prototype)
install(TARGETS ${robot_prototype_libs} EXPORT RobotAlgoTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)


###############################################################################
# Build test
###############################################################################
if(ENABLE_TEST)
  # kinematics
  add_executable(test_kinematics
    test/test_kinematics.cpp
  )
  target_link_libraries(test_kinematics PUBLIC
    GTest::gtest GTest::gtest_main
    robot_prototype
  )
  gtest_discover_tests(test_kinematics WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})
endif()

###############################################################################
# Build example
###############################################################################
add_executable(robotic_system
  ./example/robotic_system_base.cpp
)
target_include_directories(robotic_system PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
