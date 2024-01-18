# path_planning_lib
add_library(nurbs
  src/nurbs.cpp
)

# set_target_properties(user_interface PROPERTIES
#   INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/include"
#   LINK_DIRECTORIES "${CMAKE_SOURCE_DIR}/lib"
# )

# Then, link libraries
# target_link_libraries(yourlibrary PUBLIC Qt6::Core Qt6::Widgets)

# Then, carefully add your include directories. All of your `target_include_directories` must look like this
target_include_directories(nurbs PUBLIC
  # include directory in your build tree
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  # include directory when installed
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

###############################################################################
add_library(prm
  src/probabilistic_roadmap.cpp
)

# set_target_properties(user_interface PROPERTIES
#   INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/include"
#   LINK_DIRECTORIES "${CMAKE_SOURCE_DIR}/lib"
# )

# Then, link libraries
target_link_libraries(prm PUBLIC user_interface)

# Then, carefully add your include directories. All of your `target_include_directories` must look like this
target_include_directories(prm PUBLIC
  # include directory in your build tree
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  # include directory when installed
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)


###############################################################################
add_library(curve_decomposition
  src/curve_decomposition.cpp
)

# set_target_properties(user_interface PROPERTIES
#   INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/include"
#   LINK_DIRECTORIES "${CMAKE_SOURCE_DIR}/lib"
# )

# Then, link libraries
# target_link_libraries(curve_decomposition PUBLIC )

# Then, carefully add your include directories. All of your `target_include_directories` must look like this
target_include_directories(curve_decomposition PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

###############################################################################
# Install package
###############################################################################
# Install public include files
install(DIRECTORY include/path_planning
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

# We add `yourlibrary` target into the export set.
# The export set will contain all targets to be imported by the other project.
# It also installs the library to the install script so they are installed:
set(path_planning_libs
  nurbs
  prm
  curve_decomposition
)
install(TARGETS ${path_planning_libs} EXPORT RobotAlgoTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

###############################################################################
# Build example
###############################################################################
add_executable(discrete_trajectory_decomposition
  ./example/discrete_trajectory_decomposition.cpp
)
target_include_directories(discrete_trajectory_decomposition PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(discrete_trajectory_decomposition PUBLIC curve_decomposition)
