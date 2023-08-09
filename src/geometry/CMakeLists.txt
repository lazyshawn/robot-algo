# geometry_lib
add_library(algebra
  src/algebra.cpp
)

# set_target_properties(user_interface PROPERTIES
#   INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/include"
#   LINK_DIRECTORIES "${CMAKE_SOURCE_DIR}/lib"
# )

# Then, link libraries
# target_link_libraries(yourlibrary PUBLIC Qt6::Core Qt6::Widgets)

# Then, carefully add your include directories. All of your `target_include_directories` must look like this
target_include_directories(algebra PUBLIC
  # include directory in your build tree
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  # include directory when installed
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

###############################################################################
add_library(kd_tree
  src/kd_tree.cpp
)

# set_target_properties(user_interface PROPERTIES
#   INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/include"
#   LINK_DIRECTORIES "${CMAKE_SOURCE_DIR}/lib"
# )

# Then, link libraries
target_link_libraries(kd_tree PUBLIC algebra)

# Then, carefully add your include directories. All of your `target_include_directories` must look like this
target_include_directories(kd_tree PUBLIC
  # include directory in your build tree
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  # include directory when installed
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

###############################################################################
add_library(geometry
  src/geometry.cpp
)

# set_target_properties(user_interface PROPERTIES
#   INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/include"
#   LINK_DIRECTORIES "${CMAKE_SOURCE_DIR}/lib"
# )

# Then, link libraries
# target_link_libraries(geometry PUBLIC algebra)

# Then, carefully add your include directories. All of your `target_include_directories` must look like this
target_include_directories(geometry PUBLIC
  # include directory in your build tree
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  # include directory when installed
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

###############################################################################
add_library(transformation
  src/transformation.cpp
)

# Then, link libraries
# target_link_libraries(geometry PUBLIC algebra)

# Then, carefully add your include directories. All of your `target_include_directories` must look like this
target_include_directories(transformation PUBLIC
  # include directory in your build tree
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  # include directory when installed
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)




###############################################################################
# Install package
###############################################################################
# Install public include files
install(DIRECTORY include/geometry
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

# We add `yourlibrary` target into the export set.
# The export set will contain all targets to be imported by the other project.
# It also installs the library to the install script so they are installed:
set(geometry_libs
  algebra
  kd_tree
  geometry
  transformation
)
install(TARGETS ${geometry_libs} EXPORT RobotAlgoTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
