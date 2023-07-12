# convex_decomposition_lib
add_library(quickhull
  src/quickhull.cpp
)

# set_target_properties(user_interface PROPERTIES
#   INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/include"
#   LINK_DIRECTORIES "${CMAKE_SOURCE_DIR}/lib"
# )

# Then, link libraries
target_link_libraries(quickhull PUBLIC algebra geometry)

# Then, carefully add your include directories. All of your `target_include_directories` must look like this
target_include_directories(quickhull PUBLIC
  # include directory in your build tree
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  # include directory when installed
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)


###############################################################################
# Install package
###############################################################################
# Install public include files
install(DIRECTORY include/convex_decomposition
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

# We add `yourlibrary` target into the export set.
# The export set will contain all targets to be imported by the other project.
# It also installs the library to the install script so they are installed:
set(convex_decomposition_libs quickhull)
install(TARGETS ${convex_decomposition_libs} EXPORT RobotAlgoTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
