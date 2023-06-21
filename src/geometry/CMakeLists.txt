# geometry_lib
add_library(algebra
  algebra.cpp
)
set_target_properties(algebra PROPERTIES
  INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/include/geometry"
)
# Usage requirement of lib
target_include_directories(algebra
  INTERFACE ${PROJECT_SOURCE_DIR}/include/geometry
)

add_library(kd_tree
  kd_tree.cpp
)
set_target_properties(kd_tree PROPERTIES
  INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/include/geometry"
)
target_link_libraries(kd_tree PUBLIC
  algebra
)
# Usage requirement of lib
target_include_directories(kd_tree
  INTERFACE ${PROJECT_SOURCE_DIR}/include/geometry
)

# Install include files
file(GLOB_RECURSE geometry_headers
  ${PROJECT_SOURCE_DIR}/include/geometry/*.h
)
install(FILES ${geometry_headers} DESTINATION
  include/geometry
)

# Install all lib in the package
set(geometry_libs
  algebra
  kd_tree
)
install(TARGETS ${geometry_libs}
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
)
