# path_planning_lib
add_library(nurbs
  nurbs.cpp
)
set_target_properties(nurbs PROPERTIES
  INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/include/path_planning"
)
# Usage requirement of lib
target_include_directories(nurbs
  INTERFACE ${PROJECT_SOURCE_DIR}/include/path_planning
)

add_library(prm
  probabilistic_roadmap.cpp
)
set_target_properties(prm PROPERTIES
  INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/include/path_planning"
)
# Usage requirement of lib
target_include_directories(prm
  INTERFACE ${PROJECT_SOURCE_DIR}/include/path_planning
)
target_link_libraries(prm PUBLIC
  user_interface
)

# Install include files
file(GLOB_RECURSE path_planning_headers
  ${PROJECT_SOURCE_DIR}/include/path_planning/*.h
)
install(FILES ${path_planning_headers} DESTINATION 
  include/path_planning
)

# Install all lib in the package
set(path_planning_libs nurbs prm)
install(TARGETS ${path_planning_libs}
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
)
