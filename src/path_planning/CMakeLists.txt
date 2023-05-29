# path_planning_lib
add_library(nurbs SHARED
  nurbs.cpp
)
include_directories(${PROJECT_SOURCE_DIR}/include/path_planning)
# Usage requirement of lib
target_include_directories(nurbs
  INTERFACE ${PROJECT_SOURCE_DIR}/include/path_planning
)
# Specify the dependencies for a lib
# target_link_libraries(nurbs -lpthread)
# Install include files
file(GLOB_RECURSE path_planning_headers
  ${PROJECT_SOURCE_DIR}/include/path_planning/*.h
)
install(FILES ${path_planning_headers} DESTINATION 
  ${PROJECT_BINARY_DIR}/out/include/path_planning
)

# Install all lib in the package
set(path_planning_libs nurbs)
install(TARGETS ${path_planning_libs} LIBRARY DESTINATION 
  ${PROJECT_BINARY_DIR}/out/lib/path_planning
)
