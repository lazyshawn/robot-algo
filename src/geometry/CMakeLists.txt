# geometry_lib
include_directories(${PROJECT_SOURCE_DIR}/include/geometry/)
# Install include files
file(GLOB_RECURSE geometry_headers
  ${PROJECT_SOURCE_DIR}/include/geometry/*.h
)
install(FILES ${geometry_headers} DESTINATION
  ${PROJECT_BINARY_DIR}/out/include/geometry
)

add_library(algebra SHARED
  algebra.cpp
)
# Usage requirement of lib
target_include_directories(algebra
  INTERFACE ${PROJECT_SOURCE_DIR}/include/geometry/
)

add_library(kd_tree SHARED
  kd_tree.cpp
)
# Usage requirement of lib
target_include_directories(kd_tree
  INTERFACE ${PROJECT_SOURCE_DIR}/include/geometry/
)

# Install all lib in the package
set(geometry_libs
  algebra
  kd_tree
)
install(TARGETS ${geometry_libs} LIBRARY DESTINATION
  ${PROJECT_BINARY_DIR}/out/lib/geometry
)
