# geometry_lib
add_library(algebra SHARED
  algebra.cpp
)
include_directories(${PROJECT_SOURCE_DIR}/include)
# Usage requirement of lib
target_include_directories(algebra
  INTERFACE ${PROJECT_SOURCE_DIR}/include
)
# Install include files
file(GLOB_RECURSE geometry_headers
  ${PROJECT_SOURCE_DIR}/include/geometry/*.h
)
install(FILES ${geometry_headers} DESTINATION
  ${PROJECT_BINARY_DIR}/out/include/geometry
)

# Install all lib in the package
set(geometry_libs algebra)
install(TARGETS ${geometry_libs} LIBRARY DESTINATION
  ${PROJECT_BINARY_DIR}/out/lib/geometry
)
