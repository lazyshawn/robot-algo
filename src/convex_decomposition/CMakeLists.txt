# convex_decomposition_lib
add_library(quickhull SHARED
  quickhull.cpp
)
include_directories(${PROJECT_SOURCE_DIR}/include)
# Specify the dependencies for a lib
target_link_libraries(quickhull algebra user_interface)
# Usage requirement of lib
target_include_directories(quickhull
  INTERFACE ${PROJECT_SOURCE_DIR}/include
)
# Install include files
file(GLOB_RECURSE COLLISION_DETECTION_HEADERS 
  ${PROJECT_SOURCE_DIR}/include/convex_decomposition/*.h
)
install(FILES ${COLLISION_DETECTION_HEADERS} DESTINATION 
  ${PROJECT_BINARY_DIR}/out/include/convex_decomposition
)

# Install all lib in the package
set(convex_decomposition_libs quickhull)
install(TARGETS ${convex_decomposition_libs} LIBRARY DESTINATION 
  ${PROJECT_BINARY_DIR}/out/lib/convex_decomposition
)
