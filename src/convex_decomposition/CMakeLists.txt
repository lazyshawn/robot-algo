# convex_decomposition_lib
add_library(quickhull
  quickhull.cpp
)
set_target_properties(quickhull PROPERTIES
  INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/include/convex_decomposition"
)
# Specify the dependencies for a lib
target_link_libraries(quickhull algebra)
# Usage requirement of lib
target_include_directories(quickhull
  INTERFACE ${PROJECT_SOURCE_DIR}/include/convex_decomposition
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
install(TARGETS ${convex_decomposition_libs}
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)
