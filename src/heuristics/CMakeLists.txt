# heuristics_lib
add_library(genetic_algo
  genetic_algo.cpp
)
set_target_properties(genetic_algo PROPERTIES
  INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/include/heuristics"
)
# Usage requirement of lib
target_include_directories(genetic_algo
  INTERFACE ${PROJECT_SOURCE_DIR}/include/heuristics
)

# Install include files
file(GLOB_RECURSE heuristics_headers
  ${PROJECT_SOURCE_DIR}/include/heuristics/*.h
)
install(FILES ${heuristics_headers} DESTINATION 
  ${PROJECT_BINARY_DIR}/out/include/heuristics
)

# Install all lib in the package
set(heuristics_libs genetic_algo)
install(TARGETS ${heuristics_libs}
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)
