# heuristics_lib
add_library(genetic_algo SHARED
  genetic_algo.cpp
)
include_directories(${PROJECT_SOURCE_DIR}/include)
# Usage requirement of lib
target_include_directories(genetic_algo
  INTERFACE ${PROJECT_SOURCE_DIR}/include
)
# Specify the dependencies for a lib
# target_link_libraries(genetic_algo -lpthread)
# Install include files
file(GLOB_RECURSE heuristics_headers
  ${PROJECT_SOURCE_DIR}/include/heuristics/*.h
)
install(FILES ${heuristics_headers} DESTINATION 
  ${PROJECT_BINARY_DIR}/out/include/heuristics
)

# Install all lib in the package
set(heuristics_libs genetic_algo)
install(TARGETS ${heuristics_libs} LIBRARY DESTINATION 
  ${PROJECT_BINARY_DIR}/out/lib/heuristics
)
