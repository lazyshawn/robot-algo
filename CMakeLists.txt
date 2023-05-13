cmake_minimum_required(VERSION 2.8.12)
project(robot_algo)

set(CMAKE_CXX_STANDARD 17)
# Support degug
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE {CXXFLAGS} -O3 -Wall)

include_directories(include)
link_directories(lib)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/out/lib)

# convex_decomposition_lib
add_library(convex_decomposition SHARED
  src/convex_decomposition/quickhull.cpp
)
# Specify include directories for a lib
target_include_directories(convex_decomposition PUBLIC
  ${PROJECT_SOURCE_DIR}/include/convex_decomposition
)
file(GLOB_RECURSE COLLISION_DETECTION_HEADERS include/collision_detection/*.h)
install(FILES ${COLLISION_DETECTION_HEADERS} DESTINATION 
  ${PROJECT_BINARY_DIR}/out/include/collision_detection
)

# collistion_detection_lib
add_library(collistion_detection SHARED
  src/collision_detection/gjk.cpp
)
target_include_directories(collistion_detection PUBLIC
  ${PROJECT_SOURCE_DIR}/include/collision_detection
)
file(GLOB_RECURSE COLLISION_DETECTION_HEADERS include/collision_detection/*.h)
install(FILES ${COLLISION_DETECTION_HEADERS} DESTINATION 
  ${PROJECT_BINARY_DIR}/out/include/collision_detection
)

# add_executable(robot_algo src/main.cpp)
add_executable(test_gjk src/test/test_gjk.cpp)
target_link_libraries(test_gjk
  collistion_detection
)
add_executable(test_qkhull src/test/test_qkhull.cpp)
target_link_libraries(test_qkhull
  convex_decomposition
)

install(TARGETS test_gjk
    RUNTIME DESTINATION ${PROJECT_BINARY_DIR}/bin
    LIBRARY DESTINATION ${PROJECT_BINARY_DIR}/lib
    ARCHIVE DESTINATION ${PROJECT_BINARY_DIR}/lib)

