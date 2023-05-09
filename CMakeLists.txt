cmake_minimum_required(VERSION 2.8.12)
project(robot_algo)

set(CMAKE_CXX_STANDARD 17)
# Support degug
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE {CXXFLAGS} -O3 -Wall)

include_directories(include)
link_directories(lib)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/out/lib)

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

add_executable(robot_algo src/test/test_gjk.cpp)
# add_executable(robot_algo src/main.cpp)
target_link_libraries(robot_algo
  collistion_detection
)

install(TARGETS robot_algo
    RUNTIME DESTINATION ${PROJECT_BINARY_DIR}/bin
    LIBRARY DESTINATION ${PROJECT_BINARY_DIR}/lib
    ARCHIVE DESTINATION ${PROJECT_BINARY_DIR}/lib)

