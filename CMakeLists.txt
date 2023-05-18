cmake_minimum_required(VERSION 2.8.12)
project(robot_algo)

set(CMAKE_CXX_STANDARD 17)
# Support degug
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE {CXXFLAGS} -O3 -Wall)

# RPATH settings for install
# https://stackoverflow.com/questions/22206122
# https://gitlab.kitware.com/cmake/community/-/wikis/doc/cmake/RPATH-handling
# list( APPEND CMAKE_INSTALL_RPATH ${PROJECT_BINARY_DIR}/out/lib/geometry )

# include_directories(include)
# 3rd party lib
link_directories(lib)

add_subdirectory(src)
