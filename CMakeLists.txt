cmake_minimum_required(VERSION 2.8.12)
project(robot_algo)

set(CMAKE_CXX_STANDARD 17)
# Support degug
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE {CXXFLAGS} -O3 -Wall)
SET(CMAKE_BUILD_TYPE "Debug")

option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

if(MSVC)
  # create symbol for exporting lib for dynmic library
  SET(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
  # control where the static and shared libraries are built so that on windows
  # we don't need to tinker with the path to run the executable
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
  # Build static in windows
  option(BUILD_SHARED_LIBS "Build using shared libraries" OFF)
endif()

# include_directories(include)
# 3rd party lib
link_directories(lib)

add_subdirectory(src)
