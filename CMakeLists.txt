cmake_minimum_required(VERSION 3.15)
project(RobotAlgo VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
# Support degug
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
# SET(CMAKE_CXX_FLAGS_RELEASE {CXXFLAGS} -O3 -Wall)
SET(CMAKE_BUILD_TYPE "Debug")

if(WIN32)
  # create symbol for exporting lib for dynmic library
  SET(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
  # control where the static and shared libraries are built so that on windows
  # we don't need to tinker with the path to run the executable
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
  # Build static in windows
  option(BUILD_SHARED_LIBS "Build using shared libraries" OFF)
  # Set the runtime lib for MSCV (global)
  set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
else()
  option(BUILD_SHARED_LIBS "Build using shared libraries" ON)
endif()

# Set the installation directory
SET(CMAKE_INSTALL_PREFIX "${PROJECT_BINARY_DIR}/out/${PROJECT_NAME}-${CMAKE_BUILD_TYPE}")

# include_directories(include)

add_subdirectory(src)

