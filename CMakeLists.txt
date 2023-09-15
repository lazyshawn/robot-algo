cmake_minimum_required(VERSION 3.15)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/cmake")
if(PROJECT_IS_TOP_LEVEL)
  include(TopCmakeConfig)
else()
  set(EXPORT_CMAKE_PACKAGE OFF)
endif()

project(RobotAlgo VERSION 1.0.0)

# Set deault build type
if (NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "" FORCE)
endif()
# Set the installation directory
SET(CMAKE_INSTALL_PREFIX "${PROJECT_BINARY_DIR}/out/${PROJECT_NAME}-${CMAKE_BUILD_TYPE}")

option(ENABLE_TEST "Enable GoogleTest" ON)
add_compile_definitions(PRINT_DEBUG_LOG)

if(ENABLE_TEST)
  include(CTest)
  find_package(GTest REQUIRED)
  include(GoogleTest)
endif()

add_subdirectory(src)

