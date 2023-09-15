# Set the desired runtime path relative to the executable location
cmake_policy(SET CMP0095 NEW)
SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib:\${ORIGIN}/../lib:${CMAKE_SOURCE_DIR}/lib")

# Include gnuinstalldir to get the platform's standard directories:
include(GNUInstallDirs)

add_subdirectory(user_interface)
add_subdirectory(geometry)
add_subdirectory(collision_detection)
add_subdirectory(convex_decomposition)
add_subdirectory(heuristics)
add_subdirectory(path_planning)
add_subdirectory(robot_prototype)
add_subdirectory(test)


if(EXPORT_CMAKE_PACKAGE)
  include(ExportPackage)
endif()
