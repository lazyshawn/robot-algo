cmake_policy(SET CMP0095 NEW)
# Set the installation directory
SET(CMAKE_INSTALL_PREFIX "${PROJECT_BINARY_DIR}/out/${CMAKE_BUILD_TYPE}" CACHE INTERNAL "")
# Set the desired runtime path relative to the executable location
SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib:\${ORIGIN}/../lib:${CMAKE_SOURCE_DIR}/lib")

add_subdirectory(user_interface)
add_subdirectory(geometry)
add_subdirectory(collision_detection)
add_subdirectory(convex_decomposition)
add_subdirectory(heuristics)
add_subdirectory(path_planning)
add_subdirectory(test)

