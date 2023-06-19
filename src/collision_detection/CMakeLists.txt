# collision_detection_lib
add_library(gjk
  gjk.cpp
)
set_target_properties(gjk PROPERTIES
  INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/include/collision_detection"
)
# Usage requirement of lib
target_include_directories(gjk
  INTERFACE ${PROJECT_SOURCE_DIR}/include/collision_detection
)
# Install include
file(GLOB_RECURSE COLLISION_DETECTION_HEADERS
  ${PROJECT_SOURCE_DIR}/include/collision_detection/*.h
)
install(FILES ${COLLISION_DETECTION_HEADERS} DESTINATION 
  ${PROJECT_BINARY_DIR}/out/include/collision_detection
)

# Install all lib in the package
set(collision_detection_libs gjk)
install(TARGETS ${collision_detection_libs}
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)
