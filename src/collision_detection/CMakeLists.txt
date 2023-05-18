# collistion_detection_lib
add_library(gjk SHARED
  gjk.cpp
)
include_directories(${PROJECT_SOURCE_DIR}/include)
# Usage requirement of lib
target_include_directories(gjk
  INTERFACE ${PROJECT_SOURCE_DIR}/include
)
# Install include
file(GLOB_RECURSE COLLISION_DETECTION_HEADERS
  ${PROJECT_SOURCE_DIR}/include/collision_detection/*.h
)
install(FILES ${COLLISION_DETECTION_HEADERS} DESTINATION 
  ${PROJECT_BINARY_DIR}/out/include/collision_detection
)

# Install all lib in the package
set(collistion_detection_libs gjk)
install(TARGETS ${collistion_detection_libs} LIBRARY DESTINATION 
  ${PROJECT_BINARY_DIR}/out/lib/collision_detection
)
