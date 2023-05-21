# user_interface_lib
add_library(user_interface SHARED
  timer.cpp
  random.cpp
)
include_directories(${PROJECT_SOURCE_DIR}/include)
# Usage requirement of lib
target_include_directories(user_interface
  INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
)
# Specify the dependencies for a lib
# target_link_libraries(user_interface -lpthread)
# Install include files
file(GLOB_RECURSE user_interface_headers
  ${PROJECT_SOURCE_DIR}/include/user_interface/*.h
)
install(FILES ${user_interface_headers} DESTINATION 
  ${PROJECT_BINARY_DIR}/out/include/user_interface
)

# Install all lib in the package
set(user_interface_libs user_interface)
install(TARGETS ${user_interface_libs} LIBRARY DESTINATION 
  ${PROJECT_BINARY_DIR}/out/lib/user_interface
)
