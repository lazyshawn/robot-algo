# user_interface_lib
add_library(user_interface
  timer.cpp
  random.cpp
)
set_target_properties(user_interface PROPERTIES
  INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/include/user_interface"
)
# Usage requirement of lib
target_include_directories(user_interface
  INTERFACE ${PROJECT_SOURCE_DIR}/include/user_interface
)

# Install include files
file(GLOB_RECURSE user_interface_headers
  ${PROJECT_SOURCE_DIR}/include/user_interface/*.h
)
install(FILES ${user_interface_headers} DESTINATION 
  ${CMAKE_INSTALL_PREFIX}/include/user_interface
)

# Install all lib in the package
set(user_interface_libs user_interface)
install(TARGETS ${user_interface_libs}
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
)
