add_subdirectory(geometry)
add_subdirectory(collision_detection)
add_subdirectory(convex_decomposition)
add_subdirectory(test)

# add_executable(test_algebra
#   test/test_algebra.cpp
# )
# target_link_libraries(test_algebra PUBLIC
#   algebra
# )
#
# add_executable(test_gjk
#   test/test_gjk.cpp
# )
# target_link_libraries(test_gjk PUBLIC
#   gjk
# )
#
# add_executable(test_qkhull
#   test/test_qkhull.cpp
# )
# target_link_libraries(test_qkhull PUBLIC
#   quickhull
# )
#
# set(test_bins
#   test_gjk
#   test_qkhull
#   test_algebra
# )
# install(TARGETS ${test_bins} RUNTIME DESTINATION
#   ${PROJECT_BINARY_DIR}/bin
# )

