add_executable(test_algebra
  test_algebra.cpp
)
target_link_libraries(test_algebra PUBLIC
  algebra
)

add_executable(test_gjk
  test_gjk.cpp
)
target_link_libraries(test_gjk PUBLIC
  gjk
)

add_executable(test_qkhull
  test_qkhull.cpp
)
target_link_libraries(test_qkhull PUBLIC
  quickhull
)

add_executable(test_ga
  test_ga.cpp
)
target_link_libraries(test_ga PUBLIC
  genetic_algo
  user_interface
)

add_executable(test_nurbs
  test_nurbs.cpp
)
target_link_libraries(test_nurbs PUBLIC
  nurbs
  user_interface
)

# set(test_bins test_gjk test_qkhull test_algebra)
# install(TARGETS ${test_bins} RUNTIME DESTINATION
#   ${PROJECT_BINARY_DIR}
# )

