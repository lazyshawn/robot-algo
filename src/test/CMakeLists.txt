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
  user_interface
)

add_executable(test_qkhull
  test_qkhull.cpp
)
target_link_libraries(test_qkhull PUBLIC
  quickhull
  user_interface
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

add_executable(test_pca
  test_pca.cpp
)
target_link_libraries(test_pca PUBLIC
  algebra
  user_interface
)

add_executable(test_kdtree
  test_kdtree.cpp
)
target_link_libraries(test_kdtree PUBLIC
  kd_tree
  user_interface
)

add_executable(test_probabilistic_roadmap
  test_probabilistic_roadmap.cpp
)
target_link_libraries(test_probabilistic_roadmap PUBLIC
  prm
  user_interface
)

set(test_bins
  # test_gjk
  # test_qkhull
  # test_algebra
  # test_probabilistic_roadmap
)
install(TARGETS ${test_bins}
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
)
