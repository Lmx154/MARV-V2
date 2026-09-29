add_executable(unit_smoke smoke_test.cpp)
target_link_libraries(unit_smoke PRIVATE GTest::gtest_main)
marv_apply_flags(unit_smoke)
gtest_discover_tests(unit_smoke)

add_executable(unit_prim prim/vec_mat_test.cpp prim/quat_test.cpp)
target_link_libraries(unit_prim PRIVATE marv_prim GTest::gtest_main)
marv_apply_flags(unit_prim)
gtest_discover_tests(unit_prim)
