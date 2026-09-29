add_executable(unit_smoke smoke_test.cpp)
target_link_libraries(unit_smoke PRIVATE GTest::gtest_main)
marv_apply_flags(unit_smoke)
gtest_discover_tests(unit_smoke)

add_executable(unit_prim prim/vec_mat_test.cpp prim/quat_test.cpp)
target_link_libraries(unit_prim PRIVATE marv_prim GTest::gtest_main)
marv_apply_flags(unit_prim)
gtest_discover_tests(unit_prim)

add_executable(unit_params params/param_defaults_test.cpp)
target_link_libraries(unit_params PRIVATE marv_params_generated GTest::gtest_main)
marv_apply_flags(unit_params)
gtest_discover_tests(unit_params)

add_executable(unit_params_runtime params/param_runtime_test.cpp)
target_link_libraries(unit_params_runtime PRIVATE marv_params_runtime marv_hal_sim GTest::gtest_main)
marv_apply_flags(unit_params_runtime)
gtest_discover_tests(unit_params_runtime)

add_executable(unit_params_runtime_uninit params/param_runtime_uninit_test.cpp)
target_link_libraries(unit_params_runtime_uninit PRIVATE marv_params_runtime marv_hal_sim GTest::gtest_main)
marv_apply_flags(unit_params_runtime_uninit)
gtest_discover_tests(unit_params_runtime_uninit)

add_executable(unit_types types/dshot_test.cpp types/layout_test.cpp)
target_link_libraries(unit_types PRIVATE marv_types GTest::gtest_main)
marv_apply_flags(unit_types)
gtest_discover_tests(unit_types)

add_executable(unit_hal_sim hal_sim/clock_test.cpp hal_sim/actuator_latch_test.cpp)
target_link_libraries(unit_hal_sim PRIVATE marv_hal_sim GTest::gtest_main)
marv_apply_flags(unit_hal_sim)
gtest_discover_tests(unit_hal_sim)

add_executable(unit_sched sched/rate_groups_test.cpp)
target_link_libraries(unit_sched PRIVATE marv_sched marv_hal_sim GTest::gtest_main)
marv_apply_flags(unit_sched)
gtest_discover_tests(unit_sched)
