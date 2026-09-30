# The l2_open_loop composition through the SIL C ABI. marv_sil_init succeeds once per process, so every test that
# initialises runs in a forked child (open_loop_test.cpp).
add_executable(unit_l2_open_loop open_loop_test.cpp)
target_link_libraries(unit_l2_open_loop PRIVATE
  marv_sil_l2_open_loop marv_types marv_params_l2_open_loop_generated GTest::gtest_main)
marv_apply_flags(unit_l2_open_loop)
marv_frozen_suite(unit_l2_open_loop)
