# L6 stage (c), commit 1 (decision 0014), the inert firmware pieces, T1: the rate loop's D low-pass and the
# lag-compensated omega x J omega feed-forward (rate_loop_test.cpp), and the gyro chain's NaN guard and seeding
# (chain_guard_test.cpp). Each piece is off by default, so the tests also show the loop is today's loop bit for bit.
add_executable(unit_l6_d_term rate_loop_test.cpp chain_guard_test.cpp)
target_link_libraries(unit_l6_d_term PRIVATE
  marv_rate marv_mixer marv_gyro_chain marv_types marv_prim marv_params_runtime marv_hal_sim GTest::gtest_main)
marv_apply_flags(unit_l6_d_term)
marv_frozen_suite(unit_l6_d_term)
