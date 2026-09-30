# The L4 rate loop unit suite. The mixer and effectiveness fixtures come from the product parameter set (the card).
add_executable(unit_l4_rate rate_test.cpp)
target_link_libraries(unit_l4_rate PRIVATE
  marv_rate marv_mixer marv_types marv_params_runtime marv_hal_sim GTest::gtest_main)
marv_apply_flags(unit_l4_rate)
marv_frozen_suite(unit_l4_rate)

# The configuration read from the product parameter set (rate::from_params, rate::load_config).
add_executable(unit_l4_rate_params params_test.cpp)
target_link_libraries(unit_l4_rate_params PRIVATE
  marv_rate marv_mixer marv_types marv_params_runtime marv_hal_sim GTest::gtest_main)
marv_apply_flags(unit_l4_rate_params)
marv_frozen_suite(unit_l4_rate_params)
