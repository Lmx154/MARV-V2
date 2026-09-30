# The L3 mixer against the product parameter set (the vehicle card's M = B^-1). params_init succeeds once per
# process, so every test runs against the same initialised set.
add_executable(unit_l3_mixer mixer_test.cpp)
target_link_libraries(unit_l3_mixer PRIVATE
  marv_mixer marv_types marv_params_runtime marv_hal_sim GTest::gtest_main)
marv_apply_flags(unit_l3_mixer)
marv_frozen_suite(unit_l3_mixer)
