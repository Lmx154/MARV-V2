# L6 stage (d), packet P1 (decision 0017), T1: DShot error diffusion (marv::mixer::DshotDiffuser), not live. The carry
# bound, the window-mean bound with its derived float term, saturation, each reset path, thrust_to_dshot's bit identity
# after the d* refactor, and the four negative controls.
add_executable(unit_l6_dshot_diffusion diffusion_test.cpp)
target_link_libraries(unit_l6_dshot_diffusion PRIVATE
  marv_mixer marv_types marv_prim marv_params_runtime marv_hal_sim GTest::gtest_main)
marv_apply_flags(unit_l6_dshot_diffusion)
marv_frozen_suite(unit_l6_dshot_diffusion)
