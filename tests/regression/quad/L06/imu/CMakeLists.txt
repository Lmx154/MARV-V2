# L6 stage (a), the plant's generic IMU model (decision 0012): zero-noise known-answer tests (output order, rounding,
# clamps and flags, delay, turn-on bias, free fall), refusals that leave a twin plant bit-identical, seeded determinism,
# the noise formulas against an independent oracle (each with a control that must fail), and the SIL validity rules.
# The generated corner builder (imu_corner_config) is checked here too: 64 distinct corners, nominal separate.
# The shared scenario fixture is the frozen L1 plant fixture (every value in it is a labelled scenario value).
add_executable(unit_l6_imu kat_test.cpp refusal_test.cpp noise_test.cpp layout_test.cpp corner_test.cpp)
target_include_directories(unit_l6_imu PRIVATE
  "${PROJECT_SOURCE_DIR}/tests/regression/quad/L01/unit/plant"
  "${PROJECT_SOURCE_DIR}/fw/sil/include")
target_link_libraries(unit_l6_imu PRIVATE marv_imu_profile_config marv_plant marv_plant_noise marv_prim marv_types GTest::gtest_main)
marv_apply_flags(unit_l6_imu)
marv_frozen_suite(unit_l6_imu)
