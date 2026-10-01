# L6 stage (b), the gyro vibration model of the plant (decision 0013): the rotor angle against the closed-form integral
# of the motor model, the vibration known answer and its freshness, bit-identical step outputs, the off / A = 0 bytes,
# seeded determinism, saturation, refusals, and a negative control for each metric check.
# The shared scenario fixtures are the frozen L1 plant fixture and the L6 IMU test support (labelled scenario values).
add_executable(unit_l6_vibration theta_test.cpp vibration_test.cpp)
target_include_directories(unit_l6_vibration PRIVATE
  "${PROJECT_SOURCE_DIR}/tests/regression/quad/L01/unit/plant"
  "${PROJECT_SOURCE_DIR}/tests/regression/quad/L06/imu")
target_link_libraries(unit_l6_vibration PRIVATE
  marv_plant marv_plant_noise marv_musl_math marv_prim marv_types GTest::gtest_main)
marv_apply_flags(unit_l6_vibration)
marv_frozen_suite(unit_l6_vibration)
