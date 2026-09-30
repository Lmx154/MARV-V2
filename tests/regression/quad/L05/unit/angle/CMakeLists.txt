# The L5 angle-mode setpoint generator, T1 (decision 0006 D): tilt, deadband, yaw-rate mode, the braking lock and its
# guards (a) and (b), re-arm, initialisation and faults.
add_executable(unit_l5_angle angle_test.cpp)
target_link_libraries(unit_l5_angle PRIVATE marv_attitude marv_types marv_hal_sim GTest::gtest_main)
marv_apply_flags(unit_l5_angle)
marv_frozen_suite(unit_l5_angle)
