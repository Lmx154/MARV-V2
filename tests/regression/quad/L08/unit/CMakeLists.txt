# The CRSF frame decoder (fw/hal/target/decode, M2 P5a): frame parsing, CRC8 0xD5, the 11-bit channel unpack, resynchronisation
# and the error counters, on the host.
add_executable(unit_crsf crsf_test.cpp)
target_link_libraries(unit_crsf PRIVATE marv_crsf_decode GTest::gtest_main)
marv_apply_flags(unit_crsf)
marv_frozen_suite(unit_crsf)

# The QF-1 rates curve (fw/stick_rates, M2 P2): known answers, symmetry, monotonicity, centre slope, validation.
add_executable(unit_stick_rates stick_rates_test.cpp)
target_link_libraries(unit_stick_rates PRIVATE marv_stick_rates GTest::gtest_main)
marv_apply_flags(unit_stick_rates)
marv_frozen_suite(unit_stick_rates)

# The manual-control pull and its SIL staging (M2 P3, decision 0040 Q3).
add_executable(unit_manual_control manual_control_test.cpp)
target_link_libraries(unit_manual_control PRIVATE marv_hal_sim GTest::gtest_main)
marv_apply_flags(unit_manual_control)
marv_frozen_suite(unit_manual_control)

# The arming and link-loss failsafe state machine (fw/arming, M2 P1, decisions 0040 and 0041): every transition, each failsafe
# boundary at +-1 us, the switch-cycle requirement and the registered parameters, on the host. marv_params_fc3v2_runtime is
# listed first so its param_get is the one linked (see L09/unit).
add_executable(unit_arming arming_test.cpp)
target_link_libraries(unit_arming PRIVATE
  marv_arming marv_params_fc3v2_runtime marv_params_fc3v2_generated marv_hal_sim GTest::gtest_main)
marv_apply_flags(unit_arming)
marv_frozen_suite(unit_arming)

# The IMU read-deadline miss count and its SIL staging (M2 P7, decision 0040 Q2).
add_executable(unit_imu_deadline imu_deadline_test.cpp)
target_link_libraries(unit_imu_deadline PRIVATE marv_hal_sim GTest::gtest_main)
marv_apply_flags(unit_imu_deadline)
marv_frozen_suite(unit_imu_deadline)

# The CRSF-to-ManualControl conversion (fw/hal/target/decode rc_input, M2 P5b): the tick-to-microsecond formula, the stick
# and throttle normalisation with its clamps, the signs, the AETR channel map and the link quality.
add_executable(unit_rc_input rc_input_test.cpp)
target_link_libraries(unit_rc_input PRIVATE marv_crsf_decode GTest::gtest_main)
marv_apply_flags(unit_rc_input)
marv_frozen_suite(unit_rc_input)
