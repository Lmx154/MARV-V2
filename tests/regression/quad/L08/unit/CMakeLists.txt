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

# The IMU read-deadline miss count and its SIL staging (M2 P7, decision 0040 Q2).
add_executable(unit_imu_deadline imu_deadline_test.cpp)
target_link_libraries(unit_imu_deadline PRIVATE marv_hal_sim GTest::gtest_main)
marv_apply_flags(unit_imu_deadline)
marv_frozen_suite(unit_imu_deadline)
