# The CRSF frame decoder (fw/hal/target/decode, M2 P5a): frame parsing, CRC8 0xD5, the 11-bit channel unpack, resynchronisation
# and the error counters, on the host.
add_executable(unit_crsf crsf_test.cpp)
target_link_libraries(unit_crsf PRIVATE marv_crsf_decode GTest::gtest_main)
marv_apply_flags(unit_crsf)
marv_frozen_suite(unit_crsf)

# The arming and link-loss failsafe state machine (fw/arming, M2 P1, decisions 0040 and 0041): every transition, each failsafe
# boundary at +-1 us, the switch-cycle requirement and the registered parameters, on the host. marv_params_fc3v2_runtime is
# listed first so its param_get is the one linked (see L09/unit).
add_executable(unit_arming arming_test.cpp)
target_link_libraries(unit_arming PRIVATE
  marv_arming marv_params_fc3v2_runtime marv_params_fc3v2_generated marv_hal_sim GTest::gtest_main)
marv_apply_flags(unit_arming)
marv_frozen_suite(unit_arming)
