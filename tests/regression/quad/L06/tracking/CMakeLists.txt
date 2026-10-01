# L6 stage (b) T1, packet P6 (decision 0013): the gyro chain's notches track the rotor speeds across the vibration sweep. The
# plant's vibration and rotor-speed model (marv_plant) feed the firmware chain (marv_gyro_chain), IMU noise off; the residual
# of each component is bounded by a_min c plus a derived allowance, with the controls that must break it (see the test).
# The shared scenario fixture is the frozen L1 plant fixture; the IMU configuration is the generated profile header's; the
# chain's tick period is read from the product parameter set (as in L06/gyro_chain).
# The relative tracking error eps and its terms are the generator's own (marv_params_gyro_chain_design.hpp, written by
# tools/card/flatten.py next to the derived parameters), so the test checks the Q that ships.
get_target_property(MARV_GYRO_CHAIN_DESIGN_HEADER marv_params_generated MARV_GYRO_CHAIN_DESIGN_HEADER)
add_executable(unit_l6_tracking tracking_test.cpp "${MARV_GYRO_CHAIN_DESIGN_HEADER}")
target_include_directories(unit_l6_tracking PRIVATE
  "$<PATH:GET_PARENT_PATH,${MARV_GYRO_CHAIN_DESIGN_HEADER}>"
  "${PROJECT_SOURCE_DIR}/tests/regression/quad/L01/unit/plant"
  "${PROJECT_SOURCE_DIR}/sim/plant/src")
target_link_libraries(unit_l6_tracking PRIVATE
  marv_gyro_chain marv_plant marv_plant_noise marv_musl_math marv_imu_profile_config marv_types marv_prim marv_params_runtime marv_hal_sim
  GTest::gtest_main)
marv_apply_flags(unit_l6_tracking)
marv_frozen_suite(unit_l6_tracking)
