# L6 stage (a), the Gazebo adapter's opt-in noisy-IMU path and the clock-error map (decision 0012), host-free.
# unit_l6_adapter_imu defines its own marv_sil_tick (a recorder), so it sees the exact bytes handed to the SIL and needs no
# SIL library; unit_l6_adapter_sil runs the real SIL (open-loop composition) for the validity rules and the stamps.
# marv_sil_init succeeds once per process, so each SIL case runs in a forked child (sil_test.cpp).
add_executable(unit_l6_adapter_imu imu_path_test.cpp clock_test.cpp)
target_include_directories(unit_l6_adapter_imu PRIVATE
  "${PROJECT_SOURCE_DIR}/tests/regression/quad/L01/unit/plant"
  "${PROJECT_SOURCE_DIR}/tests/regression/quad/L06/adapter")
target_link_libraries(unit_l6_adapter_imu PRIVATE marv_gz_adapter marv_plant marv_plant_noise marv_prim marv_types GTest::gtest_main)
marv_apply_flags(unit_l6_adapter_imu)
marv_frozen_suite(unit_l6_adapter_imu)

add_executable(unit_l6_adapter_sil sil_test.cpp)
target_include_directories(unit_l6_adapter_sil PRIVATE
  "${PROJECT_SOURCE_DIR}/tests/regression/quad/L01/unit/plant"
  "${PROJECT_SOURCE_DIR}/tests/regression/quad/L06/adapter")
target_link_libraries(unit_l6_adapter_sil PRIVATE
  marv_gz_adapter marv_plant marv_plant_noise marv_sil_l2_open_loop marv_prim marv_types
  marv_params_l2_open_loop_generated GTest::gtest_main)
marv_apply_flags(unit_l6_adapter_sil)
marv_frozen_suite(unit_l6_adapter_sil)
