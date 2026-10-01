# l6_noise_run (host only): the host-free run driver of the L6 stage (a) T1 SIM-2-with-noise check. It closes the Gazebo
# adapter (marv_gz_adapter), marv_plant with its IMU and the l4_rate_scripted SIL library (a flight composition built
# without a truth-state entry; the firmware sees the plant's noisy IMU bytes). The vehicle is the product card's plant
# configuration (MARV_PLANT_CARD_HEADER) and the IMU is the sensor profile's (marv_imu_profile_config). The SHA-256 is sim/common (marv_sim_sha256).
add_executable(l6_noise_run main.cpp)
target_include_directories(l6_noise_run PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(l6_noise_run PRIVATE
  marv_gz_adapter marv_sim_sha256 marv_plant marv_plant_card marv_imu_profile_config marv_plant_noise marv_sil_l4_rate_scripted marv_prim marv_types
  marv_params_l4_rate_scripted_generated)
marv_apply_flags(l6_noise_run)
