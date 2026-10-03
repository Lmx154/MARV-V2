# Quad L6 suite. Stage (a), the sensor noise model: the seeded noise source (noise/).
add_subdirectory(noise)
# The plant's generic IMU model (imu/).
add_subdirectory(imu)
# The Gazebo adapter's noisy-IMU path and the clock-error map (adapter/).
add_subdirectory(adapter)
# SIM-2 with noise: two separate processes of the host-free run driver (sim/l6_noise_run) with the same seed give
# identical logs (sim2/).
add_subdirectory(sim2)
# L6 stage (a) T1 Allan check: the simulated IMU's Allan deviation against the profile, with its two negative controls
# (allan/).
add_subdirectory(allan)
add_subdirectory(gyro_chain)
add_subdirectory(vibration)
add_subdirectory(rotor_speed)
add_subdirectory(tracking)
add_subdirectory(d_term)
add_subdirectory(rate_group)
