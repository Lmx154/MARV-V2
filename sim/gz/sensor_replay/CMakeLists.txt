# sensor_replay (host only, harness only; decision 0019, W1 C2): the SIL-2 replay of a lockstep log of a model-gyro run
# against direct marv_plant calls. tests/regression/quad/L06/gz/test_sensor_replay.py runs it from the build that holds
# the Gazebo plugin. It links marv_plant and the generated IMU header only: no SIL library, no adapter, no Gazebo. The
# log's record enum and constants are the plugin's (plugin/src, headers only), kMicrosecondsPerSecond the adapter's, and
# the sample layouts marv_sil.h's and marv_truth.h's (headers only).
add_executable(sensor_replay sensor_replay.cpp)
target_include_directories(sensor_replay PRIVATE
  "${PROJECT_SOURCE_DIR}/sim/gz/plugin/src"
  "${PROJECT_SOURCE_DIR}/sim/gz/adapter/include"
  "${PROJECT_SOURCE_DIR}/fw/sil/include")
target_link_libraries(sensor_replay PRIVATE marv_plant marv_imu_profile_config)
marv_apply_flags(sensor_replay)
