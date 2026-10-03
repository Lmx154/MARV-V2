# l4_acro_replay (harness only, not firmware): the l4_rate_scripted composition's due-tick sequence replayed on a logged
# run's IMU samples, with the run's sil_overrides applied by the SIL init's own mechanism (fw/sil/src/param_override.cpp,
# compiled in here as marv_add_sil_library compiles it). tests/regression/quad/L04/gz/test_t4_acro.py runs it from the
# build that holds the Gazebo plugin (tools/sim/run_l4.py replay_tool). marv_params_l4_rate_scripted_runtime is listed
# before marv_rate and marv_mixer so its param_get is the one linked (as in sim/l4_rate_scripted/CMakeLists.txt).
add_executable(l4_acro_replay
  l4_acro_replay.cpp
  "${PROJECT_SOURCE_DIR}/fw/sil/src/param_override.cpp")
target_include_directories(l4_acro_replay PRIVATE
  "${PROJECT_SOURCE_DIR}/fw/compositions/l4_rate_scripted/include"
  "${PROJECT_SOURCE_DIR}/fw/sil/include"
  "${PROJECT_SOURCE_DIR}/fw/sil/src")
target_link_libraries(l4_acro_replay PRIVATE
  marv_params_l4_rate_scripted_runtime marv_params_l4_rate_scripted_generated
  marv_rate_group marv_rate marv_mixer marv_sched marv_types marv_prim marv_hal_sim)
marv_apply_flags(l4_acro_replay)
