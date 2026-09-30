# The L5 attitude-scripted test composition (quad spec section 4 L5, decision 0006 B, D, E and F): angle mode, the attitude
# law, the rate loop in bypass and the mixer, flown with a scripted stick, chirp, yaw disturbance and thrust read from
# scenario parameters. Static library on host and m33. The SIL library is built from it by marv_add_sil_library in
# sim/l5_attitude_scripted.
add_library(marv_composition_l5_attitude_scripted STATIC src/l5_attitude_scripted.cpp)
target_include_directories(marv_composition_l5_attitude_scripted PUBLIC include)
target_link_libraries(marv_composition_l5_attitude_scripted
  PUBLIC marv_hal marv_types marv_prim
  PRIVATE marv_sched marv_attitude marv_rate marv_mixer marv_params_l5_attitude_scripted_runtime)
marv_apply_flags(marv_composition_l5_attitude_scripted FIRMWARE)
