# The L4 rate-scripted test composition (quad spec section 4 L4, decision 0005 T4 bullet): the rate loop and the mixer
# flown with scripted setpoint, chirp and thrust read from scenario parameters. Static library on host and m33. The SIL
# library is built from it by marv_add_sil_library in sim/l4_rate_scripted.
add_library(marv_composition_l4_rate_scripted STATIC src/l4_rate_scripted.cpp)
target_include_directories(marv_composition_l4_rate_scripted PUBLIC include)
target_link_libraries(marv_composition_l4_rate_scripted
  PUBLIC marv_hal marv_types marv_prim
  PRIVATE marv_sched marv_rate_group marv_rate marv_mixer marv_params_l4_rate_scripted_runtime)
marv_apply_flags(marv_composition_l4_rate_scripted FIRMWARE)
