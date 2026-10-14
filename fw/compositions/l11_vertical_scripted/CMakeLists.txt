# The L11 vertical-scripted test composition (quad spec section 4 L11, decision 0020): the L5 composition with the
# vertical loop in place of the constant collective, flown with a scripted stick and height reference, a thrust
# disturbance and a torque disturbance pulse read from scenario parameters. Static library on host and m33. The SIL
# library is built from it by marv_add_sil_library in sim/l11_vertical_scripted.
add_library(marv_composition_l11_vertical_scripted STATIC src/l11_vertical_scripted.cpp)
target_include_directories(marv_composition_l11_vertical_scripted PUBLIC include)
target_link_libraries(marv_composition_l11_vertical_scripted
  PUBLIC marv_hal marv_types marv_prim
  PRIVATE marv_sched marv_attitude marv_vertical marv_rate_group marv_rate marv_mixer
          marv_params_l11_vertical_scripted_runtime)
marv_apply_flags(marv_composition_l11_vertical_scripted FIRMWARE)
