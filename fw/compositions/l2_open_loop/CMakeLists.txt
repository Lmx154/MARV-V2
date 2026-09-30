# The L2 open-loop composition (quad spec section 4 L2, decision 0003 item 5): every tick outputs the per-motor DShot
# read from parameters. Static library on host and m33. The SIL library is built from it by marv_add_sil_library in
# sim/l2_open_loop.
add_library(marv_composition_l2_open_loop STATIC src/l2_open_loop.cpp)
target_include_directories(marv_composition_l2_open_loop PUBLIC include)
target_link_libraries(marv_composition_l2_open_loop
  PUBLIC marv_hal marv_types
  PRIVATE marv_params_l2_open_loop_runtime)
marv_apply_flags(marv_composition_l2_open_loop FIRMWARE)
