# The fixed L0 test composition (quad spec section 4, L0 T2 pass bar). Static library on host and m33: it is the first
# composition compiled for the M33, so the HAL and tick headers are compiled there too. The SIL library is built from
# it by marv_add_sil_library in sim/null_plant.
add_library(marv_composition_l0 STATIC src/l0.cpp)
target_include_directories(marv_composition_l0 PUBLIC include)
target_link_libraries(marv_composition_l0
  PUBLIC marv_hal marv_types
  PRIVATE marv_sched marv_params_l0_runtime)
marv_apply_flags(marv_composition_l0 FIRMWARE)
