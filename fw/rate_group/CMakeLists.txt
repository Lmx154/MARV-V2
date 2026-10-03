# The rate group's step (quad spec section 4 L6 stage (c), decision 0014 "Wiring"): the gyro chain in front of the rate
# loop, then the mixer, shared by the L4 and L5 compositions and the harness replays. Reads the product parameter set
# (as marv_rate does).
add_library(marv_rate_group STATIC src/rate_group.cpp)
target_include_directories(marv_rate_group PUBLIC include)
target_link_libraries(marv_rate_group
  PUBLIC marv_gyro_chain marv_rate marv_mixer marv_prim marv_types
  PRIVATE marv_hal marv_params_runtime)
marv_apply_flags(marv_rate_group FIRMWARE)
