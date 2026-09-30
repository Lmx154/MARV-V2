# The L4 rate loop (quad spec section 4 L4). The header carries the scalar-templated algorithm; rate_loop.cpp
# instantiates it for float under the firmware flags.
add_library(marv_rate STATIC src/rate_loop.cpp)
target_include_directories(marv_rate PUBLIC include)
target_link_libraries(marv_rate
  PUBLIC marv_prim marv_types marv_mixer
  PRIVATE marv_hal marv_params_runtime)
marv_apply_flags(marv_rate FIRMWARE)
