# The L3 mixer / allocation (quad spec section 4 L3, decision 0004). The header carries the scalar-templated
# algorithm; mixer.cpp instantiates it for float under the firmware flags and reads the product parameter set.
add_library(marv_mixer STATIC src/mixer.cpp)
target_include_directories(marv_mixer PUBLIC include)
target_link_libraries(marv_mixer
  PUBLIC marv_prim marv_types
  PRIVATE marv_hal marv_params_runtime)
marv_apply_flags(marv_mixer FIRMWARE)
