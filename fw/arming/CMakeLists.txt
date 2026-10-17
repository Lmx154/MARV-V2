# The arming and link-loss failsafe state machine of the quad (quad spec section 4.0 M2, decisions 0040 and 0041). The header
# carries the scalar-templated machine, pure (no HAL, no clock read: the caller passes the tick time); arming.cpp
# instantiates it for float and reads the six parameters of fw/compositions/quad_fc/params/quad_fc_register.yaml.
add_library(marv_arming STATIC src/arming.cpp)
target_include_directories(marv_arming PUBLIC include)
target_link_libraries(marv_arming
  PUBLIC marv_prim marv_params
  PRIVATE marv_hal marv_params_fc3v2_generated)
marv_apply_flags(marv_arming FIRMWARE)
