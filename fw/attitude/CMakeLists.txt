# The L5 attitude law and angle-mode setpoint generator (quad spec section 4 L5, decision 0006 C and D). The headers carry
# the scalar-templated algorithms; attitude.cpp instantiates them for float under the firmware flags.
add_library(marv_attitude STATIC src/attitude.cpp)
target_include_directories(marv_attitude PUBLIC include)
target_link_libraries(marv_attitude
  PUBLIC marv_prim marv_types marv_params
  PRIVATE marv_hal $<COMPILE_ONLY:marv_params_generated>)
add_dependencies(marv_attitude marv_params_generated)
marv_apply_flags(marv_attitude FIRMWARE)
