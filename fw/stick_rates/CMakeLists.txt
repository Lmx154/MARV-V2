# The QF-1 rates curve (quad spec section 4.0 M2, packet P2): stick in [-1, 1] to body-rate setpoint in rad/s. The header
# carries the scalar-templated function; stick_rates.cpp instantiates it for float under the firmware flags. A pure
# function: no parameters are read here (the composition builds the configuration from them).
add_library(marv_stick_rates STATIC src/stick_rates.cpp)
target_include_directories(marv_stick_rates PUBLIC include)
target_link_libraries(marv_stick_rates PUBLIC marv_prim)
marv_apply_flags(marv_stick_rates FIRMWARE)
