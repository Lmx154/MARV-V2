# The Mahony attitude filter (quad spec section 4.0 M3, L7): a pure library, no HAL, no parameter read, no clock. The header
# carries the scalar-templated filter; mahony.cpp instantiates it for float under the firmware flags. The composition builds
# the configuration (the gains) from parameters.
add_library(marv_estimator STATIC src/mahony.cpp)
target_include_directories(marv_estimator PUBLIC include)
target_link_libraries(marv_estimator PUBLIC marv_prim)
marv_apply_flags(marv_estimator FIRMWARE)
