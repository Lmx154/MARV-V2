# The Mahony attitude filter and the boot gyro calibration (quad spec section 4.0 M3, L7): pure libraries, no HAL, no parameter
# read, no clock. The headers carry the scalar-templated code; mahony.cpp and gyro_cal.cpp instantiate it for float under the
# firmware flags. The composition builds the configurations (the gains, the calibration window) from parameters.
add_library(marv_estimator STATIC src/mahony.cpp src/gyro_cal.cpp)
target_include_directories(marv_estimator PUBLIC include)
target_link_libraries(marv_estimator PUBLIC marv_prim)
marv_apply_flags(marv_estimator FIRMWARE)
