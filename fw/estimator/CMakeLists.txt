# The Mahony attitude filter and the boot gyro calibration (quad spec section 4.0 M3, L7), and the attitude and gyro-bias ESKF
# in shadow (M4, decision 0055): pure libraries, no HAL, no parameter read, no clock. The headers carry the scalar-templated
# code; mahony.cpp, gyro_cal.cpp and eskf.cpp instantiate it for float under the firmware flags. The composition builds the
# configurations (the gains, the calibration window) from parameters. No flight composition references the ESKF.
add_library(marv_estimator STATIC src/mahony.cpp src/gyro_cal.cpp src/eskf.cpp)
target_include_directories(marv_estimator PUBLIC include)
target_link_libraries(marv_estimator PUBLIC marv_prim)
marv_apply_flags(marv_estimator FIRMWARE)
