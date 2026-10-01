# The firmware gyro chain (quad spec section 4 L6 stage (b), QF-7; decision 0013). The header carries the
# scalar-templated algorithm; gyro_chain.cpp instantiates it for float under the firmware flags. No composition links
# it until stage (c).
add_library(marv_gyro_chain STATIC src/gyro_chain.cpp)
target_include_directories(marv_gyro_chain PUBLIC include)
target_link_libraries(marv_gyro_chain PUBLIC marv_prim marv_types)
marv_apply_flags(marv_gyro_chain FIRMWARE)
