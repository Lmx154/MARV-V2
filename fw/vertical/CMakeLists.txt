# The L11 vertical loop (decision 0020, section 1). The headers carry the scalar-templated law; vertical.cpp
# instantiates it for float under the firmware flags.
add_library(marv_vertical STATIC src/vertical.cpp)
target_include_directories(marv_vertical PUBLIC include)
target_link_libraries(marv_vertical PUBLIC marv_prim marv_types marv_mixer)
marv_apply_flags(marv_vertical FIRMWARE)
