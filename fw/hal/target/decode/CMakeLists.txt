# The LSM6DSV sample decode of the FC3v2 driver (M1 P9): FIFO words to ImuSample, the mounting map and the stamping rule.
# No Pico SDK, so it is added on every target (fw/hal/CMakeLists.txt): the m33 image links it, the L09 T1 tests run it
# on the host. The part's register map and figures are in boards/parts/lsm6dsv.hpp.
add_library(marv_lsm6dsv_decode STATIC src/lsm6dsv_decode.cpp)
target_include_directories(marv_lsm6dsv_decode PUBLIC include "${PROJECT_SOURCE_DIR}/boards")
target_link_libraries(marv_lsm6dsv_decode PUBLIC marv_types marv_prim)
marv_apply_flags(marv_lsm6dsv_decode FIRMWARE)
