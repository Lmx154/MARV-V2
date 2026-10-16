# The LSM6DSV sample decode of the FC3v2 driver (M1 P9): FIFO words to ImuSample, the mounting map and the stamping rule.
# No Pico SDK, so it is added on every target (fw/hal/CMakeLists.txt): the m33 image links it, the L09 T1 tests run it
# on the host. The part's register map and figures are in boards/parts/lsm6dsv.hpp.
add_library(marv_lsm6dsv_decode STATIC src/lsm6dsv_decode.cpp)
target_include_directories(marv_lsm6dsv_decode PUBLIC include "${PROJECT_SOURCE_DIR}/boards")
target_link_libraries(marv_lsm6dsv_decode PUBLIC marv_types marv_prim)
marv_apply_flags(marv_lsm6dsv_decode FIRMWARE)

# The DShot600 output logic of the FC3v2 (M1 P10): frame encoder, pad permutation, word packing, PIO bit timing and the
# panic loop's stop frames. No Pico SDK, so it is added on every target: the m33 image links it, the L09 T1 tests run it
# on the host.
add_library(marv_dshot STATIC src/dshot.cpp)
target_include_directories(marv_dshot PUBLIC include)
target_link_libraries(marv_dshot PUBLIC marv_types marv_prim)
marv_apply_flags(marv_dshot FIRMWARE)

# The FC3v2 console's line parser (M1 P11): the motor test command and its unlock token. No Pico SDK, so it is added on every
# target: the m33 image links it, the L09 T1 tests run it on the host.
add_library(marv_console_decode STATIC src/console.cpp)
target_include_directories(marv_console_decode PUBLIC include)
target_link_libraries(marv_console_decode PUBLIC marv_hal marv_prim)
marv_apply_flags(marv_console_decode FIRMWARE)

# The CRSF frame decoder of the ELRS receiver link (M2 P5a): a streaming parser for the RC channels (0x16) and link statistics
# (0x14) frames. No Pico SDK, so it is added on every target: the m33 image links it, the L08 T1 tests run it on the host.
add_library(marv_crsf_decode STATIC src/crsf.cpp)
target_include_directories(marv_crsf_decode PUBLIC include)
target_link_libraries(marv_crsf_decode PUBLIC marv_prim)
marv_apply_flags(marv_crsf_decode FIRMWARE)
