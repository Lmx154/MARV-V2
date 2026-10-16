# The CRSF frame decoder (fw/hal/target/decode, M2 P5a): frame parsing, CRC8 0xD5, the 11-bit channel unpack, resynchronisation
# and the error counters, on the host.
add_executable(unit_crsf crsf_test.cpp)
target_link_libraries(unit_crsf PRIVATE marv_crsf_decode GTest::gtest_main)
marv_apply_flags(unit_crsf)
marv_frozen_suite(unit_crsf)
