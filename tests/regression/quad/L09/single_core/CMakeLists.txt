# The negative control of invariant L-8b's post-link check (tools/ci/check_single_core.py; onboard logger unit U4-2), m33 only
# (added from the top-level CMakeLists.txt): an SDK image that launches core 1. Built by every m33 build; the check in REQUIRE
# mode fails the build unless it finds multicore_launch_core1 in this image, so the FORBID on the flight image cannot pass
# vacuously (the L-6 pattern of fw/hal/target/CMakeLists.txt).
add_executable(single_core_planted single_core_planted.cpp)
target_link_libraries(single_core_planted PRIVATE pico_stdlib pico_multicore)
marv_apply_flags(single_core_planted FIRMWARE)
marv_check_single_core(single_core_planted REQUIRE)
