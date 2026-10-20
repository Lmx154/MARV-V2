# The negative control of invariant L-5a's post-link check (tools/ci/check_emb6_open.py; onboard logger unit U4-4), m33 only
# (added from the top-level CMakeLists.txt): an SDK image with a symbol that names the measurement build. Built by every m33 build;
# the check in REQUIRE mode fails the build unless it finds emb6_open in this image, so the FORBID on the flight image cannot pass
# vacuously (the L-8b pattern of tests/regression/quad/L09/single_core).
add_executable(emb6_open_planted emb6_open_planted.cpp)
target_link_libraries(emb6_open_planted PRIVATE pico_stdlib)
marv_apply_flags(emb6_open_planted FIRMWARE)
marv_check_emb6_open(emb6_open_planted REQUIRE)
