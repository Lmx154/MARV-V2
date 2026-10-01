# Host simulation helpers shared by the sim drivers and their tests (header-only).
#   marv_sim_sha256   SHA-256 (FIPS 180-4), marv/sim/sha256.hpp
add_library(marv_sim_sha256 INTERFACE)
target_include_directories(marv_sim_sha256 INTERFACE include)
