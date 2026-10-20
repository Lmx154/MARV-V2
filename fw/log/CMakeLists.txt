# The onboard SD flight logger's core (unit U1 LOG-CORE; architect ruling 2026-10-18): the record formats, the SPSC ring, the
# sector framer, the header writer and the logger state machine behind an abstract storage. Pure C++20: no HAL, no FatFs, no
# Pico SDK, no parameter set (marv/log/params.hpp is a header a consumer includes against its own set), so the library builds
# for the host and the m33 alike. The flight image links it (decision 0060). The logger on the disarmed arena
# (src/arena_logger.cpp, unit U4-2) takes the arena through the config store's lease, marv_store_arena (no parameter set either).
add_library(marv_log STATIC src/ring.cpp src/framer.cpp src/header.cpp src/schema.cpp src/logger.cpp src/file_life.cpp src/arena_logger.cpp)
target_include_directories(marv_log PUBLIC include)
target_link_libraries(marv_log PUBLIC marv_prim marv_types marv_params PRIVATE marv_store_arena)
marv_apply_flags(marv_log FIRMWARE)
