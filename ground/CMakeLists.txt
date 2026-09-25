# marv_ground: the mission software. Sends MissionCommand / SetPreset / Reboot frames, reads Telemetry.
# Boost.JSON (header-only) reads the pilot's stored radio config (radio_json.hpp).
find_package(Boost 1.83 REQUIRED CONFIG)
add_executable(marv_ground src/main.cpp src/transport.cpp src/json.cpp)
target_link_libraries(marv_ground PRIVATE marv_link Boost::headers)
