# marv_ground: the mission software. Sends MissionCommand / SetPreset / Reboot frames, reads Telemetry.
add_executable(marv_ground src/main.cpp src/transport.cpp)
target_link_libraries(marv_ground PRIVATE marv_link)
