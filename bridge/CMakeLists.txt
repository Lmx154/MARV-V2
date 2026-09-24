# marv_bridge: steps the Gazebo world in lockstep with the flight software over the wire protocol.
find_package(gz-msgs10 REQUIRED)

add_executable(marv_bridge src/main.cpp src/endpoint.cpp src/gz_world.cpp)
target_link_libraries(marv_bridge PRIVATE marv_link marv_fsw gz-transport13::core gz-msgs10::core)
