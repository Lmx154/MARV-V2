# The link codec (core section 8; ground segment unit 5, docs/decisions/0053). Two generated inputs, both in the build tree:
#   - the MAVLink C headers: pymavlink's mavgen run on link/marv.xml (the MARV dialect, which includes the standard
#     common.xml, standard.xml and minimal.xml). The standard XML is fetched at configure time at the commit pinned in
#     link/mavlink_pin.json (tools/link/fetch_defs.py checks each file's SHA-256) and never vendored. pymavlink is LGPL-3 and
#     is a build tool only (a dev dependency in uv.lock); the code it generates is MIT under the exception in its COPYING
#     (THIRD_PARTY_NOTICES.md). The headers are SYSTEM includes of marv_link_codec_mavlink alone (GS-3).
#   - marv/link/messages.hpp: the plain message structs, written by tools/link/gen_cpp_structs.py from the same dialect and
#     link/scope.txt. It includes no MAVLink header.
# marv_link is the interface every module may use (envelope, frame, status, the structs); marv_link_codec_mavlink is the
# codec and the only code that sees MAVLink.
# The generation step (marv_link_generated and the MARV_LINK_GEN_ROOT / MARV_MAVLINK_C_DIR variables) is defined in
# link/CMakeLists.txt, outside fw/: a custom target has no code for the flight gates (G3) to check.
set(MARV_MAVLINK_C_DIR "${CMAKE_BINARY_DIR}/mavlink/c")             # as in link/CMakeLists.txt
set(MARV_LINK_GEN_ROOT "${CMAKE_BINARY_DIR}/generated/marv_link")  # as in link/CMakeLists.txt
add_library(marv_link INTERFACE)
target_include_directories(marv_link INTERFACE include "${MARV_LINK_GEN_ROOT}")
add_dependencies(marv_link marv_link_generated)

add_library(marv_link_codec_mavlink STATIC src/codec_mavlink.cpp)
target_include_directories(marv_link_codec_mavlink SYSTEM PRIVATE "${MARV_MAVLINK_C_DIR}")
target_link_libraries(marv_link_codec_mavlink PUBLIC marv_link)
add_dependencies(marv_link_codec_mavlink marv_link_generated)
marv_apply_flags(marv_link_codec_mavlink FIRMWARE)
