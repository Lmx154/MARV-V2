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

# The ground link service (include/marv/link/service.hpp, src/service.cpp; ground-segment unit 6, decision 0053): the codec,
# the config store and the parameters of the flight composition's set, which it takes compile-only like fw/store (the binary
# that links it links the set). The SIL library of the flight composition links it (fw/sil, STORAGE), and so does the board's
# flight image (fw/hal/target, ground-segment unit 12b). On the host the two archives are built PIC for that shared library.
add_library(marv_link_service STATIC src/service.cpp)
target_include_directories(marv_link_service PUBLIC include)
target_link_libraries(marv_link_service
  PUBLIC marv_link marv_store
  PRIVATE marv_link_codec_mavlink marv_hal marv_params marv_arming marv_prim marv_types
          $<COMPILE_ONLY:marv_params_fc3v2_generated>)
add_dependencies(marv_link_service marv_params_fc3v2_generated)
marv_apply_flags(marv_link_service FIRMWARE)
if(MARV_TARGET STREQUAL "host")
  set_target_properties(marv_link_codec_mavlink marv_link_service PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()
# On the board both are built for size (-Os, after the build type's -O3 on the command line, so it wins): they run only in the
# background loop, below every interrupt, and the image runs from SRAM (copy_to_ram), so their code is RAM. Measured at
# ground-segment unit 11 (the telemetry streams): at -O3 the flight image overflowed RAM by 16340 B; -Os takes the codec from
# 45628 to 13304 B of text and the service from 12855 to 7821 B (arm-none-eabi-size of the m33 objects). Lead decision,
# 2026-10-19.
if(MARV_TARGET STREQUAL "m33")
  target_compile_options(marv_link_codec_mavlink PRIVATE -Os)
  target_compile_options(marv_link_service PRIVATE -Os)
endif()

# The same service compiled against the FC3v2 set with the logger register (marv_params_fc3v2_log) and with the store of that set
# (marv_store_log), for the flight image with the logger only (MARV_FC3V2_FLIGHT_LOG, fw/hal/target/CMakeLists.txt; onboard
# logger unit U4-4), built for size like the service above. Not defined in any other configuration.
if(MARV_FC3V2_FLIGHT_LOG)
  add_library(marv_link_service_log STATIC src/service.cpp)
  target_include_directories(marv_link_service_log PUBLIC include)
  target_link_libraries(marv_link_service_log
    PUBLIC marv_link marv_store_log
    PRIVATE marv_link_codec_mavlink marv_hal marv_params marv_arming marv_prim marv_types
            $<COMPILE_ONLY:marv_params_fc3v2_log_generated>)
  add_dependencies(marv_link_service_log marv_params_fc3v2_log_generated)
  marv_apply_flags(marv_link_service_log FIRMWARE)
  if(MARV_TARGET STREQUAL "m33")
    target_compile_options(marv_link_service_log PRIVATE -Os)
  endif()
endif()
