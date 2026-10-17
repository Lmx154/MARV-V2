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
set(MARV_MAVLINK_DEFS_DIR "${CMAKE_BINARY_DIR}/mavlink/defs")
set(MARV_MAVLINK_C_DIR "${CMAKE_BINARY_DIR}/mavlink/c")
set(MARV_LINK_GEN_ROOT "${CMAKE_BINARY_DIR}/generated/marv_link")
set(MARV_LINK_DIALECT "${PROJECT_SOURCE_DIR}/link/marv.xml")
set(MARV_LINK_SCOPE "${PROJECT_SOURCE_DIR}/link/scope.txt")
set(MARV_LINK_TOOLS "${PROJECT_SOURCE_DIR}/tools/link")

set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  "${MARV_LINK_DIALECT}" "${PROJECT_SOURCE_DIR}/link/mavlink_pin.json" "${MARV_LINK_TOOLS}/fetch_defs.py")

execute_process(
  COMMAND "${MARV_PYTHON}" "${MARV_LINK_TOOLS}/fetch_defs.py" --out-dir "${MARV_MAVLINK_DEFS_DIR}"
  RESULT_VARIABLE _marv_defs_result OUTPUT_VARIABLE _marv_defs_out ERROR_VARIABLE _marv_defs_error)
if(NOT _marv_defs_result EQUAL 0)
  message(FATAL_ERROR "Fetching the pinned MAVLink message definitions failed (link/mavlink_pin.json): ${_marv_defs_error}")
endif()

set(_marv_defs_xml
  "${MARV_MAVLINK_DEFS_DIR}/marv.xml" "${MARV_MAVLINK_DEFS_DIR}/common.xml"
  "${MARV_MAVLINK_DEFS_DIR}/standard.xml" "${MARV_MAVLINK_DEFS_DIR}/minimal.xml")

add_custom_command(
  OUTPUT "${MARV_MAVLINK_C_DIR}/marv/mavlink.h"
  COMMAND "${CMAKE_COMMAND}" -E rm -rf "${MARV_MAVLINK_C_DIR}"
  COMMAND "${MARV_PYTHON}" -m pymavlink.tools.mavgen --lang C --wire-protocol 2.0
          --output "${MARV_MAVLINK_C_DIR}" "${MARV_MAVLINK_DEFS_DIR}/marv.xml"
  DEPENDS ${_marv_defs_xml}
  COMMENT "Generating the MAVLink C headers (pymavlink mavgen, MARV dialect)"
  VERBATIM)

add_custom_command(
  OUTPUT "${MARV_LINK_GEN_ROOT}/marv/link/messages.hpp"
  COMMAND "${MARV_PYTHON}" "${MARV_LINK_TOOLS}/gen_cpp_structs.py"
          --dialect "${MARV_LINK_DIALECT}" --include-dir "${MARV_MAVLINK_DEFS_DIR}"
          --scope "${MARV_LINK_SCOPE}" --out "${MARV_LINK_GEN_ROOT}/marv/link/messages.hpp"
  DEPENDS ${_marv_defs_xml} "${MARV_LINK_DIALECT}" "${MARV_LINK_SCOPE}"
          "${MARV_LINK_TOOLS}/gen_cpp_structs.py" "${MARV_LINK_TOOLS}/mavxml.py"
  COMMENT "Generating the plain link message structs"
  VERBATIM)

add_custom_target(marv_link_generated DEPENDS
  "${MARV_LINK_GEN_ROOT}/marv/link/messages.hpp" "${MARV_MAVLINK_C_DIR}/marv/mavlink.h")

add_library(marv_link INTERFACE)
target_include_directories(marv_link INTERFACE include "${MARV_LINK_GEN_ROOT}")
add_dependencies(marv_link marv_link_generated)

add_library(marv_link_codec_mavlink STATIC src/codec_mavlink.cpp)
target_include_directories(marv_link_codec_mavlink SYSTEM PRIVATE "${MARV_MAVLINK_C_DIR}")
target_link_libraries(marv_link_codec_mavlink PUBLIC marv_link)
add_dependencies(marv_link_codec_mavlink marv_link_generated)
marv_apply_flags(marv_link_codec_mavlink FIRMWARE)
