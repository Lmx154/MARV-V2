# Generation of the link codec's inputs (ground segment unit 5, docs/decisions/0053); see fw/link/CMakeLists.txt.
# Kept outside fw/ so the custom target is not a flight target (cmake/flight_targets.cmake collects fw/ targets for G3).
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
