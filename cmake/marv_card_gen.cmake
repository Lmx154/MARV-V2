# Card-driven generators (core contracts 2.1 rule 2). Needs MARV_PYTHON (fw/params/CMakeLists.txt). Host only.
# The generated files live in ${CMAKE_BINARY_DIR}/generated/plant_card and are never committed.
#
# CARD is repository-relative or absolute. A card file is named <vehicle>.yaml (the generators refuse anything else), so the output names are known here
# without reading the card.

set(MARV_PLANT_CARD_GEN_DIR "${CMAKE_BINARY_DIR}/generated/plant_card")

function(_marv_card_gen_depends out card)
  file(GLOB tool_scripts CONFIGURE_DEPENDS "${PROJECT_SOURCE_DIR}/tools/card/*.py")
  file(GLOB profiles CONFIGURE_DEPENDS "${PROJECT_SOURCE_DIR}/sensors/profiles/*.yaml")
  set(${out} "${card}" ${profiles} ${tool_scripts} "${PROJECT_SOURCE_DIR}/sim/plant/include/marv_plant.h"
      PARENT_SCOPE)
endfunction()

# marv_add_plant_card_config(<target> CARD <card>)
#
# Runs tools/card/gen_plant_config.py at build time and exposes the header marv_plant_card_<vehicle>.h through the
# INTERFACE target <target> (include directory, build ordering, and the compile definitions
# MARV_PLANT_CARD_HEADER = that header name and MARV_PLANT_CARD_FILL = its fill function). The plugin configuration
# marv_plant_card_<vehicle>.plugin.xml is written beside it.
function(marv_add_plant_card_config target)
  cmake_parse_arguments(ARG "" "CARD" "" ${ARGN})
  cmake_path(ABSOLUTE_PATH ARG_CARD BASE_DIRECTORY "${PROJECT_SOURCE_DIR}" NORMALIZE OUTPUT_VARIABLE card)
  get_filename_component(vehicle "${card}" NAME_WE)
  set(header "${MARV_PLANT_CARD_GEN_DIR}/marv_plant_card_${vehicle}.h")
  set(plugin_xml "${MARV_PLANT_CARD_GEN_DIR}/marv_plant_card_${vehicle}.plugin.xml")
  _marv_card_gen_depends(deps "${card}")
  add_custom_command(
    OUTPUT "${header}" "${plugin_xml}"
    COMMAND "${MARV_PYTHON}" "${PROJECT_SOURCE_DIR}/tools/card/gen_plant_config.py"
            --card "${card}" --root "${PROJECT_SOURCE_DIR}" --out-dir "${MARV_PLANT_CARD_GEN_DIR}"
    DEPENDS ${deps}
    COMMENT "Generating the plant configuration header and plugin configuration (${vehicle})"
    VERBATIM)
  add_custom_target(${target}_gen DEPENDS "${header}" "${plugin_xml}")
  add_library(${target} INTERFACE)
  add_dependencies(${target} ${target}_gen)
  target_include_directories(${target} INTERFACE "${MARV_PLANT_CARD_GEN_DIR}")
  target_compile_definitions(${target} INTERFACE
    "MARV_PLANT_CARD_HEADER=\"marv_plant_card_${vehicle}.h\""
    "MARV_PLANT_CARD_FILL=marv_plant_card_${vehicle}_fill")
endfunction()

# marv_add_plant_card_sdf(<target> CARD <card>)
#
# A custom target (part of ALL) that runs tools/card/gen_sdf.py at build time and writes <vehicle>.sdf.
function(marv_add_plant_card_sdf target)
  cmake_parse_arguments(ARG "" "CARD" "" ${ARGN})
  cmake_path(ABSOLUTE_PATH ARG_CARD BASE_DIRECTORY "${PROJECT_SOURCE_DIR}" NORMALIZE OUTPUT_VARIABLE card)
  get_filename_component(vehicle "${card}" NAME_WE)
  set(sdf "${MARV_PLANT_CARD_GEN_DIR}/${vehicle}.sdf")
  _marv_card_gen_depends(deps "${card}")
  add_custom_command(
    OUTPUT "${sdf}"
    COMMAND "${MARV_PYTHON}" "${PROJECT_SOURCE_DIR}/tools/card/gen_sdf.py"
            --card "${card}" --root "${PROJECT_SOURCE_DIR}" --out-dir "${MARV_PLANT_CARD_GEN_DIR}"
    DEPENDS ${deps}
    COMMENT "Generating the vehicle SDF (${vehicle})"
    VERBATIM)
  add_custom_target(${target} ALL DEPENDS "${sdf}")
  set_target_properties(${target} PROPERTIES MARV_SDF "${sdf}")
endfunction()
