add_library(marv_params INTERFACE)
target_include_directories(marv_params INTERFACE include)

set(MARV_PARAM_CARD_FILES "${PROJECT_SOURCE_DIR}/tests/regression/quad/L00/fixtures/l0_card.yaml" CACHE STRING
    "Vehicle-card parameter sources (origin default-from-card); the L0 test fixture until L1 supplies the card")
set(MARV_PARAM_REGISTER_FILES "${PROJECT_SOURCE_DIR}/tests/regression/quad/L00/fixtures/l0_register.yaml" CACHE STRING
    "Design-budget register parameter sources (origin default-from-register); the L0 test fixture until L1")
set(MARV_PYTHON "${PROJECT_SOURCE_DIR}/.venv/bin/python" CACHE FILEPATH
    "Python of the uv environment (created by `uv sync --frozen`) that runs tools/gen/params_gen.py")

if(NOT EXISTS "${MARV_PYTHON}")
  message(FATAL_ERROR
    "MARV_PYTHON='${MARV_PYTHON}' does not exist. Run `uv sync --frozen` in ${PROJECT_SOURCE_DIR} "
    "to create the uv environment, or set -DMARV_PYTHON=<python with PyYAML>.")
endif()
execute_process(COMMAND "${MARV_PYTHON}" -c "import yaml"
  RESULT_VARIABLE _marv_yaml_result OUTPUT_QUIET ERROR_VARIABLE _marv_yaml_error)
if(NOT _marv_yaml_result EQUAL 0)
  message(FATAL_ERROR
    "MARV_PYTHON='${MARV_PYTHON}' cannot import PyYAML (${_marv_yaml_error}). Run `uv sync --frozen` in "
    "${PROJECT_SOURCE_DIR}, or set -DMARV_PYTHON=<python with PyYAML>.")
endif()

set(MARV_PARAMS_GEN_SCRIPT "${PROJECT_SOURCE_DIR}/tools/gen/params_gen.py")
set(MARV_PARAMS_RUNTIME_SOURCE "${CMAKE_CURRENT_LIST_DIR}/src/param.cpp")

# marv_add_param_set(<name> CARD <files...> REGISTER <files...>)
#
# A self-contained parameter set: runs tools/gen/params_gen.py on the given sources into its own output directory
# ${CMAKE_BINARY_DIR}/generated/<name> and builds two static FIRMWARE libraries (host and m33):
#   <name>_generated   the generated param_ids.hpp and param_defaults.cpp; PUBLIC include root is that directory, so
#                      <marv/params/param_ids.hpp> resolves to this set's header only
#   <name>_runtime     src/param.cpp compiled against <name>_generated
# Its target property MARV_PARAM_MANIFEST on <name>_generated is the path of the set's params_manifest.json, and
# MARV_PARAM_CARD_FILES / MARV_PARAM_REGISTER_FILES hold the sources. Sets are independent: a binary must link
# the generated and runtime library of one set only, since every set defines the same marv::generated symbols.
# Call it from fw/params (the targets take the calling directory as SOURCE_DIR, which puts them under fw/ for G3).
function(marv_add_param_set name)
  cmake_parse_arguments(ARG "" "" "CARD;REGISTER" ${ARGN})
  set(gen_root "${CMAKE_BINARY_DIR}/generated/${name}")
  set(gen_dir "${gen_root}/marv/params")

  set(gen_args)
  foreach(f IN LISTS ARG_CARD)
    list(APPEND gen_args --card "${f}")
  endforeach()
  foreach(f IN LISTS ARG_REGISTER)
    list(APPEND gen_args --register "${f}")
  endforeach()

  add_custom_command(
    OUTPUT "${gen_dir}/param_ids.hpp"
           "${gen_dir}/param_defaults.cpp"
           "${gen_dir}/params_manifest.json"
    COMMAND "${MARV_PYTHON}" "${MARV_PARAMS_GEN_SCRIPT}" ${gen_args} --out-dir "${gen_dir}"
    DEPENDS "${MARV_PARAMS_GEN_SCRIPT}" ${ARG_CARD} ${ARG_REGISTER}
    COMMENT "Generating parameter ids, defaults and manifest (${name})"
    VERBATIM)

  add_library(${name}_generated STATIC
    "${gen_dir}/param_defaults.cpp"
    "${gen_dir}/param_ids.hpp")
  target_include_directories(${name}_generated PUBLIC "${gen_root}")
  target_link_libraries(${name}_generated PUBLIC marv_params)
  set_target_properties(${name}_generated PROPERTIES
    MARV_PARAM_MANIFEST "${gen_dir}/params_manifest.json"
    MARV_PARAM_CARD_FILES "${ARG_CARD}"
    MARV_PARAM_REGISTER_FILES "${ARG_REGISTER}")
  marv_apply_flags(${name}_generated FIRMWARE)

  add_library(${name}_runtime STATIC "${MARV_PARAMS_RUNTIME_SOURCE}")
  target_link_libraries(${name}_runtime PUBLIC ${name}_generated marv_hal)
  marv_apply_flags(${name}_runtime FIRMWARE)
endfunction()

marv_add_param_set(marv_params CARD ${MARV_PARAM_CARD_FILES} REGISTER ${MARV_PARAM_REGISTER_FILES})

# The L0 parameter set: built from the frozen L0 fixtures, whatever MARV_PARAM_*_FILES name. The L0 composition, the
# null plant, the SIL test library and the L0 tests link it, so later layers can change the product's parameter
# sources without touching them.
marv_add_param_set(marv_params_l0
  CARD "${PROJECT_SOURCE_DIR}/tests/regression/quad/L00/fixtures/l0_card.yaml"
  REGISTER "${PROJECT_SOURCE_DIR}/tests/regression/quad/L00/fixtures/l0_register.yaml")
