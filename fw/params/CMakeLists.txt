add_library(marv_params INTERFACE)
target_include_directories(marv_params INTERFACE include)

set(MARV_PARAM_CARD_FILES "${PROJECT_SOURCE_DIR}/tests/fixtures/params/l0_card.yaml" CACHE STRING
    "Vehicle-card parameter sources (origin default-from-card); the L0 test fixture until L1 supplies the card")
set(MARV_PARAM_REGISTER_FILES "${PROJECT_SOURCE_DIR}/tests/fixtures/params/l0_register.yaml" CACHE STRING
    "Design-budget register parameter sources (origin default-from-register); the L0 test fixture until L1")
set(MARV_PYTHON "${PROJECT_SOURCE_DIR}/.venv/bin/python" CACHE FILEPATH
    "Python of the uv environment (created by `uv sync --frozen`) that runs tools/gen/params_gen.py")

if(NOT EXISTS "${MARV_PYTHON}")
  message(FATAL_ERROR
    "MARV_PYTHON='${MARV_PYTHON}' does not exist. Run `uv sync --frozen` in ${PROJECT_SOURCE_DIR} "
    "to create the uv environment, or set -DMARV_PYTHON=<python with PyYAML>.")
endif()

set(MARV_PARAMS_GEN_DIR "${CMAKE_BINARY_DIR}/generated/marv/params")
set(MARV_PARAMS_GEN_SCRIPT "${PROJECT_SOURCE_DIR}/tools/gen/params_gen.py")

set(_marv_param_args)
foreach(_f IN LISTS MARV_PARAM_CARD_FILES)
  list(APPEND _marv_param_args --card "${_f}")
endforeach()
foreach(_f IN LISTS MARV_PARAM_REGISTER_FILES)
  list(APPEND _marv_param_args --register "${_f}")
endforeach()

add_custom_command(
  OUTPUT "${MARV_PARAMS_GEN_DIR}/param_ids.hpp"
         "${MARV_PARAMS_GEN_DIR}/param_defaults.cpp"
         "${MARV_PARAMS_GEN_DIR}/params_manifest.json"
  COMMAND "${MARV_PYTHON}" "${MARV_PARAMS_GEN_SCRIPT}" ${_marv_param_args} --out-dir "${MARV_PARAMS_GEN_DIR}"
  DEPENDS "${MARV_PARAMS_GEN_SCRIPT}" ${MARV_PARAM_CARD_FILES} ${MARV_PARAM_REGISTER_FILES}
  COMMENT "Generating parameter ids, defaults and manifest"
  VERBATIM)

add_library(marv_params_generated STATIC
  "${MARV_PARAMS_GEN_DIR}/param_defaults.cpp"
  "${MARV_PARAMS_GEN_DIR}/param_ids.hpp")
target_include_directories(marv_params_generated PUBLIC "${CMAKE_BINARY_DIR}/generated")
target_link_libraries(marv_params_generated PUBLIC marv_params)
marv_apply_flags(marv_params_generated FIRMWARE)
