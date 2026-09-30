add_library(marv_params INTERFACE)
target_include_directories(marv_params INTERFACE include)

set(MARV_VEHICLE_CARD "vehicles/uzh_neurobem_5in.yaml" CACHE STRING
    "Vehicle card of the product parameter set (repository-relative or absolute); linted and flattened by tools/card/flatten.py")
set(MARV_DESIGN_BUDGET "design/budget.yaml" CACHE STRING
    "Design-budget register of the product parameter set (repository-relative or absolute)")
set(MARV_SCENARIO_VALUES "design/scenario_values.yaml" CACHE STRING
    "Scenario-values register of the product parameter set (repository-relative or absolute)")
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
           "${gen_dir}/params_provenance.json"
    COMMAND "${MARV_PYTHON}" "${MARV_PARAMS_GEN_SCRIPT}" ${gen_args} --out-dir "${gen_dir}"
    DEPENDS "${MARV_PARAMS_GEN_SCRIPT}" ${ARG_CARD} ${ARG_REGISTER}
    COMMENT "Generating parameter ids, defaults, manifest and provenance (${name})"
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

# marv_card_profile_file(<out-var> <name> <card.yaml>): the sensor profile file named by the card's `sensor_profile:` line.
function(marv_card_profile_file out name card)
  file(STRINGS "${card}" _profile_line REGEX "^sensor_profile:")
  string(REGEX REPLACE "^sensor_profile:[ ]*([A-Za-z0-9_]+).*$" "\\1" _profile "${_profile_line}")
  if(NOT _profile OR _profile STREQUAL _profile_line)
    message(FATAL_ERROR "marv_add_card_param_set(${name}): no `sensor_profile: <id>` line in ${card}")
  endif()
  set(${out} "${PROJECT_SOURCE_DIR}/sensors/profiles/${_profile}.yaml" PARENT_SCOPE)
endfunction()

# marv_add_card_param_set(<name> CARD <card.yaml> BUDGET <budget.yaml> [SCENARIO <scenario_values.yaml>]
#                         [EXTRA_REGISTER <register.yaml>])
#
# A parameter set (marv_add_param_set) whose sources are a vehicle card and the design-budget register: a build-time
# custom command runs tools/card/flatten.py (lints first; a lint finding fails the build) into
# ${CMAKE_BINARY_DIR}/generated/<name>/flatten/ and marv_add_param_set consumes its outputs: the flattened card and
# register plus the L3 mixer parameters (idle_speed, mixer_m<i>_*; tools/card/mixer.py). With SCENARIO the
# scenario-values register is linted and flattened too (--scenario/--out-scenario) and joins the CARD files, and the
# L4 rate-loop parameters (rate_{kp,ki,kd,tau_ref}_<axis>; tools/card/rate.py, --out-rate) and their derivation report
# are generated from it, and so are the L5 attitude-loop parameters (att_kp, att_yaw_weight, att_loop_ratio;
# tools/card/attitude.py, --out-attitude) with their report (<name>_attitude_report.txt). The command depends on
# the card, the sensor profile the card names, the budget, the scenario register and tools/card/*.py.
# EXTRA_REGISTER appends one more register file after the budget register, so every id of the set without it keeps its
# value and the extra entries follow (a composition's own scenario parameters); without it the outputs are unchanged.
function(marv_add_card_param_set name)
  cmake_parse_arguments(ARG "" "CARD;BUDGET;SCENARIO;EXTRA_REGISTER" "" ${ARGN})
  cmake_path(ABSOLUTE_PATH ARG_CARD BASE_DIRECTORY "${PROJECT_SOURCE_DIR}" NORMALIZE)
  cmake_path(ABSOLUTE_PATH ARG_BUDGET BASE_DIRECTORY "${PROJECT_SOURCE_DIR}" NORMALIZE)
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${ARG_CARD}")
  set(scenario_args)
  set(scenario_outputs)
  set(scenario_depends)
  set(rate_args)
  set(rate_outputs)
  set(attitude_args)
  set(attitude_outputs)
  if(ARG_SCENARIO)
    cmake_path(ABSOLUTE_PATH ARG_SCENARIO BASE_DIRECTORY "${PROJECT_SOURCE_DIR}" NORMALIZE)
    set(flat_scenario "${CMAKE_BINARY_DIR}/generated/${name}/flatten/${name}_scenario.yaml")
    set(scenario_args --scenario "${ARG_SCENARIO}" --out-scenario "${flat_scenario}")
    set(scenario_outputs "${flat_scenario}")
    set(scenario_depends "${ARG_SCENARIO}")
    set(flat_rate "${CMAKE_BINARY_DIR}/generated/${name}/flatten/${name}_rate.yaml")
    set(flat_rate_report "${CMAKE_BINARY_DIR}/generated/${name}/flatten/${name}_rate_report.txt")
    set(rate_args --out-rate "${flat_rate}")
    set(rate_outputs "${flat_rate}" "${flat_rate_report}")
    set(flat_attitude "${CMAKE_BINARY_DIR}/generated/${name}/flatten/${name}_attitude.yaml")
    set(flat_attitude_report "${CMAKE_BINARY_DIR}/generated/${name}/flatten/${name}_attitude_report.txt")
    set(attitude_args --out-attitude "${flat_attitude}")
    set(attitude_outputs "${flat_attitude}" "${flat_attitude_report}")
  endif()

  marv_card_profile_file(profile_file ${name} "${ARG_CARD}")

  set(flat_dir "${CMAKE_BINARY_DIR}/generated/${name}/flatten")
  set(flat_card "${flat_dir}/${name}_card.yaml")
  set(flat_register "${flat_dir}/${name}_register.yaml")
  set(flat_mixer "${flat_dir}/${name}_mixer.yaml")
  file(GLOB marv_card_tools CONFIGURE_DEPENDS "${PROJECT_SOURCE_DIR}/tools/card/*.py")

  add_custom_command(
    OUTPUT "${flat_card}" "${flat_register}" "${flat_mixer}" ${scenario_outputs} ${rate_outputs} ${attitude_outputs}
    COMMAND "${MARV_PYTHON}" "${PROJECT_SOURCE_DIR}/tools/card/flatten.py"
            --card "${ARG_CARD}" --budget "${ARG_BUDGET}"
            --out-card "${flat_card}" --out-register "${flat_register}" --out-mixer "${flat_mixer}"
            ${scenario_args} ${rate_args} ${attitude_args}
            --root "${PROJECT_SOURCE_DIR}"
    DEPENDS "${ARG_CARD}" "${profile_file}" "${ARG_BUDGET}" ${scenario_depends} ${marv_card_tools}
    COMMENT "Linting and flattening the vehicle card and design budget (${name})"
    VERBATIM)

  marv_add_param_set(${name} CARD "${flat_card}" "${flat_mixer}" ${scenario_outputs} ${flat_rate} ${flat_attitude} REGISTER "${flat_register}" ${ARG_EXTRA_REGISTER})
endfunction()

# marv_add_card_report(<name> CARD <card.yaml> BUDGET <budget.yaml>)
#
# The target <name>_report ALWAYS runs at build time (the generator commit is part of it) and writes
# ${CMAKE_BINARY_DIR}/generated/<name>/l1_report.txt, tools/card/report.py's L1 run report. It is a custom (UTILITY)
# target with no code, so it is called from the top-level CMakeLists.txt, outside fw/, where G3 does not treat it as a
# flight target (tools/ci/check_g3.py refuses a target type it cannot scan).
function(marv_add_card_report name)
  cmake_parse_arguments(ARG "" "CARD;BUDGET" "" ${ARGN})
  cmake_path(ABSOLUTE_PATH ARG_CARD BASE_DIRECTORY "${PROJECT_SOURCE_DIR}" NORMALIZE)
  cmake_path(ABSOLUTE_PATH ARG_BUDGET BASE_DIRECTORY "${PROJECT_SOURCE_DIR}" NORMALIZE)
  marv_card_profile_file(profile_file ${name} "${ARG_CARD}")
  file(GLOB marv_card_tools CONFIGURE_DEPENDS "${PROJECT_SOURCE_DIR}/tools/card/*.py")

  add_custom_target(${name}_report ALL
    COMMAND "${MARV_PYTHON}" "${PROJECT_SOURCE_DIR}/tools/card/report.py"
            --card "${ARG_CARD}" --budget "${ARG_BUDGET}" --root "${PROJECT_SOURCE_DIR}"
            --out "${CMAKE_BINARY_DIR}/generated/${name}/l1_report.txt"
    BYPRODUCTS "${CMAKE_BINARY_DIR}/generated/${name}/l1_report.txt"
    DEPENDS "${ARG_CARD}" "${profile_file}" "${ARG_BUDGET}" ${marv_card_tools}
    COMMENT "Writing the L1 run report (${name})"
    VERBATIM)
endfunction()

# The product parameter set: the vehicle card, the design-budget register and the scenario-values register. (The
# INTERFACE library `marv_params` above is the include root that every set's <name>_generated links; this set's
# targets are marv_params_generated and marv_params_runtime.)
marv_add_card_param_set(marv_params CARD "${MARV_VEHICLE_CARD}" BUDGET "${MARV_DESIGN_BUDGET}"
  SCENARIO "${MARV_SCENARIO_VALUES}")

# The L0 parameter set: built from the frozen L0 fixtures, whatever MARV_PARAM_*_FILES name. The L0 composition, the
# null plant, the SIL test library and the L0 tests link it, so later layers can change the product's parameter
# sources without touching them.
marv_add_param_set(marv_params_l0
  CARD "${PROJECT_SOURCE_DIR}/tests/regression/quad/L00/fixtures/l0_card.yaml"
  REGISTER "${PROJECT_SOURCE_DIR}/tests/regression/quad/L00/fixtures/l0_register.yaml")

# The L1 parameter-record fixture set: every sigma kind, lock and the vector shapes, exercised through params_init.
marv_add_param_set(marv_params_l1_fixture
  CARD "${PROJECT_SOURCE_DIR}/tests/regression/quad/L01/fixtures/l1_card.yaml"
  REGISTER "${PROJECT_SOURCE_DIR}/tests/regression/quad/L01/fixtures/l1_register.yaml")

# The L2 open-loop composition's parameter set: the four per-motor DShot commands, default 0 (stop).
marv_add_param_set(marv_params_l2_open_loop
  REGISTER "${PROJECT_SOURCE_DIR}/fw/compositions/l2_open_loop/params/l2_open_loop_register.yaml")

# The L4 rate-scripted composition's parameter set: the product set (card, budget, mixer, rate, scenario values) followed
# by the composition's own register (thrust, setpoint script, chirp; neutral defaults). marv_rate and marv_mixer read
# the product ids, which this set keeps.
marv_add_card_param_set(marv_params_l4_rate_scripted CARD "${MARV_VEHICLE_CARD}" BUDGET "${MARV_DESIGN_BUDGET}"
  SCENARIO "${MARV_SCENARIO_VALUES}"
  EXTRA_REGISTER "${PROJECT_SOURCE_DIR}/fw/compositions/l4_rate_scripted/params/l4_rate_scripted_register.yaml")

# The L5 attitude-scripted composition's parameter set: the product set (card, budget, mixer, rate, attitude, scenario
# values) followed by the composition's own register (thrust, stick script, chirp, yaw disturbance; neutral defaults).
# marv_attitude, marv_rate and marv_mixer read the product ids, which this set keeps.
marv_add_card_param_set(marv_params_l5_attitude_scripted CARD "${MARV_VEHICLE_CARD}" BUDGET "${MARV_DESIGN_BUDGET}"
  SCENARIO "${MARV_SCENARIO_VALUES}"
  EXTRA_REGISTER "${PROJECT_SOURCE_DIR}/fw/compositions/l5_attitude_scripted/params/l5_attitude_scripted_register.yaml")
