# L6 stage (a), T1 Allan check (quad spec section 4 L6 pass bar (a), decision 0012): the simulated IMU's overlapping Allan
# deviation against the profile's noise density and bias instability, at the chi-square interval of allan_check_confidence
# (Bonferroni over the 12 checks), with its two negative controls (noise density x 1.1, bias instability x 1.1) and the
# record length derived by rule. unit_l6_allan_helpers holds the unit checks of the helpers (chi-square inverse, edf,
# the streaming estimator, the closed form); unit_l6_allan_check runs the model.
#
# allan_check_confidence and per_push_check_time_max come from the design-budget register and the tick period from the
# scenario values, all read at configure time into a generated header (the same pattern as L06/noise).
if(IS_ABSOLUTE "${MARV_DESIGN_BUDGET}")
  set(MARV_ALLAN_BUDGET_FILE "${MARV_DESIGN_BUDGET}")
else()
  set(MARV_ALLAN_BUDGET_FILE "${PROJECT_SOURCE_DIR}/${MARV_DESIGN_BUDGET}")
endif()
if(IS_ABSOLUTE "${MARV_SCENARIO_VALUES}")
  set(MARV_ALLAN_SCENARIO_FILE "${MARV_SCENARIO_VALUES}")
else()
  set(MARV_ALLAN_SCENARIO_FILE "${PROJECT_SOURCE_DIR}/${MARV_SCENARIO_VALUES}")
endif()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${MARV_ALLAN_BUDGET_FILE}" "${MARV_ALLAN_SCENARIO_FILE}")
file(READ "${MARV_ALLAN_BUDGET_FILE}" marv_allan_budget_text)
file(READ "${MARV_ALLAN_SCENARIO_FILE}" marv_allan_scenario_text)
if(NOT marv_allan_budget_text MATCHES "\nallan_check_confidence:[ \t]*\n[ \t]+value:[ \t]*([0-9][0-9.eE+-]*)[ \t]*\n")
  message(FATAL_ERROR "allan_check_confidence has no numeric value in ${MARV_ALLAN_BUDGET_FILE}")
endif()
set(MARV_ALLAN_CHECK_CONFIDENCE "${CMAKE_MATCH_1}")
if(NOT marv_allan_budget_text MATCHES "\nper_push_check_time_max:[ \t]*\n[ \t]+value:[ \t]*([0-9][0-9.eE+-]*)[ \t]*\n")
  message(FATAL_ERROR "per_push_check_time_max has no numeric value in ${MARV_ALLAN_BUDGET_FILE}")
endif()
set(MARV_ALLAN_PER_PUSH_CHECK_TIME_MAX "${CMAKE_MATCH_1}")
if(NOT marv_allan_scenario_text MATCHES "\ntick_period_num_us:[ \t]*\n[ \t]+value:[ \t]*([0-9]+)[ \t]*\n")
  message(FATAL_ERROR "tick_period_num_us has no integer value in ${MARV_ALLAN_SCENARIO_FILE}")
endif()
set(MARV_ALLAN_TICK_PERIOD_NUM_US "${CMAKE_MATCH_1}")
if(NOT marv_allan_scenario_text MATCHES "\ntick_period_den:[ \t]*\n[ \t]+value:[ \t]*([0-9]+)[ \t]*\n")
  message(FATAL_ERROR "tick_period_den has no integer value in ${MARV_ALLAN_SCENARIO_FILE}")
endif()
set(MARV_ALLAN_TICK_PERIOD_DEN "${CMAKE_MATCH_1}")
configure_file(allan_register.hpp.in "${CMAKE_CURRENT_BINARY_DIR}/gen/allan_register.hpp" @ONLY)

add_executable(unit_l6_allan_helpers helpers_test.cpp)
target_include_directories(unit_l6_allan_helpers PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/gen")
target_link_libraries(unit_l6_allan_helpers PRIVATE GTest::gtest_main)
marv_apply_flags(unit_l6_allan_helpers)
marv_frozen_suite(unit_l6_allan_helpers)

add_executable(unit_l6_allan_check check_test.cpp)
target_include_directories(unit_l6_allan_check PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}/gen"
  "${PROJECT_SOURCE_DIR}/tests/regression/quad/L01/unit/plant"
  "${PROJECT_SOURCE_DIR}/tests/regression/quad/L06/imu")
target_link_libraries(unit_l6_allan_check PRIVATE
  marv_plant marv_plant_noise marv_imu_profile_config marv_prim marv_types GTest::gtest_main)
marv_apply_flags(unit_l6_allan_check)
marv_frozen_suite(unit_l6_allan_check)
