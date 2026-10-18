# The configuration validator's host parts (ground-segment unit 9, decision 0053 G7; tools/validate/validate_config.py runs
# them, tools/validate/required_checks.yaml lists the checks).
#
# marv_validate_table: checks 1 and 2, the firmware's own reader (store::decode) and the composition's validator on a config
# file for the FC3v2 set. Link order as tests/regression/quad/L09/unit's unit_store: the FC3v2 set before the libraries that
# link the product set.
add_executable(marv_validate_table table_check.cpp)
target_link_libraries(marv_validate_table PRIVATE
  marv_store marv_composition_quad_fc marv_params_fc3v2_runtime marv_params_fc3v2_generated
  marv_attitude marv_estimator marv_rate_group marv_mixer marv_stick_rates marv_types marv_hal_sim)
marv_apply_flags(marv_validate_table)

# Check 3: every ctest labelled config-validation runs with the candidate config in MARV_VALIDATE_CONFIG; without it, on the
# build's factory config. qf3_table_fc3v2: the QF-3 margins of the table's rate and attitude loops by the generator's own
# rule (tools/validate/qf3_table.py). Its negative control (core 7.2 item 3, the rate gains x 1.1 on the factory config,
# whatever MARV_VALIDATE_CONFIG says) passes only if the margin evaluation ran and the roll axis failed; it is not labelled
# config-validation, so the validator does not run it.
set(fc3v2_dir "${CMAKE_BINARY_DIR}/generated/marv_params_fc3v2")
set(qf3_args
  --manifest "$<TARGET_PROPERTY:marv_params_fc3v2_generated,MARV_PARAM_MANIFEST>"
  --factory "${fc3v2_dir}/marv_params_fc3v2.mcfg"
  --card "${PROJECT_SOURCE_DIR}/vehicles/marv_5in_fc3v2.yaml"
  --budget "${PROJECT_SOURCE_DIR}/design/budget.yaml"
  --scenario "${PROJECT_SOURCE_DIR}/fw/compositions/quad_fc/params/fc3v2_scenario_values.yaml")
add_test(NAME qf3_table_fc3v2
  COMMAND "${MARV_PYTHON}" "${CMAKE_CURRENT_SOURCE_DIR}/qf3_table.py" ${qf3_args})
add_test(NAME qf3_table_fc3v2_negative_control
  COMMAND "${MARV_PYTHON}" "${CMAKE_CURRENT_SOURCE_DIR}/qf3_table.py" ${qf3_args} --gain-factor 1.1)
set_tests_properties(qf3_table_fc3v2 PROPERTIES LABELS config-validation)
set_tests_properties(qf3_table_fc3v2_negative_control PROPERTIES
  ENVIRONMENT_MODIFICATION "MARV_VALIDATE_CONFIG=unset:"
  PASS_REGULAR_EXPRESSION "qf3_table: rate roll worst PM [^\n]*; FAIL")
