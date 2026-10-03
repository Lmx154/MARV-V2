# The l5_attitude_scripted composition: its scripted inputs as pure functions (unit_l5_script) and the whole composition
# through the SIL entry points, with a reference tick loop built from the same modules (unit_l5_composition).
add_executable(unit_l5_script script_test.cpp)
target_include_directories(unit_l5_script PRIVATE
  "${PROJECT_SOURCE_DIR}/fw/compositions/l5_attitude_scripted/include")
target_link_libraries(unit_l5_script PRIVATE marv_prim marv_types GTest::gtest_main)
marv_apply_flags(unit_l5_script)
marv_frozen_suite(unit_l5_script)

# The composition test builds its reference (attitude, rate and mixer configurations on the composition's default table
# with the test's overrides) in this executable. marv_params_l5_attitude_scripted_runtime is listed before marv_attitude,
# marv_rate and marv_mixer so its param_get is the one linked (see sim/l5_attitude_scripted/CMakeLists.txt).
add_executable(unit_l5_composition composition_test.cpp)
target_include_directories(unit_l5_composition PRIVATE
  "${PROJECT_SOURCE_DIR}/fw/compositions/l5_attitude_scripted/include")
target_link_libraries(unit_l5_composition PRIVATE
  marv_sil_l5_attitude_scripted marv_params_l5_attitude_scripted_runtime marv_params_l5_attitude_scripted_generated
  marv_attitude marv_rate_group marv_rate marv_mixer marv_types marv_hal_sim GTest::gtest_main)
marv_apply_flags(unit_l5_composition)
marv_frozen_suite(unit_l5_composition)

# The composition's set extends the product set with every product id unchanged (its ids are a prefix). The control
# swaps the arguments: the product manifest is not an extension of the composition's, so the check must fail.
set(l5_product_manifest "$<TARGET_PROPERTY:marv_params_generated,MARV_PARAM_MANIFEST>")
set(l5_extended_manifest "$<TARGET_PROPERTY:marv_params_l5_attitude_scripted_generated,MARV_PARAM_MANIFEST>")
add_test(NAME l5_composition_param_prefix
  COMMAND "${MARV_PYTHON}" "${CMAKE_CURRENT_SOURCE_DIR}/check_param_prefix.py"
          "${l5_product_manifest}" "${l5_extended_manifest}")
add_test(NAME l5_composition_param_prefix_negative_control
  COMMAND "${MARV_PYTHON}" "${CMAKE_CURRENT_SOURCE_DIR}/check_param_prefix.py"
          "${l5_extended_manifest}" "${l5_product_manifest}")
set_tests_properties(l5_composition_param_prefix l5_composition_param_prefix_negative_control PROPERTIES LABELS frozen)
set_tests_properties(l5_composition_param_prefix_negative_control PROPERTIES WILL_FAIL TRUE)
