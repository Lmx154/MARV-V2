# The l4_rate_scripted composition: its scripted inputs as pure functions (unit_l4_script) and the whole composition
# through the SIL C ABI (unit_l4_composition).
add_executable(unit_l4_script script_test.cpp)
target_include_directories(unit_l4_script PRIVATE
  "${PROJECT_SOURCE_DIR}/fw/compositions/l4_rate_scripted/include")
target_link_libraries(unit_l4_script PRIVATE marv_prim marv_types GTest::gtest_main)
marv_apply_flags(unit_l4_script)
marv_frozen_suite(unit_l4_script)

# The composition test builds its reference (mixer::load_config on the composition's default table) in this executable.
# marv_params_l4_rate_scripted_runtime is listed before marv_mixer so its param_get is the one linked (see
# sim/l4_rate_scripted/CMakeLists.txt).
add_executable(unit_l4_composition composition_test.cpp)
target_include_directories(unit_l4_composition PRIVATE
  "${PROJECT_SOURCE_DIR}/fw/compositions/l4_rate_scripted/include")
target_link_libraries(unit_l4_composition PRIVATE
  marv_sil_l4_rate_scripted marv_params_l4_rate_scripted_runtime marv_params_l4_rate_scripted_generated
  marv_mixer marv_types marv_hal_sim GTest::gtest_main)
marv_apply_flags(unit_l4_composition)
marv_frozen_suite(unit_l4_composition)

# The composition's set extends the product set with every product id unchanged (its ids are a prefix). The control
# swaps the arguments: the product manifest is not an extension of the composition's, so the check must fail.
set(l4_product_manifest "$<TARGET_PROPERTY:marv_params_generated,MARV_PARAM_MANIFEST>")
set(l4_extended_manifest "$<TARGET_PROPERTY:marv_params_l4_rate_scripted_generated,MARV_PARAM_MANIFEST>")
add_test(NAME l4_composition_param_prefix
  COMMAND "${MARV_PYTHON}" "${CMAKE_CURRENT_SOURCE_DIR}/check_param_prefix.py"
          "${l4_product_manifest}" "${l4_extended_manifest}")
add_test(NAME l4_composition_param_prefix_negative_control
  COMMAND "${MARV_PYTHON}" "${CMAKE_CURRENT_SOURCE_DIR}/check_param_prefix.py"
          "${l4_extended_manifest}" "${l4_product_manifest}")
set_tests_properties(l4_composition_param_prefix l4_composition_param_prefix_negative_control PROPERTIES LABELS frozen)
set_tests_properties(l4_composition_param_prefix_negative_control PROPERTIES WILL_FAIL TRUE)
