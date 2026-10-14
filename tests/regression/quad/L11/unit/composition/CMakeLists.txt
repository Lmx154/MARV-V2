# T1 of the l11_vertical_scripted composition (decision 0020 section 5, "composition order (vertical before rate, subset
# of the rate ticks, stale-latch panic, id prefix)", and the composition's parameter validation, hover equality, sign,
# script stamps, thrust disturbance, torque pulse and invalid-state fallback): the whole composition through the SIL
# entry points (marv_sil.h, marv_truth.h, marv_truth_vertical.h) against a reference tick loop built in this executable
# from the same modules. marv_params_l11_vertical_scripted_runtime is listed before marv_rate_group, marv_rate and
# marv_mixer so its param_get is the one linked (see sim/l11_vertical_scripted/CMakeLists.txt).
add_executable(unit_l11_composition composition_test.cpp)
target_link_libraries(unit_l11_composition PRIVATE
  marv_sil_l11_vertical_scripted marv_params_l11_vertical_scripted_runtime marv_params_l11_vertical_scripted_generated
  marv_vertical marv_rate_group marv_rate marv_mixer marv_types marv_hal_sim GTest::gtest_main)
marv_apply_flags(unit_l11_composition)
marv_frozen_suite(unit_l11_composition)

# The composition's set extends the product set with every product id unchanged, and every id it adds starts with l11_ or
# vert_ (decision 0020 section 4: the composition's own register, appended last). Controls: the arguments swapped (the
# product manifest is not an extension of the composition's) and a prefix list the added ids do not match.
set(l11_product_manifest "$<TARGET_PROPERTY:marv_params_generated,MARV_PARAM_MANIFEST>")
set(l11_extended_manifest "$<TARGET_PROPERTY:marv_params_l11_vertical_scripted_generated,MARV_PARAM_MANIFEST>")
add_test(NAME l11_composition_param_ids
  COMMAND "${MARV_PYTHON}" "${CMAKE_CURRENT_SOURCE_DIR}/check_param_ids.py"
          "${l11_product_manifest}" "${l11_extended_manifest}" l11_ vert_)
add_test(NAME l11_composition_param_ids_negative_control_swapped
  COMMAND "${MARV_PYTHON}" "${CMAKE_CURRENT_SOURCE_DIR}/check_param_ids.py"
          "${l11_extended_manifest}" "${l11_product_manifest}" l11_ vert_)
add_test(NAME l11_composition_param_ids_negative_control_prefix
  COMMAND "${MARV_PYTHON}" "${CMAKE_CURRENT_SOURCE_DIR}/check_param_ids.py"
          "${l11_product_manifest}" "${l11_extended_manifest}" l5_)
set_tests_properties(l11_composition_param_ids l11_composition_param_ids_negative_control_swapped
  l11_composition_param_ids_negative_control_prefix PROPERTIES LABELS frozen)
set_tests_properties(l11_composition_param_ids_negative_control_swapped
  l11_composition_param_ids_negative_control_prefix PROPERTIES WILL_FAIL TRUE)
