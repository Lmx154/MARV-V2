add_executable(unit_l1_param_record
  sigma_kind_test.cpp
  "${PROJECT_SOURCE_DIR}/fw/sil/src/param_override.cpp")
target_include_directories(unit_l1_param_record PRIVATE
  "${PROJECT_SOURCE_DIR}/fw/sil/src"
  "${PROJECT_SOURCE_DIR}/fw/sil/include")
target_link_libraries(unit_l1_param_record PRIVATE marv_params_l1_fixture_runtime marv_hal_sim GTest::gtest_main)
marv_apply_flags(unit_l1_param_record)
marv_frozen_suite(unit_l1_param_record)
