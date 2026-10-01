# L6 stage (a), the plant's seeded noise source (decision 0012): counter mode equals the sequential SplitMix64, the
# vendored log/sin/cos against the platform's, moments and independence at the budget's confidence, each with a control
# that must fail, and the pinned bit pattern of the first normals.
#
# normal_moments_confidence and stream_independence_confidence are read from the design-budget register at configure time
# (no other C++ test reads the register) into a generated header, so each confidence has the one source the register
# gives it.
if(IS_ABSOLUTE "${MARV_DESIGN_BUDGET}")
  set(MARV_NOISE_BUDGET_FILE "${MARV_DESIGN_BUDGET}")
else()
  set(MARV_NOISE_BUDGET_FILE "${PROJECT_SOURCE_DIR}/${MARV_DESIGN_BUDGET}")
endif()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${MARV_NOISE_BUDGET_FILE}")
file(READ "${MARV_NOISE_BUDGET_FILE}" marv_noise_budget_text)
if(NOT marv_noise_budget_text MATCHES "\nnormal_moments_confidence:[ \t]*\n[ \t]+value:[ \t]*([0-9][0-9.eE+-]*)[ \t]*\n")
  message(FATAL_ERROR "normal_moments_confidence has no numeric value in ${MARV_NOISE_BUDGET_FILE}")
endif()
set(MARV_NOISE_NORMAL_MOMENTS_CONFIDENCE "${CMAKE_MATCH_1}")
if(NOT marv_noise_budget_text MATCHES "\nstream_independence_confidence:[ \t]*\n[ \t]+value:[ \t]*([0-9][0-9.eE+-]*)[ \t]*\n")
  message(FATAL_ERROR "stream_independence_confidence has no numeric value in ${MARV_NOISE_BUDGET_FILE}")
endif()
set(MARV_NOISE_STREAM_INDEPENDENCE_CONFIDENCE "${CMAKE_MATCH_1}")
configure_file(budget_register.hpp.in "${CMAKE_CURRENT_BINARY_DIR}/gen/budget_register.hpp" @ONLY)

add_executable(unit_l6_noise
  counter_mode_test.cpp libm_ulp_test.cpp moments_test.cpp independence_test.cpp pin_test.cpp)
target_include_directories(unit_l6_noise PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}/gen"
  "${PROJECT_SOURCE_DIR}/sim/null_plant/include")
target_link_libraries(unit_l6_noise PRIVATE marv_plant_noise marv_musl_math marv_sim_sha256 GTest::gtest_main)
marv_apply_flags(unit_l6_noise)
marv_frozen_suite(unit_l6_noise)
