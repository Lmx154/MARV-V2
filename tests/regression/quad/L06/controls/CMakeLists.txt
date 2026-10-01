# Negative control for the per-push CTest timeout (cmake/marv_test_timeout.cmake). The planted test sleeps longer than the
# limit, so it exists only in the control build, which sets MARV_PER_PUSH_CHECK_TIME_MAX_OVERRIDE (ci/run_ci.sh,
# per_push_timeout_control). It is not registered in any preset build.
if(NOT "${MARV_PER_PUSH_CHECK_TIME_MAX_OVERRIDE}" STREQUAL "")
  math(EXPR planted_sleep_s "${MARV_PER_PUSH_CHECK_TIME_MAX_OVERRIDE} * 3")
  add_test(NAME per_push_timeout_planted COMMAND "${CMAKE_COMMAND}" -E sleep ${planted_sleep_s})
endif()
