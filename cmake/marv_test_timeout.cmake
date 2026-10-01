# The per-push time limit as the CTest TIMEOUT of every per-push check (quad L6 stage (b), decision 0013; the 60 s rule is
# S9, decision 0009 second message item 4).
# The limit is the design-budget entry per_push_check_time_max, read here once at configure time; no other file carries
# its value. A check that overruns it fails the ctest run (a timeout is a failure), so the limit is enforced, not measured
# by hand.
#
# MARV_PER_PUSH_CHECK_TIME_MAX_OVERRIDE exists only for the negative control (ci/run_ci.sh, per_push_timeout_control): it
# replaces the register value in that control's own build tree so the control needs seconds, not the limit, of wall time.
# No preset sets it.
#
# How the timeout reaches each registration path:
#   - gtest_discover_tests: its two callers, marv_frozen_suite (tests/regression/CMakeLists.txt) and tests/unit, pass
#     PROPERTIES TIMEOUT ${MARV_PER_PUSH_CHECK_TIME_MAX}, so each discovered test carries it.
#   - add_test (scripts, binaries, the reference-set steps): marv_apply_per_push_timeout, deferred to the end of the top
#     level directory, sets TIMEOUT on every test registered by then that has none.
# Exemption: the reference-set generation steps (tests labelled `reference`, made by marv_reference_set, decision 0011)
# are not per-push checks. On a fresh tree they regenerate a reference set (about 110 s for the L5 set); CI generates the
# set in an earlier step, where they take 0.03 s. They keep CTest's default timeout.
if(IS_ABSOLUTE "${MARV_DESIGN_BUDGET}")
  set(marv_timeout_budget_file "${MARV_DESIGN_BUDGET}")
else()
  set(marv_timeout_budget_file "${PROJECT_SOURCE_DIR}/${MARV_DESIGN_BUDGET}")
endif()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${marv_timeout_budget_file}")
file(READ "${marv_timeout_budget_file}" marv_timeout_budget_text)
if(NOT marv_timeout_budget_text MATCHES "\nper_push_check_time_max:[ \t]*\n[ \t]+value:[ \t]*([0-9][0-9.eE+-]*)[ \t]*\n")
  message(FATAL_ERROR "per_push_check_time_max has no numeric value in ${marv_timeout_budget_file}")
endif()
set(MARV_PER_PUSH_CHECK_TIME_MAX "${CMAKE_MATCH_1}")
if(NOT "${MARV_PER_PUSH_CHECK_TIME_MAX_OVERRIDE}" STREQUAL "")
  set(MARV_PER_PUSH_CHECK_TIME_MAX "${MARV_PER_PUSH_CHECK_TIME_MAX_OVERRIDE}")
endif()

function(marv_apply_per_push_timeout_in dir)
  get_property(tests DIRECTORY "${dir}" PROPERTY TESTS)
  foreach(t IN LISTS tests)
    get_test_property(${t} LABELS DIRECTORY "${dir}" labels)
    get_test_property(${t} TIMEOUT DIRECTORY "${dir}" existing)
    if("reference" IN_LIST labels OR NOT existing STREQUAL "NOTFOUND")
      continue()
    endif()
    set_tests_properties(${t} DIRECTORY "${dir}" PROPERTIES TIMEOUT ${MARV_PER_PUSH_CHECK_TIME_MAX})
  endforeach()
  get_property(subdirs DIRECTORY "${dir}" PROPERTY SUBDIRECTORIES)
  foreach(sub IN LISTS subdirs)
    marv_apply_per_push_timeout_in("${sub}")
  endforeach()
endfunction()

function(marv_apply_per_push_timeout)
  marv_apply_per_push_timeout_in("${PROJECT_SOURCE_DIR}")
endfunction()

cmake_language(DEFER DIRECTORY "${PROJECT_SOURCE_DIR}" CALL marv_apply_per_push_timeout)
