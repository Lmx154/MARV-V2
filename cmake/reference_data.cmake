# Reference data that is regenerated, not stored (decision 0011; tools/refdata/refdata.py). Needs MARV_PYTHON
# (fw/params/CMakeLists.txt). Host only.
#
# marv_reference_set(<id> <dir_var> <fixture_var>)
#   <dir_var>      the directory the set is generated into: $MARV_REFERENCE_DIR (read at configure time) or
#                  <source>/build/reference, then /<id>.
#   <fixture_var>  the ctest fixture name. A test that reads the directory lists it in FIXTURES_REQUIRED; the fixture's
#                  setup test runs `refdata.py ensure <id>`, which regenerates the files when they are missing or differ from
#                  the committed SHA256SUMS and fails when the generated files do not match it.
function(marv_reference_set id dir_var fixture_var)
  if(DEFINED ENV{MARV_REFERENCE_DIR} AND NOT "$ENV{MARV_REFERENCE_DIR}" STREQUAL "")
    set(out "$ENV{MARV_REFERENCE_DIR}")
  else()
    set(out "${PROJECT_SOURCE_DIR}/build/reference")
  endif()
  string(REPLACE "/" "_" fixture "marv_reference_${id}")
  add_test(NAME ${fixture}_ensure
    COMMAND "${MARV_PYTHON}" "${PROJECT_SOURCE_DIR}/tools/refdata/refdata.py" ensure "${id}" --out "${out}")
  set_tests_properties(${fixture}_ensure PROPERTIES FIXTURES_SETUP ${fixture} LABELS reference)
  set(${dir_var} "${out}/${id}" PARENT_SCOPE)
  set(${fixture_var} ${fixture} PARENT_SCOPE)
endfunction()
