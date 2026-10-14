include(GoogleTest)

# Every test under tests/regression/<product>/Lnn/ carries the ctest label Lnn besides its own labels (the selector and the
# smoke test run a layer's entries with ctest -L). Two paths, one rule (the layer directory name):
#   - gtest_discover_tests (discovery happens at test time): marv_frozen_suite takes the label from its own source directory
#     and adds it to the discovered tests through an include file that runs after the discovery.
#   - add_test (scripts, binaries, reference-set steps, controls): after each layer directory is added, the loop below
#     appends the label to every test registered in it and its subdirectories.
function(marv_frozen_suite test_target)
  if(NOT CMAKE_CURRENT_SOURCE_DIR MATCHES "/(L[0-9][0-9])(/|$)")
    message(FATAL_ERROR "marv_frozen_suite(${test_target}): ${CMAKE_CURRENT_SOURCE_DIR} is not under a layer directory Lnn")
  endif()
  set(layer_label "${CMAKE_MATCH_1}")
  gtest_discover_tests(${test_target} PROPERTIES LABELS frozen TIMEOUT ${MARV_PER_PUSH_CHECK_TIME_MAX} ${ARGN})
  # A label list cannot pass through gtest_discover_tests PROPERTIES (the list is flattened), so an include file, run after
  # the discovery file, appends the layer label to the discovered tests (the variable <target>_TESTS holds their names).
  set(label_file "${CMAKE_CURRENT_BINARY_DIR}/${test_target}_layer_label.cmake")
  file(WRITE "${label_file}"
    "foreach(t IN LISTS ${test_target}_TESTS)\n"
    "  set_tests_properties(\"\${t}\" PROPERTIES LABELS \"frozen;${layer_label}\")\n"
    "endforeach()\n")
  set_property(DIRECTORY APPEND PROPERTY TEST_INCLUDE_FILES "${label_file}")
endfunction()

function(marv_label_tests_in dir label)
  get_property(tests DIRECTORY "${dir}" PROPERTY TESTS)
  foreach(t IN LISTS tests)
    get_test_property(${t} LABELS DIRECTORY "${dir}" labels)
    if(labels STREQUAL "NOTFOUND")
      set(labels "")
    endif()
    list(APPEND labels "${label}")
    set_tests_properties(${t} DIRECTORY "${dir}" PROPERTIES LABELS "${labels}")
  endforeach()
  get_property(subdirs DIRECTORY "${dir}" PROPERTY SUBDIRECTORIES)
  foreach(sub IN LISTS subdirs)
    marv_label_tests_in("${sub}" "${label}")
  endforeach()
endfunction()

file(GLOB marv_frozen_layers
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/*/L[0-9][0-9]/CMakeLists.txt")
list(SORT marv_frozen_layers)
foreach(layer_cmake IN LISTS marv_frozen_layers)
  get_filename_component(layer_dir "${layer_cmake}" DIRECTORY)
  add_subdirectory("${layer_dir}")
  get_filename_component(layer_label "${layer_dir}" NAME)
  marv_label_tests_in("${CMAKE_CURRENT_SOURCE_DIR}/${layer_dir}" "${layer_label}")
endforeach()
