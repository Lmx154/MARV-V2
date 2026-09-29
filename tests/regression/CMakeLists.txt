include(GoogleTest)

function(marv_frozen_suite test_target)
  gtest_discover_tests(${test_target} PROPERTIES LABELS frozen ${ARGN})
endfunction()

file(GLOB marv_frozen_layers
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/*/L[0-9][0-9]/CMakeLists.txt")
list(SORT marv_frozen_layers)
foreach(layer_cmake IN LISTS marv_frozen_layers)
  get_filename_component(layer_dir "${layer_cmake}" DIRECTORY)
  add_subdirectory("${layer_dir}")
endforeach()
