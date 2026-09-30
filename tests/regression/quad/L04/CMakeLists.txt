# Quad L4 suite. Each unit subdirectory registers its own tests.
file(GLOB marv_l04_units
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/unit/*/CMakeLists.txt")
list(SORT marv_l04_units)
foreach(unit_cmake IN LISTS marv_l04_units)
  get_filename_component(unit_dir "${unit_cmake}" DIRECTORY)
  add_subdirectory("${unit_dir}")
endforeach()

# T3: the float rate loop closed with the design plant in double against the committed double oracle.
add_subdirectory(t3)
# The acro replay tool (harness only): the composition's allocation outputs on a logged run's samples.
add_subdirectory(replay)
