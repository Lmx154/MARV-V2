# Quad L2 suite. Each unit subdirectory registers its own tests.
file(GLOB marv_l02_units
  RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
  CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/unit/*/CMakeLists.txt")
list(SORT marv_l02_units)
foreach(unit_cmake IN LISTS marv_l02_units)
  get_filename_component(unit_dir "${unit_cmake}" DIRECTORY)
  add_subdirectory("${unit_dir}")
endforeach()
