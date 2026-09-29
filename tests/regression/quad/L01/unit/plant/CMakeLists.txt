set(marv_plant_ref_dir "${CMAKE_CURRENT_SOURCE_DIR}/reference")

foreach(name motor wrench kat abi)
  add_executable(unit_plant_${name} ${name}_test.cpp)
  target_link_libraries(unit_plant_${name} PRIVATE marv_plant marv_prim GTest::gtest_main)
  target_compile_definitions(unit_plant_${name} PRIVATE MARV_PLANT_REF_DIR="${marv_plant_ref_dir}")
  marv_apply_flags(unit_plant_${name})
  marv_frozen_suite(unit_plant_${name})
endforeach()
