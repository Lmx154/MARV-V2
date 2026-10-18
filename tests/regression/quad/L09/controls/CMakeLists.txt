# The post-link parameter-table check's negative control (cmake/param_table.cmake; ci/run_ci.sh param_table_control).
# Built only by its CI step: the FC3v2 runtime with the product set's generated archive linked ahead of FC3v2's
# (LINK_ONLY, so the source still sees only the FC3v2 header), checked against the FC3v2 set. The build must fail in the
# check and leave no image.
add_executable(param_table_planted EXCLUDE_FROM_ALL param_table_planted.cpp)
target_link_libraries(param_table_planted PRIVATE marv_params_fc3v2_runtime $<LINK_ONLY:marv_params_generated>)
if(MARV_TARGET STREQUAL "m33")
  target_link_libraries(param_table_planted PRIVATE pico_runtime)
endif()
marv_apply_flags(param_table_planted FIRMWARE)
marv_check_param_table(param_table_planted marv_params_fc3v2)
