# G3 target manifest (core section 7.4, row G3).
#
# marv_write_g3_manifest(<file> [FLIGHT <target>...] [SIL <target>...])
#   Writes <file> at generate time. JSON: {"flight_targets": [{"name", "archive", "objects"}],
#   "sil_libraries": [{"name", "file"}]}. tools/ci/check_g3.py reads it.
#
# marv_collect_flight_targets(<out-var>)
#   Every STATIC library defined anywhere in the project whose source directory is under fw/, except fw/hal/sim and
#   fw/sil. Derived from the directory tree, so a new fw/ library is a flight target without any list to update.
#
# marv_collect_sil_libraries(<out-var>)
#   Every SHARED or MODULE library defined in the project (all of them are SIL entry libraries), except the negative
#   controls under tests/controls.

function(marv_all_project_targets out)
  set(dirs "${PROJECT_SOURCE_DIR}")
  set(targets "")
  while(dirs)
    list(POP_FRONT dirs dir)
    get_property(dir_targets DIRECTORY "${dir}" PROPERTY BUILDSYSTEM_TARGETS)
    list(APPEND targets ${dir_targets})
    get_property(subdirs DIRECTORY "${dir}" PROPERTY SUBDIRECTORIES)
    list(APPEND dirs ${subdirs})
  endwhile()
  set(${out} "${targets}" PARENT_SCOPE)
endfunction()

function(marv_collect_flight_targets out)
  marv_all_project_targets(all_targets)
  set(fw_dir "${PROJECT_SOURCE_DIR}/fw")
  set(hal_sim_dir "${fw_dir}/hal/sim")
  set(sil_dir "${fw_dir}/sil")
  set(flight "")
  foreach(tgt IN LISTS all_targets)
    get_target_property(type ${tgt} TYPE)
    if(NOT type STREQUAL "STATIC_LIBRARY")
      continue()
    endif()
    get_target_property(src_dir ${tgt} SOURCE_DIR)
    cmake_path(IS_PREFIX fw_dir "${src_dir}" NORMALIZE in_fw)
    cmake_path(IS_PREFIX hal_sim_dir "${src_dir}" NORMALIZE in_hal_sim)
    cmake_path(IS_PREFIX sil_dir "${src_dir}" NORMALIZE in_sil)
    if(in_fw AND NOT in_hal_sim AND NOT in_sil)
      list(APPEND flight ${tgt})
    endif()
  endforeach()
  set(${out} "${flight}" PARENT_SCOPE)
endfunction()

function(marv_collect_sil_libraries out)
  marv_all_project_targets(all_targets)
  set(controls_dir "${PROJECT_SOURCE_DIR}/tests/controls")
  set(sil "")
  foreach(tgt IN LISTS all_targets)
    get_target_property(type ${tgt} TYPE)
    if(NOT type STREQUAL "SHARED_LIBRARY" AND NOT type STREQUAL "MODULE_LIBRARY")
      continue()
    endif()
    get_target_property(src_dir ${tgt} SOURCE_DIR)
    cmake_path(IS_PREFIX controls_dir "${src_dir}" NORMALIZE in_controls)
    if(NOT in_controls)
      list(APPEND sil ${tgt})
    endif()
  endforeach()
  set(${out} "${sil}" PARENT_SCOPE)
endfunction()

function(marv_write_g3_manifest file)
  cmake_parse_arguments(ARG "" "" "FLIGHT;SIL" ${ARGN})
  set(flight_entries "")
  foreach(tgt IN LISTS ARG_FLIGHT)
    list(APPEND flight_entries
      "    {\"name\": \"${tgt}\", \"archive\": \"$<TARGET_FILE:${tgt}>\", \"objects\": [\"$<JOIN:$<TARGET_OBJECTS:${tgt}>,\"$<COMMA> \">\"]}")
  endforeach()
  set(sil_entries "")
  foreach(tgt IN LISTS ARG_SIL)
    list(APPEND sil_entries "    {\"name\": \"${tgt}\", \"file\": \"$<TARGET_FILE:${tgt}>\"}")
  endforeach()
  list(JOIN flight_entries ",\n" flight_json)
  list(JOIN sil_entries ",\n" sil_json)
  file(GENERATE OUTPUT "${file}" CONTENT
"{
  \"flight_targets\": [
${flight_json}
  ],
  \"sil_libraries\": [
${sil_json}
  ]
}
")
endfunction()

# Flight and SIL manifest of this build tree, written next to compile_commands.json.
function(marv_generate_g3_manifest)
  marv_collect_flight_targets(flight_targets)
  marv_collect_sil_libraries(sil_libraries)
  marv_write_g3_manifest("${CMAKE_BINARY_DIR}/g3_manifest.json"
    FLIGHT ${flight_targets} SIL ${sil_libraries})
  message(STATUS "G3 flight targets: ${flight_targets}")
  message(STATUS "G3 SIL libraries: ${sil_libraries}")
endfunction()
