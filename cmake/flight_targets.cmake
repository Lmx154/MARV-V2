# G3 target manifest (core section 7.4, row G3).
#
# marv_write_g3_manifest(<file> [FLIGHT <target>...] [SIL <target>...])
#   Writes <file> at generate time. JSON: {"flight_targets": [...], "sil_libraries": [{"name", "file", "truth_state"}]}.
#   "truth_state" is true iff the target property MARV_SIL_TRUTH_STATE (marv_add_sil_library TRUTH_STATE) is set when
#   the manifest is written; it selects the export rule of tools/ci/check_g3.py.
#   A flight entry always has "name" and "type" (the CMake TYPE property); by type it also has
#     STATIC/SHARED/MODULE_LIBRARY, EXECUTABLE  "file" (linked file), "objects"
#     OBJECT_LIBRARY                            "objects"
#     INTERFACE_LIBRARY                         "include_dirs", "system_include_dirs" (generator expressions
#                                               evaluated: INTERFACE_INCLUDE_DIRECTORIES and
#                                               INTERFACE_SYSTEM_INCLUDE_DIRECTORIES)
#   Any other type has no more keys; tools/ci/check_g3.py fails on it. tools/ci/check_g3.py reads the file.
#
# marv_collect_flight_targets(<out-var>)
#   Every target of any type defined anywhere in the project whose source directory is under fw/, except fw/hal/sim
#   and fw/sil. Derived from the directory tree, so a new fw/ target is a flight target without any list to update;
#   a type the checker cannot handle fails the checker rather than being skipped.
#
# marv_collect_sil_libraries(<out-var>)
#   Every SHARED or MODULE library defined in the project (all of them are SIL entry libraries), except the negative
#   controls under tests/regression/<product>/Lnn/controls and the Gazebo lockstep plugin sim/gz/plugin (a host plugin,
#   not a SIL entry: it links a SIL library and exports gz plugin symbols; it is not under fw/, so it is no flight
#   target).

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
  set(gz_plugin_dir "${PROJECT_SOURCE_DIR}/sim/gz/plugin")
  set(sil "")
  foreach(tgt IN LISTS all_targets)
    get_target_property(type ${tgt} TYPE)
    if(NOT type STREQUAL "SHARED_LIBRARY" AND NOT type STREQUAL "MODULE_LIBRARY")
      continue()
    endif()
    get_target_property(src_dir ${tgt} SOURCE_DIR)
    cmake_path(RELATIVE_PATH src_dir BASE_DIRECTORY "${PROJECT_SOURCE_DIR}" OUTPUT_VARIABLE rel_src_dir)
    if(rel_src_dir MATCHES "^tests/regression/[^/]+/L[0-9][0-9]/controls(/|$)")
      set(in_controls TRUE)
    else()
      set(in_controls FALSE)
    endif()
    cmake_path(IS_PREFIX gz_plugin_dir "${src_dir}" NORMALIZE in_gz_plugin)
    if(NOT in_controls AND NOT in_gz_plugin)
      list(APPEND sil ${tgt})
    endif()
  endforeach()
  set(${out} "${sil}" PARENT_SCOPE)
endfunction()

function(marv_write_g3_manifest file)
  cmake_parse_arguments(ARG "" "" "FLIGHT;SIL" ${ARGN})
  set(flight_entries "")
  foreach(tgt IN LISTS ARG_FLIGHT)
    get_target_property(type ${tgt} TYPE)
    set(objects "\"objects\": [\"$<JOIN:$<TARGET_OBJECTS:${tgt}>,\"$<COMMA> \">\"]")
    set(head "{\"name\": \"${tgt}\", \"type\": \"${type}\"")
    if(type STREQUAL "STATIC_LIBRARY" OR type STREQUAL "SHARED_LIBRARY" OR type STREQUAL "MODULE_LIBRARY"
       OR type STREQUAL "EXECUTABLE")
      set(entry "${head}, \"file\": \"$<TARGET_FILE:${tgt}>\", ${objects}}")
    elseif(type STREQUAL "OBJECT_LIBRARY")
      set(entry "${head}, ${objects}}")
    elseif(type STREQUAL "INTERFACE_LIBRARY")
      set(entry "${head}, \"include_dirs\": [\"$<JOIN:$<TARGET_PROPERTY:${tgt},INTERFACE_INCLUDE_DIRECTORIES>,\"$<COMMA> \">\"], \"system_include_dirs\": [\"$<JOIN:$<TARGET_PROPERTY:${tgt},INTERFACE_SYSTEM_INCLUDE_DIRECTORIES>,\"$<COMMA> \">\"]}")
    else()
      set(entry "${head}}")
    endif()
    list(APPEND flight_entries "    ${entry}")
  endforeach()
  set(sil_entries "")
  foreach(tgt IN LISTS ARG_SIL)
    get_target_property(truth_state ${tgt} MARV_SIL_TRUTH_STATE)
    if(truth_state)
      set(truth_json "true")
    else()
      set(truth_json "false")
    endif()
    list(APPEND sil_entries
      "    {\"name\": \"${tgt}\", \"file\": \"$<TARGET_FILE:${tgt}>\", \"truth_state\": ${truth_json}}")
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
