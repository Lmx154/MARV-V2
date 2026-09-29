# The SIL shared library exports exactly the marv_sil_* C functions and nothing else.
# Usage: cmake -DLIB=<shared library> -DNM=<nm> -P check_exports.cmake
cmake_minimum_required(VERSION 3.28)

foreach(var LIB NM)
  if(NOT DEFINED ${var} OR "${${var}}" STREQUAL "")
    message(FATAL_ERROR "check_exports.cmake: -D${var}= is required")
  endif()
endforeach()

execute_process(COMMAND "${NM}" -D --defined-only "${LIB}"
  OUTPUT_VARIABLE nm_out ERROR_VARIABLE nm_err RESULT_VARIABLE nm_rc)
if(NOT nm_rc EQUAL 0)
  message(FATAL_ERROR "nm failed (${nm_rc}) on ${LIB}: ${nm_err}")
endif()

string(REPLACE "\n" ";" nm_lines "${nm_out}")
set(exported "")
set(foreign "")
foreach(line IN LISTS nm_lines)
  string(STRIP "${line}" line)
  if(line STREQUAL "")
    continue()
  endif()
  if(NOT line MATCHES "^[0-9a-fA-F]+ [A-Za-z] ([^ ]+)$")
    message(FATAL_ERROR "unparsable nm line: '${line}'")
  endif()
  set(name "${CMAKE_MATCH_1}")
  if(name MATCHES "^marv_sil_[a-z_]+$")
    list(APPEND exported "${name}")
  else()
    list(APPEND foreign "${name}")
  endif()
endforeach()

set(required marv_sil_info_get marv_sil_init marv_sil_tick marv_sil_shutdown marv_sil_status_str
    marv_sil_error_index)
set(missing "")
foreach(fn IN LISTS required)
  if(NOT fn IN_LIST exported)
    list(APPEND missing "${fn}")
  endif()
endforeach()

if(foreign OR missing)
  message(FATAL_ERROR
    "SIL exports are wrong for ${LIB}\n  not marv_sil_*: ${foreign}\n  missing: ${missing}\n  exported: ${exported}")
endif()
message(STATUS "SIL library exports exactly: ${exported}")
