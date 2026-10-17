# The SIL library resolves the parameter table of the set it is built for: the size of marv::generated::kParamDefaults in
# the shared library equals the one in EXPECT (that set's generated archive) and differs from the one in OTHER (another
# set's), so a link order that took the other set's table fails. The size is read from the linked tables themselves.
# Usage: cmake -DLIB=<shared library> -DEXPECT=<archive> -DOTHER=<archive> -DNM=<nm> -P check_param_table.cmake
cmake_minimum_required(VERSION 3.28)

foreach(var LIB EXPECT OTHER NM)
  if(NOT DEFINED ${var} OR "${${var}}" STREQUAL "")
    message(FATAL_ERROR "check_param_table.cmake: -D${var}= is required")
  endif()
endforeach()

function(table_size out file)
  execute_process(COMMAND "${NM}" -S -C "${file}"
    OUTPUT_VARIABLE nm_out ERROR_VARIABLE nm_err RESULT_VARIABLE nm_rc)
  if(NOT nm_rc EQUAL 0)
    message(FATAL_ERROR "nm failed (${nm_rc}) on ${file}: ${nm_err}")
  endif()
  string(REGEX MATCHALL "[0-9a-fA-F]+ [0-9a-fA-F]+ [A-Za-z] marv::generated::kParamDefaults\n" hits "${nm_out}")
  list(LENGTH hits n)
  if(NOT n EQUAL 1)
    message(FATAL_ERROR "${file}: expected one marv::generated::kParamDefaults, found ${n}")
  endif()
  string(REGEX REPLACE "^[0-9a-fA-F]+ ([0-9a-fA-F]+) .*$" "\\1" size_hex "${hits}")
  math(EXPR size "0x${size_hex}")
  set(${out} "${size}" PARENT_SCOPE)
endfunction()

table_size(lib_size "${LIB}")
table_size(expect_size "${EXPECT}")
table_size(other_size "${OTHER}")
if(expect_size EQUAL other_size)
  message(FATAL_ERROR "the two sets' tables have the same size (${expect_size}): the check cannot tell them apart")
endif()
if(NOT lib_size EQUAL expect_size)
  message(FATAL_ERROR "${LIB}: kParamDefaults is ${lib_size} bytes, ${EXPECT} has ${expect_size} (${OTHER} has ${other_size})")
endif()
message(STATUS "${LIB}: kParamDefaults ${lib_size} bytes = ${EXPECT}")
