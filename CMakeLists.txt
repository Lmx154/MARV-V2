# Host build: the flight software, the wire protocol, their tests, and (when Gazebo is installed) the
# bridge. The Pico firmware is a separate build: cmake -S firmware -B build/fw.
cmake_minimum_required(VERSION 3.20)
project(marv LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

add_library(marv_link INTERFACE)
target_include_directories(marv_link INTERFACE link/include fsw/include)

add_library(marv_fsw STATIC fsw/src/fsw.cpp)
target_include_directories(marv_fsw PUBLIC fsw/include)

enable_testing()
add_executable(test_protocol tests/test_protocol.cpp)
target_link_libraries(test_protocol PRIVATE marv_link)
add_test(NAME protocol COMMAND test_protocol)

find_package(gz-transport13 QUIET)
if(gz-transport13_FOUND)
    add_subdirectory(bridge)
else()
    message(STATUS "gz-transport13 not found: bridge not built")
endif()
