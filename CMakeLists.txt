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

file(GLOB MARV_FSW_SOURCES CONFIGURE_DEPENDS fsw/src/*.cpp)
add_library(marv_fsw STATIC ${MARV_FSW_SOURCES})
target_include_directories(marv_fsw PUBLIC fsw/include)

enable_testing()
# Every tests/test_<name>.cpp is one test, linked against the protocol and the flight software.
file(GLOB MARV_TESTS CONFIGURE_DEPENDS tests/test_*.cpp)
foreach(src ${MARV_TESTS})
    get_filename_component(name ${src} NAME_WE)
    add_executable(${name} ${src})
    target_link_libraries(${name} PRIVATE marv_link marv_fsw)
    add_test(NAME ${name} COMMAND ${name})
endforeach()

find_package(gz-transport13 QUIET)
if(gz-transport13_FOUND)
    add_subdirectory(bridge)
else()
    message(STATUS "gz-transport13 not found: bridge not built")
endif()
