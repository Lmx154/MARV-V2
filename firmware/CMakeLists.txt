cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico2 CACHE STRING "Board type")

include(${CMAKE_CURRENT_LIST_DIR}/cmake/pico_sdk_import.cmake)

project(marv_fw C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

pico_sdk_init()

set(MARV_ROOT ${CMAKE_CURRENT_LIST_DIR}/..)

file(GLOB MARV_FSW_SOURCES CONFIGURE_DEPENDS ${MARV_ROOT}/fsw/src/*.cpp)
set(MARV_FW_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/src/main.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/usb_descriptors.c
    ${MARV_FSW_SOURCES}
)

add_executable(marv_fw ${MARV_FW_SOURCES})

# Warnings as errors on our sources only; the SDK sources compiled into this target are left alone.
set_source_files_properties(${MARV_FW_SOURCES} PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra;-Werror")

target_include_directories(marv_fw PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${MARV_ROOT}/fsw/include
    ${MARV_ROOT}/link/include
)

target_link_libraries(marv_fw PRIVATE pico_stdlib pico_unique_id tinyusb_device hardware_flash hardware_watchdog pico_flash)

# The USB port carries protocol bytes only: no stdio on it, and none on the UART either.
pico_enable_stdio_usb(marv_fw 0)
pico_enable_stdio_uart(marv_fw 0)

# Run from SRAM: ~80 KB of hot code thrashed the 16 KB XIP cache (on the Pico, fsw step max 1116 -> 264 us, mean 510 -> 102 us).
pico_set_binary_type(marv_fw copy_to_ram)

pico_add_extra_outputs(marv_fw)
