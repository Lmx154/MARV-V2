# The Pico SDK for the m33 target (decision 0027). Included before project() when MARV_TARGET is m33; host presets never
# include it, so they download nothing.
#
# The SDK and TinyUSB are fetched as GitHub archives pinned by SHA-256 (the cmake/googletest.cmake pattern), so a fresh
# clone needs no submodules. SOURCE_SUBDIR names a directory that does not exist, so FetchContent_MakeAvailable
# downloads the sources and does not add_subdirectory them; the SDK is added by pico_sdk_init() after project().
#
# What the preset sets as cache variables: PICO_PLATFORM, PICO_BOARD, PICO_BOARD_HEADER_DIRS, PICO_HARD_FLOAT_ABI,
# PICO_NO_PICOTOOL. Including pico_sdk_init.cmake selects the SDK's pico_arm_cortex_m33_gcc toolchain file.
include(FetchContent)

FetchContent_Declare(pico_sdk
  URL https://github.com/raspberrypi/pico-sdk/archive/079c6f39023649b154152db30f1d781e884879bc.tar.gz
  URL_HASH SHA256=626ef3c5b364080eb3989e33d5b21f7af694e6c226c3e2154547ad447e9fe6d2
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  SOURCE_SUBDIR marv_no_cmake_here
)
FetchContent_Declare(pico_tinyusb
  URL https://github.com/hathach/tinyusb/archive/86ad6e56c1700e85f1c5678607a762cfe3aa2f47.tar.gz
  URL_HASH SHA256=9f43fd2a745e2543a42ac79b99774c24155e6ef1e0d2ae3cb16ed4ffacf5c517
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  SOURCE_SUBDIR marv_no_cmake_here
)
FetchContent_MakeAvailable(pico_sdk pico_tinyusb)

set(PICO_SDK_PATH "${pico_sdk_SOURCE_DIR}" CACHE PATH "Path to the Raspberry Pi Pico SDK" FORCE)
set(PICO_TINYUSB_PATH "${pico_tinyusb_SOURCE_DIR}" CACHE PATH "Path to TinyUSB" FORCE)

include("${PICO_SDK_PATH}/pico_sdk_init.cmake")

set(MARV_PICO_SDK_VERSION 2.3.1)
if(NOT PICO_SDK_VERSION_STRING STREQUAL MARV_PICO_SDK_VERSION)
  message(FATAL_ERROR "Pico SDK ${MARV_PICO_SDK_VERSION} is pinned (decision 0027); the fetched SDK reports "
                      "'${PICO_SDK_VERSION_STRING}'")
endif()
message(STATUS "Pico SDK ${PICO_SDK_VERSION_STRING} at ${PICO_SDK_PATH}, TinyUSB at ${PICO_TINYUSB_PATH}")

# marv_check_m33_toolchain()
#
# Called after project(): the compiler must be arm-none-eabi-gcc/g++ 13.2.1, the version of the apt package
# gcc-arm-none-eabi 15:13.2.rel1-2 in ci/Dockerfile (decision 0027 item 4).
function(marv_check_m33_toolchain)
  set(want 13.2.1)
  foreach(lang C CXX)
    get_filename_component(name "${CMAKE_${lang}_COMPILER}" NAME)
    if(NOT CMAKE_${lang}_COMPILER_ID STREQUAL "GNU" OR NOT name MATCHES "^arm-none-eabi-"
       OR NOT CMAKE_${lang}_COMPILER_VERSION VERSION_EQUAL want)
      message(FATAL_ERROR "The m33 build needs arm-none-eabi-gcc ${want} (gcc-arm-none-eabi 15:13.2.rel1-2, "
                          "ci/Dockerfile); the ${lang} compiler is ${CMAKE_${lang}_COMPILER} "
                          "(${CMAKE_${lang}_COMPILER_ID} ${CMAKE_${lang}_COMPILER_VERSION})")
    endif()
  endforeach()
  message(STATUS "m33 toolchain: ${CMAKE_CXX_COMPILER} ${CMAKE_CXX_COMPILER_VERSION}")
endfunction()
