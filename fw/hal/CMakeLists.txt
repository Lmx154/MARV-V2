add_library(marv_hal INTERFACE)
target_include_directories(marv_hal INTERFACE include)
target_link_libraries(marv_hal INTERFACE marv_types)

# The config storage interface (include/marv/hal/storage.hpp; decision 0053, ground-segment units 4b and 14): what links it gets
# the slot size, MARV_CONFIG_SLOT_BYTES, the design-budget entry config_slot_bytes of design/budget.yaml (its rationale holds
# the rule and the computation), read here once at configure time; no other file carries its value. It is kept in the cache
# (FORCE, so a cached value never outlives the register) for the tools that read the build's slot size.
set(marv_slot_budget_file "${PROJECT_SOURCE_DIR}/design/budget.yaml")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${marv_slot_budget_file}")
file(READ "${marv_slot_budget_file}" marv_slot_budget_text)
if(NOT marv_slot_budget_text MATCHES "\nconfig_slot_bytes:[ \t]*\n[ \t]+value:[ \t]*([0-9]+)[ \t]*\n")
  message(FATAL_ERROR "config_slot_bytes has no integer value in ${marv_slot_budget_file}")
endif()
set(MARV_CONFIG_SLOT_BYTES "${CMAKE_MATCH_1}" CACHE STRING
    "Bytes of one config slot region (design/budget.yaml, config_slot_bytes; read at configure time)" FORCE)
add_library(marv_hal_storage INTERFACE)
target_link_libraries(marv_hal_storage INTERFACE marv_hal marv_prim)
target_compile_definitions(marv_hal_storage INTERFACE MARV_CONFIG_SLOT_BYTES=${MARV_CONFIG_SLOT_BYTES})

add_subdirectory(target/decode)
if(MARV_TARGET STREQUAL "m33")
  add_subdirectory(target)
endif()
