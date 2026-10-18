add_library(marv_hal INTERFACE)
target_include_directories(marv_hal INTERFACE include)
target_link_libraries(marv_hal INTERFACE marv_types)

# The config storage interface (include/marv/hal/storage.hpp; decision 0053, ground-segment unit 4b): what links it gets
# the slot size. MARV_CONFIG_SLOT_BYTES is a PLACEHOLDER: the size of one config slot region, chosen to hold twice the
# largest generated config file (marv_params_fc3v2.mcfg, 40256 bytes at unit 4b) rounded up to a power of two; ground-segment
# unit 14 replaces it with its design-budget entry (the smallest multiple of the flash erase block >= 2x the largest config
# generated).
set(MARV_CONFIG_SLOT_BYTES 131072 CACHE STRING
    "Bytes of one config slot region (PLACEHOLDER until the slot-size design-budget entry of ground-segment unit 14)")
add_library(marv_hal_storage INTERFACE)
target_link_libraries(marv_hal_storage INTERFACE marv_hal marv_prim)
target_compile_definitions(marv_hal_storage INTERFACE MARV_CONFIG_SLOT_BYTES=${MARV_CONFIG_SLOT_BYTES})

add_subdirectory(target/decode)
if(MARV_TARGET STREQUAL "m33")
  add_subdirectory(target)
endif()
