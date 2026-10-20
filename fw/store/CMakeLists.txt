# The config store (decision 0053, ground-segment unit 4b): the .mcfg reader and writer, the boot step (the boot request,
# the A/B slots, the boot policy) and the staged copy with its commands. Built against the FC3v2 parameter set, the set
# of the flight composition (quad_fc), which is the only composition that boots through it (the m33 image and the
# marv_sil_quad_fc library). The storage itself is the HAL's (marv/hal/storage.hpp).
# The disarmed arena's lease (marv/store/arena.hpp; onboard logger unit U4-2) is its own library with no parameter set, so the
# store of every set and the onboard logger (fw/log) link the one lease.
add_library(marv_store_arena STATIC src/arena.cpp)
target_include_directories(marv_store_arena PUBLIC include)
marv_apply_flags(marv_store_arena FIRMWARE)

add_library(marv_store STATIC src/mcfg.cpp src/boot.cpp src/staging.cpp)
target_include_directories(marv_store PUBLIC include)
target_link_libraries(marv_store
  PUBLIC marv_prim marv_params marv_arming marv_hal_storage marv_store_arena
  PRIVATE $<COMPILE_ONLY:marv_params_fc3v2_generated>)
add_dependencies(marv_store marv_params_fc3v2_generated)
marv_apply_flags(marv_store FIRMWARE)

# The same store compiled against the FC3v2 set with the logger register (marv_params_fc3v2_log), for the bench log image
# (MARV_FC3V2_LOG_BENCH, fw/hal/target/CMakeLists.txt; onboard logger unit U2) and the flight image with the logger
# (MARV_FC3V2_FLIGHT_LOG, unit U4-4) only: its tables are that set's kParamCount records and its schema is that set's. Not
# defined in any other configuration.
if(MARV_FC3V2_LOG_BENCH OR MARV_FC3V2_FLIGHT_LOG)
  add_library(marv_store_log STATIC src/mcfg.cpp src/boot.cpp src/staging.cpp)
  target_include_directories(marv_store_log PUBLIC include)
  target_link_libraries(marv_store_log
    PUBLIC marv_prim marv_params marv_arming marv_hal_storage marv_store_arena
    PRIVATE $<COMPILE_ONLY:marv_params_fc3v2_log_generated>)
  add_dependencies(marv_store_log marv_params_fc3v2_log_generated)
  marv_apply_flags(marv_store_log FIRMWARE)
endif()
