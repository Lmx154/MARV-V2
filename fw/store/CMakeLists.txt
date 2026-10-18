# The config store (decision 0053, ground-segment unit 4b): the .mcfg reader and writer, the boot step (the boot request,
# the A/B slots, the boot policy) and the staged copy with its commands. Built against the FC3v2 parameter set, the set
# of the flight composition (quad_fc), which is the only composition that boots through it (the m33 image and the
# marv_sil_quad_fc library). The storage itself is the HAL's (marv/hal/storage.hpp).
add_library(marv_store STATIC src/mcfg.cpp src/boot.cpp src/staging.cpp)
target_include_directories(marv_store PUBLIC include)
target_link_libraries(marv_store
  PUBLIC marv_prim marv_params marv_arming marv_hal_storage
  PRIVATE $<COMPILE_ONLY:marv_params_fc3v2_generated>)
add_dependencies(marv_store marv_params_fc3v2_generated)
marv_apply_flags(marv_store FIRMWARE)
