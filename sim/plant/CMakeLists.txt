# marv_plant v0 (host only): rotor thrust and torque, motor/ESC lag, gravity. C ABI in include/marv_plant.h.
add_library(marv_plant STATIC src/marv_plant.cpp)
target_include_directories(marv_plant PUBLIC include)
target_link_libraries(marv_plant PRIVATE marv_prim marv_types)
marv_apply_flags(marv_plant)

# The plant configuration header, the plugin configuration and the SDF come from the product card of the parameter set
# (MARV_VEHICLE_CARD, defined in fw/params, which is processed first).
include(${PROJECT_SOURCE_DIR}/cmake/marv_card_gen.cmake)
marv_add_plant_card_config(marv_plant_card CARD "${MARV_VEHICLE_CARD}")
marv_add_plant_card_sdf(marv_plant_card_sdf CARD "${MARV_VEHICLE_CARD}")
