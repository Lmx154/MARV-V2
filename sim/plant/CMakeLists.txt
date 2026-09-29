# marv_plant v0 (host only): rotor thrust and torque, motor/ESC lag, gravity. C ABI in include/marv_plant.h.
add_library(marv_plant STATIC src/marv_plant.cpp)
target_include_directories(marv_plant PUBLIC include)
target_link_libraries(marv_plant PRIVATE marv_prim marv_types)
marv_apply_flags(marv_plant)
