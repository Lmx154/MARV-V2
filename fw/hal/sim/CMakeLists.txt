add_library(marv_hal_sim STATIC src/hal_sim.cpp)
target_include_directories(marv_hal_sim PUBLIC include)
target_link_libraries(marv_hal_sim PUBLIC marv_hal marv_types)
marv_apply_flags(marv_hal_sim FIRMWARE)
