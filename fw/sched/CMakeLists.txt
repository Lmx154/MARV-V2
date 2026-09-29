add_library(marv_sched STATIC src/instantiate.cpp)
target_include_directories(marv_sched PUBLIC include)
target_link_libraries(marv_sched PUBLIC marv_types marv_hal)
marv_apply_flags(marv_sched FIRMWARE)
