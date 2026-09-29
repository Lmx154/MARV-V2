add_library(marv_prim STATIC src/instantiate_float.cpp)
target_include_directories(marv_prim PUBLIC include)
marv_apply_flags(marv_prim FIRMWARE)
