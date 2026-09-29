add_library(marv_types INTERFACE)
target_include_directories(marv_types INTERFACE include)
target_link_libraries(marv_types INTERFACE marv_prim)

# Compiles the header-only types under the target's firmware flags so the m33 build checks them.
add_library(marv_types_instantiate STATIC src/instantiate.cpp)
target_link_libraries(marv_types_instantiate PUBLIC marv_types)
marv_apply_flags(marv_types_instantiate FIRMWARE)
