if(MARV_TARGET STREQUAL "m33")
  add_library(pb2_double_promotion OBJECT EXCLUDE_FROM_ALL pb2_double_promotion.cpp)
  marv_apply_flags(pb2_double_promotion FIRMWARE)
endif()
