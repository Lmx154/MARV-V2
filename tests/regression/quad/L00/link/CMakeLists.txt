# The link codec conformance suite (ground segment unit 5; GS-3; core section 8). Host only.
# link_vectors_update rewrites vectors/cpp_frames.txt from the C++ codec; vectors/ts_frames.txt is written by the TypeScript
# suite (MARV_UPDATE_VECTORS=1 npx vitest run in gcs/).
add_executable(link_vectors_tool link_vectors_tool.cpp)
target_link_libraries(link_vectors_tool PRIVATE marv_link_codec_mavlink)
marv_apply_flags(link_vectors_tool)

add_custom_target(link_vectors_update
  COMMAND link_vectors_tool "${CMAKE_CURRENT_SOURCE_DIR}/vectors/cpp_frames.txt"
  COMMENT "Rewriting vectors/cpp_frames.txt")

add_executable(unit_link codec_conformance_test.cpp)
target_link_libraries(unit_link PRIVATE marv_link_codec_mavlink GTest::gtest_main)
target_compile_definitions(unit_link PRIVATE MARV_LINK_VECTORS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/vectors")
marv_apply_flags(unit_link)
marv_frozen_suite(unit_link)
