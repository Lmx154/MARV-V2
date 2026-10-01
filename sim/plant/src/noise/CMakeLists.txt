# The plant's seeded noise source (quad L6 stage (a), decision 0012): counter-mode SplitMix64 streams and normals whose
# bits do not depend on the host's libm. Host only (sim/ is never built for the target).
#   marv_musl_math    the vendored musl 1.2.5 log, sin and cos (third_party/musl, MIT, see PROVENANCE.txt), built as C
#   marv_plant_noise  the stream and normal API (noise.hpp)
# The vendored C is not ours: no -Werror, no marv_apply_flags. It is built with -ffp-contract=off (no fused multiply-add
# changes a rounding), -U__FP_FAST_FMA (log.c takes its portable branch on every target) and no -ffast-math. Its public
# symbols are renamed so they cannot meet the platform libm, and musl_shim/math.h stands in for <math.h> so the platform
# header (which declares __sin, __cos, ... with other signatures) is not read by them.
enable_language(C)

set(musl_dir "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/musl")
add_library(marv_musl_math STATIC
  "${musl_dir}/src/math/log.c" "${musl_dir}/src/math/log_data.c" "${musl_dir}/src/math/__math_divzero.c"
  "${musl_dir}/src/math/__math_invalid.c" "${musl_dir}/src/math/sin.c" "${musl_dir}/src/math/cos.c"
  "${musl_dir}/src/math/__sin.c" "${musl_dir}/src/math/__cos.c" "${musl_dir}/src/math/__rem_pio2.c"
  "${musl_dir}/src/math/__rem_pio2_large.c" "${musl_dir}/src/math/floor.c" "${musl_dir}/src/math/scalbn.c")
target_include_directories(marv_musl_math PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/musl_shim" "${musl_dir}/src/internal" "${musl_dir}/arch/generic" "${musl_dir}/src/math")
target_compile_definitions(marv_musl_math PRIVATE
  hidden=
  log=marv_musl_log sin=marv_musl_sin cos=marv_musl_cos floor=marv_musl_floor scalbn=marv_musl_scalbn)
target_compile_options(marv_musl_math PRIVATE -ffp-contract=off -U__FP_FAST_FMA -fno-fast-math -fvisibility=hidden)

add_library(marv_plant_noise STATIC noise.cpp)
target_include_directories(marv_plant_noise PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/..")
target_link_libraries(marv_plant_noise PRIVATE marv_musl_math)
marv_apply_flags(marv_plant_noise)
