# FatFs R0.16 with ChaN's patches 1 and 2 (PROVENANCE.txt), the m33 target only (ground-segment unit 13, decision 0053).
# Vendored C, not ours: no marv_apply_flags (-Werror), no fw/ rules. ffconf.h (config/) is MARV's configuration. The
# disk I/O layer is not here: fw/hal/target provides disk_initialize, disk_read and the others (src/sd_diskio.cpp).
# The sources are not compiled with the project's warning flags; -w keeps the vendored files from adding warnings.
add_library(marv_fatfs STATIC ff.c ffunicode.c)
target_include_directories(marv_fatfs SYSTEM PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/config")
target_compile_options(marv_fatfs PRIVATE -w)
