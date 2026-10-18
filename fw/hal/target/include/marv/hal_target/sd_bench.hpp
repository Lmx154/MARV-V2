#pragma once

// The SD bench (MARV_FC3V2_SD_BENCH, bench only, default OFF; fw/hal/target/CMakeLists.txt; ground-segment unit 13): in the
// background loop, once the console host has been connected for MARV_FC3V2_SD_BENCH_IDLE_S seconds (the idle baseline of the
// IMU spacing, bench_stats.hpp), it mounts the card and prints its type, size and file system, then runs
// MARV_FC3V2_SD_BENCH_CYCLES cycles of: hal_config_pending_write, hal_config_pending_read and compare (a buffer of
// MARV_FC3V2_SD_BENCH_FILE_BYTES bytes), then a raw FatFs mount, f_write, f_sync, f_read and compare. Every operation is
// timed with hal_time_us; errors are counted. One cycle runs per call, so the once-per-second console lines continue between
// cycles. It prints the largest and median time of each operation at the end and removes its files.
namespace marv::hal_target {

void sd_bench_step() noexcept;

}  // namespace marv::hal_target
