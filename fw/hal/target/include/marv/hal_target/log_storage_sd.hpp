#pragma once

// The onboard logger's storage on the FC3v2's SD card (unit U2 LOG-TARGET; serena memory
// quad-m4/architect-onboard-logger-2026-10-18 item 2): marv::log::LogStorage over FatFs (third_party/fatfs) and the SPI-mode
// driver (sd_spi.hpp). Background loop only, never the tick.
//
//   prepare  (disarmed) mounts the volume afresh (the driver initialises the card), creates /marv/log if needed, scans it for
//            the highest NNNNNNNN.mlog, creates the next index, reserves the asked size as ONE contiguous cluster block
//            (f_expand), syncs the directory entry and the FAT, and computes the block's first sector:
//            database + csize x (start cluster - 2) (FatFs f_expand documentation).
//   write    (armed or not) puts a whole number of sectors at the cursor with the driver's multi-block write (CMD25; one sector
//            is a single-block write), straight into that block: no FatFs call, so no FAT, directory or FSINFO write.
//   close    (disarmed) seeks to the bytes used, truncates the file there (f_truncate frees the rest of the block), syncs and
//            closes it.
// A volume that does not mount (no card, no answer, no FAT volume) is NoCard; no contiguous block of the size is Full; any
// other failure is Error. Nothing here calls hal_panic (invariant L-3). The volume stays mounted from prepare to the next
// prepare, so a file can be read back (read_log_file) while the logger is not logging.
#include <cstddef>
#include <cstdint>
#include <span>

#include "marv/log/storage.hpp"

namespace marv::hal_target {

// The storage instance (static).
[[nodiscard]] log::LogStorage& sd_log_storage() noexcept;

// What the last prepare made: the file's index (0 before any), its first sector on the card, the region's bytes and the write
// cursor; the FatFs result of the last prepare or close that failed (FR_OK = 0 if none).
struct SdLogFile {
  std::uint32_t index = 0;
  std::uint32_t start_sector = 0;
  std::uint64_t capacity_bytes = 0;
  std::uint64_t cursor_bytes = 0;
  bool open = false;
  int last_fatfs_error = 0;
};
[[nodiscard]] SdLogFile sd_log_file() noexcept;

// Reads up to out.size() bytes of /marv/log/<index>.mlog from `offset` on the mounted volume. Returns the bytes read (0 at the
// end of the file), or -1: no mounted volume, the file is the one prepare reserved, or a FatFs error. Disarmed only, and never
// while the logger logs (it uses FatFs); the bench image reads its logs back with it.
[[nodiscard]] long sd_log_read_file(std::uint32_t index, std::uint64_t offset, std::span<std::uint8_t> out) noexcept;

// The size of /marv/log/<index>.mlog, or -1 (as sd_log_read_file).
[[nodiscard]] long long sd_log_file_size(std::uint32_t index) noexcept;

}  // namespace marv::hal_target
