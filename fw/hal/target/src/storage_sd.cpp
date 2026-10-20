// The pending activation file on the FC3v2 board's SD card (marv/hal/storage.hpp; ground-segment unit 13, decision 0053).
// FatFs (third_party/fatfs) over the SPI-mode driver (sd_spi.cpp). Each call mounts the volume on entry (the driver
// initialises the card) and unmounts on exit, so nothing is cached across a card swap, and a normal boot, which never calls
// these functions without a boot request, never touches the SD. Background loop and boot only, never the tick.
//
// The file: /marv/pending.mcfg. It is written as /marv/pending.tmp, synced, the old file removed and the temp renamed, so
// a reader never sees a half-written pending.mcfg; a power loss between the remove and the rename leaves no pending file.
#include <cstddef>
#include <cstdint>

#include "marv/hal/storage.hpp"
#include "sd_volume.hpp"

namespace marv {
namespace {

constexpr const char* kDir = "/marv";
constexpr const char* kPending = "/marv/pending.mcfg";
constexpr const char* kTemp = "/marv/pending.tmp";

FIL g_file;

FRESULT open_volume() noexcept { return hal_target::sd_volume_mount(); }

void close_volume() noexcept { hal_target::sd_volume_unmount(); }

// No card, a card that does not answer in SPI mode, or no file system on it: there is no pending file to speak of.
bool no_volume(FRESULT r) noexcept { return r == FR_NOT_READY || r == FR_NO_FILESYSTEM; }

bool absent(FRESULT r) noexcept { return r == FR_NO_FILE || r == FR_NO_PATH; }

bool write_atomic(std::span<const std::uint8_t> bytes) noexcept {
  const FRESULT made = f_mkdir(kDir);
  if (made != FR_OK && made != FR_EXIST) {
    return false;
  }
  if (f_open(&g_file, kTemp, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) {
    return false;
  }
  UINT written = 0;
  bool ok = f_write(&g_file, bytes.data(), static_cast<UINT>(bytes.size()), &written) == FR_OK &&
            written == bytes.size() && f_sync(&g_file) == FR_OK;
  ok = (f_close(&g_file) == FR_OK) && ok;
  if (!ok) {
    return false;
  }
  const FRESULT removed = f_unlink(kPending);
  if (removed != FR_OK && !absent(removed)) {
    return false;
  }
  return f_rename(kTemp, kPending) == FR_OK;
}

}  // namespace

PendingFile hal_config_pending_read(std::span<std::uint8_t> buf) noexcept {
  PendingFile out{};
  const FRESULT mounted = open_volume();
  if (mounted != FR_OK) {
    out.status = no_volume(mounted) ? PendingRead::None : PendingRead::Error;
    close_volume();
    return out;
  }
  const FRESULT opened = f_open(&g_file, kPending, FA_READ);
  if (opened != FR_OK) {
    out.status = absent(opened) ? PendingRead::None : PendingRead::Error;
    close_volume();
    return out;
  }
  const std::size_t size = f_size(&g_file);
  out.bytes = size;
  if (size > buf.size()) {
    out.status = PendingRead::TooLarge;
  } else {
    UINT got = 0;
    const bool ok = f_read(&g_file, buf.data(), static_cast<UINT>(size), &got) == FR_OK && got == size;
    out.status = ok ? PendingRead::Ok : PendingRead::Error;
  }
  static_cast<void>(f_close(&g_file));
  close_volume();
  return out;
}

bool hal_config_pending_write(std::span<const std::uint8_t> bytes) noexcept {
  if (open_volume() != FR_OK) {
    close_volume();
    return false;
  }
  const bool ok = write_atomic(bytes);
  close_volume();
  return ok;
}

bool hal_config_pending_remove() noexcept {
  const FRESULT mounted = open_volume();
  if (mounted != FR_OK) {
    close_volume();
    return no_volume(mounted);
  }
  const FRESULT removed = f_unlink(kPending);
  static_cast<void>(f_unlink(kTemp));
  close_volume();
  return removed == FR_OK || absent(removed);
}

}  // namespace marv
