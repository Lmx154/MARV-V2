// The SD card's one FatFs volume (sd_volume.hpp).
#include "sd_volume.hpp"

#include "marv/hal_target/sd_spi.hpp"

namespace marv::hal_target {
namespace {

FATFS g_fs;

}  // namespace

FRESULT sd_volume_mount() noexcept { return f_mount(&g_fs, "", 1); }

void sd_volume_unmount() noexcept {
  static_cast<void>(f_mount(nullptr, "", 0));
  sd::release();
}

const FATFS& sd_volume() noexcept { return g_fs; }

}  // namespace marv::hal_target
