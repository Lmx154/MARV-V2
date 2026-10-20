#pragma once

// The SD card's one FatFs volume (drive 0) and its one FATFS object (onboard logger unit U4-2; serena memory
// quad-m4/architect-logger-u4-2026-10-20 decision 1b), shared by the config store's pending file (storage_sd.cpp) and the
// onboard logger's files (log_storage_sd.cpp). Who may mount it is decided by the arena lease (marv/store/arena.hpp, invariant
// L-8): the store at boot and while it holds the arena for an activation, the logger while it holds the lease; each unmounts
// it before it gives the arena back. Boot and background loop only, never the tick.

extern "C" {
#include "ff.h"
}

namespace marv::hal_target {

// Mounts the volume afresh (the driver initialises the card).
[[nodiscard]] FRESULT sd_volume_mount() noexcept;

// Unmounts the volume and forgets the card (sd_spi.hpp release).
void sd_volume_unmount() noexcept;

// The volume's FATFS object (its geometry while mounted).
[[nodiscard]] const FATFS& sd_volume() noexcept;

}  // namespace marv::hal_target
