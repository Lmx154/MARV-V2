// The FC3v2 board's internal flash besides the running image (decision 0053; ground-segment unit 14): the write seal of the
// config slots (storage_flash.cpp) and the image hash (flash_image.cpp).
#pragma once

#include <array>
#include <cstdint>

#include "marv/prim/constants.hpp"

namespace marv::hal_target {

// Ends the config slot writes: hal_config_slot_write and hal_config_slot_commit (marv/hal/storage.hpp) return false from then
// on; the slots stay readable. main calls it right after store::boot, before the IMU, receiver and motor outputs start, since
// every erase and program stops the flash with interrupts disabled (a sector erase lasts milliseconds).
void config_slots_seal() noexcept;

using ImageDigest = std::array<std::uint8_t, prim::kSha256DigestBytes>;

struct ImageHash {
  ImageDigest sha256{};   // all zero = unknown
  std::uint32_t bytes = 0;  // the image's flash bytes hashed
  bool ok = false;        // false: the SHA-256 accelerator was not available, sha256 is all zero
};

// The running image's identity (core IF-1): the SHA-256 of its flash bytes, from the XIP window's start
// (__flash_binary_start) to the linker's image end (__flash_binary_end), computed on the RP2350 SHA-256 accelerator
// (pico_sha256). tools/config/image_hash.py computes the same hash from the image's ELF or .bin. Call at boot.
[[nodiscard]] ImageHash image_sha256() noexcept;

}  // namespace marv::hal_target
