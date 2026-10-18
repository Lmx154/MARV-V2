// The image hash of the FC3v2 board (marv/hal_target/flash.hpp; decision 0053, ground-segment unit 14).
#include <algorithm>
#include <cstddef>
#include <cstdint>

#include "marv/hal_target/flash.hpp"

extern "C" {
#include "pico/sha256.h"
}

// The Pico SDK linker script's image bounds (pico_standard_link section_flash_begin.incl, section_flash_end.incl).
extern "C" const std::uint8_t __flash_binary_start[];  // NOLINT(bugprone-reserved-identifier): the SDK's linker symbol
extern "C" const std::uint8_t __flash_binary_end[];    // NOLINT(bugprone-reserved-identifier): the SDK's linker symbol

namespace marv::hal_target {

ImageHash image_sha256() noexcept {
  ImageHash out;
  const auto bytes = static_cast<std::size_t>(__flash_binary_end - __flash_binary_start);
  pico_sha256_state_t state;
  // Bytes in, digest out in the standard SHA-256 byte order (FIPS 180-4); the CPU feeds the accelerator (no DMA channel).
  if (pico_sha256_try_start(&state, SHA256_BIG_ENDIAN, false) != PICO_OK) {
    return out;
  }
  pico_sha256_update(&state, __flash_binary_start, bytes);
  sha256_result_t result;
  pico_sha256_finish(&state, &result);
  std::copy_n(result.bytes, out.sha256.size(), out.sha256.begin());
  out.bytes = static_cast<std::uint32_t>(bytes);
  out.ok = true;
  return out;
}

}  // namespace marv::hal_target
