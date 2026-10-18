#pragma once

// The SD memory in SPI mode on SPI0 (ground-segment unit 13, decision 0053). Polled, blocking, every wait bounded by the
// timeouts of the SD Physical Layer Simplified Specification 9.10 (constants.hpp, kSd*). Background loop and boot only:
// never from the tick, never with interrupts disabled. Nothing here runs unless a caller asks: no pin is configured at boot,
// so a normal boot never touches the SD (fw/store/src/boot.cpp reads the pending file only when a boot request exists).
// SPI1 (the IMU) is never touched.
//
// The FatFs disk I/O functions (diskio.h) are implemented over this driver in sd_spi.cpp.

#include <cstddef>
#include <cstdint>
#include <span>

namespace marv::hal_target::sd {

enum class CardType : std::uint8_t { None, Sdsc, Sdhc };  // Sdhc: SDHC or SDXC (OCR CCS = 1)

struct CardInfo {
  CardType type = CardType::None;
  std::uint64_t sectors = 0;  // 512-byte sectors from the CSD; 0 if it could not be read
};

// Powers up the SPI side (initialisation clock, 74 clocks with CS high), resets the card into SPI mode (CMD0), checks the
// interface condition (CMD8), initialises it (ACMD41 until ready), reads CCS (CMD58), sets 512-byte blocks for an SDSC card,
// reads the CSD and switches to the data clock. False if there is no answer or any step fails or times out. Calling it
// again re-initialises from CMD0.
[[nodiscard]] bool init() noexcept;

// What the last successful init() found; type None before one or after release().
[[nodiscard]] CardInfo info() noexcept;

// Forgets the card (the next transfer needs init()); CS stays high.
void release() noexcept;

// `count` 512-byte sectors starting at `sector`, buf.size() == count * 512. False on any error or timeout.
[[nodiscard]] bool read(std::uint32_t sector, std::span<std::uint8_t> buf) noexcept;
[[nodiscard]] bool write(std::uint32_t sector, std::span<const std::uint8_t> buf) noexcept;

// Waits for the card to leave busy (bounded by the write timeout).
[[nodiscard]] bool sync() noexcept;

// The card-detect pin, read raw (true = high) with the pull-up and then with the pull-down enabled, so a driven pin and a
// floating one can be told apart. Polarity is UNVERIFIED: nothing gates on it.
struct DetectLevels {
  bool with_pull_up = false;
  bool with_pull_down = false;
};
[[nodiscard]] DetectLevels detect_levels() noexcept;

}  // namespace marv::hal_target::sd
