// The config storage on the FC3v2 board (marv/hal/storage.hpp; decision 0053, ground-segment unit 14): the two config slots
// in the internal flash, the boot request and the reboot. The pending file is storage_sd.cpp (unit 13).
//
// Slots: kConfigSlots regions of kConfigSlotBytes (design/budget.yaml config_slot_bytes) at the top of the flash, slot A first
// and slot B ending at the flash's end (PICO_FLASH_SIZE_BYTES, the board header). Every image ends below slot A: the m33
// post-link check (cmake/flash_layout.cmake) fails the build otherwise, so flashing an image never erases a slot.
// hal_config_slot reads a slot through the XIP window. Writes run only in the config store's boot step, before the IMU,
// receiver and motor outputs start (store.hpp); config_slots_seal (main.cpp, right after store::boot) refuses every later
// one. Each sector erase and each page program is one call of the SDK's flash driver (hardware/flash.h: it leaves XIP,
// erases or programs, flushes the XIP cache and re-enters XIP) with interrupts disabled, as the driver requires; the image
// runs from SRAM (copy_to_ram) and what is programmed is copied to an SRAM page first, so nothing reads the flash
// meanwhile. Every write is read back: the erase must leave the region erased, the programmed bytes must read back equal.
//
// Boot request: the config hash (32 bytes) in the eight POWMAN SCRATCH registers (RP2350 datasheet section 6.4). They keep
// their contents over the watchdog reboot of hal_reboot and every other reset short of power-on, brown-out or the RUN pin,
// which clear them (datasheet section 7.3.1, table 528, "AON Scratch"); they are not password protected (section 6.4: only
// offsets up to 0xac are). The watchdog's SCRATCH registers cannot hold the hash: the bootrom reads SCRATCH4..7 as its boot
// vector and SCRATCH2..3 as its reboot parameters (section 5.2.4), and the SDK's watchdog_reboot clears SCRATCH4. Nothing
// else in this image uses the POWMAN SCRATCH registers: the SDK's only user is pico_low_power, which the image does not link,
// and the bootrom's POWMAN boot vector is the separate BOOT0..3 registers (section 5.2.3). All zero (their reset value) is no
// request.
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <span>

#include "marv/hal/storage.hpp"
#include "marv/hal_target/flash.hpp"

#include "hardware/flash.h"
#include "hardware/regs/addressmap.h"
#include "hardware/structs/powman.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"

namespace marv {
namespace {

static_assert(kConfigSlotBytes % FLASH_SECTOR_SIZE == 0, "a config slot is a whole number of erase sectors");
static_assert(kConfigSlots * kConfigSlotBytes <= PICO_FLASH_SIZE_BYTES, "the config slots fit the flash");

constexpr std::uint8_t kErased = std::numeric_limits<std::uint8_t>::max();
constexpr std::size_t kSlotsOffset = PICO_FLASH_SIZE_BYTES - kConfigSlots * kConfigSlotBytes;
constexpr std::size_t kWordBytes = sizeof(std::uint32_t);
constexpr std::size_t kRequestWords = sizeof(ConfigHashBytes) / kWordBytes;
static_assert(kRequestWords * kWordBytes == sizeof(ConfigHashBytes), "the config hash is whole words");
static_assert(sizeof(powman_hw_t::scratch) == sizeof(ConfigHashBytes), "the config hash fills the POWMAN SCRATCH registers");

bool g_sealed = false;
alignas(kWordBytes) std::array<std::uint8_t, FLASH_PAGE_SIZE> g_page{};

[[nodiscard]] std::size_t slot_offset(ConfigSlot slot) noexcept {
  return kSlotsOffset + static_cast<std::size_t>(slot) * kConfigSlotBytes;
}

[[nodiscard]] std::span<const std::uint8_t> region(ConfigSlot slot) noexcept {
  // The XIP window maps flash offset 0 at XIP_BASE (RP2350 datasheet section 2.2).
  return {reinterpret_cast<const std::uint8_t*>(XIP_BASE + slot_offset(slot)), kConfigSlotBytes};
}

[[nodiscard]] bool erase(ConfigSlot slot) noexcept {
  const std::size_t base = slot_offset(slot);
  for (std::size_t at = 0; at < kConfigSlotBytes; at += FLASH_SECTOR_SIZE) {
    const std::uint32_t irq = save_and_disable_interrupts();
    flash_range_erase(static_cast<std::uint32_t>(base + at), FLASH_SECTOR_SIZE);
    restore_interrupts(irq);
  }
  const std::span<const std::uint8_t> r = region(slot);
  return std::all_of(r.begin(), r.end(), [](std::uint8_t b) { return b == kErased; });
}

// Programs `bytes` at `offset` into the slot (erased there before): page by page, each page's other bytes left erased.
[[nodiscard]] bool program(ConfigSlot slot, std::size_t offset, std::span<const std::uint8_t> bytes) noexcept {
  const std::size_t base = slot_offset(slot);
  const std::size_t first = offset - offset % FLASH_PAGE_SIZE;
  for (std::size_t page = first; page < offset + bytes.size(); page += FLASH_PAGE_SIZE) {
    std::fill(g_page.begin(), g_page.end(), kErased);
    const std::size_t from = std::max(page, offset);
    const std::size_t to = std::min(page + FLASH_PAGE_SIZE, offset + bytes.size());
    std::copy(bytes.begin() + static_cast<std::ptrdiff_t>(from - offset),
              bytes.begin() + static_cast<std::ptrdiff_t>(to - offset), g_page.begin() + static_cast<std::ptrdiff_t>(from - page));
    const std::uint32_t irq = save_and_disable_interrupts();
    flash_range_program(static_cast<std::uint32_t>(base + page), g_page.data(), FLASH_PAGE_SIZE);
    restore_interrupts(irq);
  }
  const std::span<const std::uint8_t> written = region(slot).subspan(offset, bytes.size());
  return std::equal(bytes.begin(), bytes.end(), written.begin());
}

}  // namespace

std::span<const std::uint8_t> hal_config_slot(ConfigSlot slot) noexcept { return region(slot); }

bool hal_config_slot_write(ConfigSlot slot, std::span<const std::uint8_t> body) noexcept {
  if (g_sealed || slot >= ConfigSlot::Count || body.size() > kConfigSlotBytes) {
    return false;
  }
  return erase(slot) && program(slot, 0, body);
}

bool hal_config_slot_commit(ConfigSlot slot, std::span<const std::uint8_t> record) noexcept {
  if (g_sealed || slot >= ConfigSlot::Count || record.size() > kConfigSlotBytes) {
    return false;
  }
  return program(slot, kConfigSlotBytes - record.size(), record);
}

bool hal_config_request_set(const ConfigHashBytes& expected) noexcept {
  std::array<std::uint32_t, kRequestWords> words{};
  std::memcpy(words.data(), expected.data(), expected.size());
  for (std::size_t i = 0; i < kRequestWords; ++i) {
    powman_hw->scratch[i] = words[i];
  }
  for (std::size_t i = 0; i < kRequestWords; ++i) {
    if (powman_hw->scratch[i] != words[i]) {
      return false;
    }
  }
  return true;
}

bool hal_config_request_take(ConfigHashBytes& expected) noexcept {
  std::array<std::uint32_t, kRequestWords> words{};
  bool any = false;
  for (std::size_t i = 0; i < kRequestWords; ++i) {
    words[i] = powman_hw->scratch[i];
    any = any || words[i] != 0;
    powman_hw->scratch[i] = 0;
  }
  if (!any) {
    return false;
  }
  std::memcpy(expected.data(), words.data(), expected.size());
  return true;
}

bool hal_reboot() noexcept {
  // A watchdog reboot into the normal flash boot (no boot vector, at once): every domain but the oscillators is reset, the
  // POWMAN SCRATCH registers (the boot request) are kept.
  watchdog_reboot(0, 0, 0);
  for (;;) {
    tight_loop_contents();
  }
}

namespace hal_target {

void config_slots_seal() noexcept { g_sealed = true; }

}  // namespace hal_target
}  // namespace marv
