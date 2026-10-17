// The config storage of the board (decision 0053; ground segment GS-4, GS-11): the two internal-flash slots, the pending
// activation file (SD card) and the boot request that survives a reboot (watchdog scratch). Used only by fw/store, at boot
// (the slots are written only by the boot step, before the IMU, receiver and motor outputs start) and from the background
// loop (the pending file, the boot request, the reboot); never from the tick.
//   hal_sim: file-backed in the run directory (marv/hal_sim/storage.hpp); nothing is stored without one.
//   hal_target: a stub until ground-segment units 13 (SD) and 14 (flash slots): no slot storage, no pending file, no boot
//   request, no reboot, so the board boots its compiled table.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "marv/prim/constants.hpp"

#ifndef MARV_CONFIG_SLOT_BYTES
#error "MARV_CONFIG_SLOT_BYTES is set by fw/hal/CMakeLists.txt (marv_hal_storage)"
#endif

namespace marv {

// The size of one slot region in bytes. PLACEHOLDER (fw/hal/CMakeLists.txt, MARV_CONFIG_SLOT_BYTES): replaced by the
// slot-size design-budget entry of ground-segment unit 14.
inline constexpr std::size_t kConfigSlotBytes = MARV_CONFIG_SLOT_BYTES;

enum class ConfigSlot : std::uint8_t { A, B, Count };
inline constexpr std::size_t kConfigSlots = static_cast<std::size_t>(ConfigSlot::Count);

// A read-only view of the whole slot region (kConfigSlotBytes bytes; an erased byte reads as flash's erased value). Empty
// if the board has no slot storage. Valid until the next hal_config_slot_write or hal_config_slot_commit of that slot.
[[nodiscard]] std::span<const std::uint8_t> hal_config_slot(ConfigSlot slot) noexcept;

// Erases the slot region, then programs `body` from its start; the rest of the region, its commit area included, stays
// erased. An empty body only erases. False if the board has no slot storage, `body` is longer than the region, or the
// write fails (the region's content is then undefined).
[[nodiscard]] bool hal_config_slot_write(ConfigSlot slot, std::span<const std::uint8_t> body) noexcept;

// Programs `record` into the last record.size() bytes of the slot region, which hal_config_slot_write left erased. False if
// the board has no slot storage, the record is longer than the region, or the write fails.
[[nodiscard]] bool hal_config_slot_commit(ConfigSlot slot, std::span<const std::uint8_t> record) noexcept;

enum class PendingRead : std::uint8_t { None, Ok, TooLarge, Error };
struct PendingFile {
  PendingRead status = PendingRead::None;
  std::size_t bytes = 0;  // the file's size when Ok or TooLarge
};

// Reads the pending activation file into `buf`. None: there is no file (or no storage). TooLarge: the file is longer than
// `buf` (nothing useful is in it). Error: the read failed.
[[nodiscard]] PendingFile hal_config_pending_read(std::span<std::uint8_t> buf) noexcept;

// Replaces the pending activation file with `bytes` atomically (a reader sees the old file or the new one). False if
// there is no storage or the write fails.
[[nodiscard]] bool hal_config_pending_write(std::span<const std::uint8_t> bytes) noexcept;

// Removes the pending activation file; true if there is none afterwards.
[[nodiscard]] bool hal_config_pending_remove() noexcept;

// The boot request: the config hash (core IF-1) the next boot is asked to make active, kept across a reboot but not a
// power cycle (the watchdog scratch registers on the board). set returns false if the board cannot keep one; take reads
// and clears it, false if there is none.
using ConfigHashBytes = std::array<std::uint8_t, prim::kSha256DigestBytes>;
[[nodiscard]] bool hal_config_request_set(const ConfigHashBytes& expected) noexcept;
[[nodiscard]] bool hal_config_request_take(ConfigHashBytes& expected) noexcept;

// Reboots the board. Call only with the maintenance grant (composition::maintenance_granted). Returns only if the reboot
// is not immediate: hal_sim records the request (marv/hal_sim/storage.hpp, "reboot requested"; the host relaunches) and
// returns true; the hal_target stub has no reboot yet and returns false.
[[nodiscard]] bool hal_reboot() noexcept;

}  // namespace marv
