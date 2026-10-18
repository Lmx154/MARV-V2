// The config storage on the FC3v2 target until ground-segment unit 14 (internal-flash slots, boot request, reboot): no slot
// storage, no boot request and no reboot (marv/hal/storage.hpp), so store::boot finds no MARV slot and the board boots its
// compiled table with config status Ok, as before the store existed. The pending file is storage_sd.cpp (unit 13).
#include "marv/hal/storage.hpp"

namespace marv {

std::span<const std::uint8_t> hal_config_slot(ConfigSlot) noexcept { return {}; }

bool hal_config_slot_write(ConfigSlot, std::span<const std::uint8_t>) noexcept { return false; }

bool hal_config_slot_commit(ConfigSlot, std::span<const std::uint8_t>) noexcept { return false; }

bool hal_config_request_set(const ConfigHashBytes&) noexcept { return false; }

bool hal_config_request_take(ConfigHashBytes&) noexcept { return false; }

bool hal_reboot() noexcept { return false; }

}  // namespace marv
