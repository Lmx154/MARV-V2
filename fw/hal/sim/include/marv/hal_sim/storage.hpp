// hal_sim's config storage (marv/hal/storage.hpp; decision 0053 Q4: "file-backed SD and slots in the run directory"). With no
// run directory open there is no storage: no slot, no pending file, no boot request, so the SIL boots its compiled table.
//
// Files in the run directory:
//   slot_a.bin, slot_b.bin  the two slot regions, kConfigSlotBytes each; a missing file, or the bytes past a shorter
//                           file, read as erased (0xFF, as flash). Rewritten whole on every slot write.
//   pending.mcfg            the pending activation file; written through pending.mcfg.tmp and a rename (atomic).
//   boot_request.bin        the boot request, the 32-byte config hash; removed when taken. It stands for the board's
//                           watchdog scratch registers, which survive the reboot (the host relaunches the SIL).
#pragma once

#include <cstdint>

namespace marv::hal_sim {

// Opens the config storage in `run_dir` (an existing directory) and loads the slots. False, with no storage open, if
// the directory cannot be used or a slot file is longer than kConfigSlotBytes or unreadable.
[[nodiscard]] bool storage_open(const char* run_dir) noexcept;

// Closes the storage: no storage from then on (the slots read empty). The files stay.
void storage_close() noexcept;

// Test injection for the next slot write or commit only: Fails makes it return false (the region is left erased);
// ReadBackDiffers makes it return true with one byte of what it wrote changed.
enum class StorageFault : std::uint8_t { None, Fails, ReadBackDiffers };
void storage_inject(StorageFault fault) noexcept;

// True once hal_reboot was called ("reboot requested": the host relaunches the SIL).
[[nodiscard]] bool reboot_requested() noexcept;

}  // namespace marv::hal_sim
