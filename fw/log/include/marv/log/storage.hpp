// The logger's view of the card: three fallible, bounded operations. The logger calls them from the background loop only,
// never from the tick, and never lets a failure panic (invariant L-3): a failure is a state and a counter.
//
// Invariant L-9: prepare and close are METADATA operations (FAT, directory, allocation); the logger calls them only while the
// vehicle is disarmed. While it logs, the only call is write, which puts whole sectors into the region prepare reserved.
//
// Every call returns within the HAL's own timeout (the SD bus timeouts, constants.hpp kSd*); an implementation that cannot
// bound a call is a defect in that implementation, not something the logger can fix.
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

namespace marv::log {

enum class StorageStatus : std::uint8_t {
  Ok,
  NoCard,  // no card, or it did not answer / mount
  Full,    // the card has no room for the file, or the file's region is used up
  Error    // the card answered and failed (init, allocation, write, sync)
};

struct Prepared {
  StorageStatus status = StorageStatus::NoCard;
  std::uint64_t capacity_bytes = 0;  // with Ok: the contiguous region now reserved for the next log file; a multiple of the sector size
};

class LogStorage {
 public:
  LogStorage() = default;
  LogStorage(const LogStorage&) = delete;
  LogStorage& operator=(const LogStorage&) = delete;
  virtual ~LogStorage() = default;

  // Disarmed only. Mounts if needed, creates the next log file of at least `bytes` contiguous bytes and positions the write
  // cursor at its start. A file that was open and not closed is abandoned.
  [[nodiscard]] virtual Prepared prepare(std::uint64_t bytes) noexcept = 0;

  // Appends `chunk` (a whole number of sectors) at the write cursor of the prepared region. Data only; no metadata.
  [[nodiscard]] virtual StorageStatus write(std::span<const std::uint8_t> chunk) noexcept = 0;

  // Disarmed only. Truncates the file to `used_bytes`, syncs and closes it.
  [[nodiscard]] virtual StorageStatus close(std::uint64_t used_bytes) noexcept = 0;

  // Disarmed only, with no file open: gives the card back (unmounts the volume), so the other user of the volume, the config
  // store's pending file, may mount it (the arena lease, invariant L-8, marv/store/arena.hpp); nothing is written until the next
  // prepare, which mounts it again. A storage that holds no volume of its own does nothing.
  virtual void release() noexcept {}
};

}  // namespace marv::log
