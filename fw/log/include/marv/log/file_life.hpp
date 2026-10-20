// The log file's life across boots (onboard logger unit U4-5; architect ruling 2026-10-20, item 5; invariant L-10).
//
//   prepare reuses, in place, the NEWEST file when it holds no data sector of its own session: a boot that never arms (or a
//   Ready logger that yields) leaves a header-only file, and the next prepare writes a new header into it instead of creating
//   another 1 GiB file. Otherwise it creates the next index. Nothing is ever deleted to make room: a card with no room for the
//   next file is the state Full (a card with room for none but a reusable newest file still logs).
//
//   L-10: a file that holds a data sector of its own session is never reused, shortened or deleted.
//
// How "holds no data sector of its own session" is decided, without a long scan and without a format field. The header is
// written while disarmed (SectorFramer, flushed, so it ends on a sector boundary); the first record ever written after it is the
// Status record of start_logging, which therefore begins at offset 0 of the first data sector. Sectors are written in order, so
// the file's own-session sectors form a run from sector 0. classify_file walks that run (the header: tens of sectors, not the
// file) and stops at the first sector that is not valid for the file's session (no data: it is the end of the header, or a stale
// sector of an earlier file) or at the first sector whose first record is not a header kind (data). A card read error is
// Unreadable, which is never reused.
// INFERRED (no committed test on a card): a power cut during a multi-sector write loses the tail of the write, not its head.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

#include "marv/log/format.hpp"
#include "marv/log/storage.hpp"
#include "marv/prim/constants.hpp"

namespace marv::log {

// ---- file names: NNNNNNNN.mlog, the index in decimal, zero-padded to the pattern's width ------------------------------------
inline constexpr std::string_view kIndexPattern = "NNNNNNNN";
inline constexpr std::string_view kExtension = ".mlog";
inline constexpr std::size_t kNameBytes = kIndexPattern.size() + kExtension.size();

[[nodiscard]] constexpr std::uint32_t largest_index() noexcept {
  std::uint64_t m = 1;
  for (std::size_t i = 0; i < kIndexPattern.size(); ++i) {
    m *= prim::kDecimalRadix;
  }
  return static_cast<std::uint32_t>(m - 1);
}

// The name of file `index` followed by a NUL.
[[nodiscard]] constexpr std::array<char, kNameBytes + 1> name_of_index(std::uint32_t index) noexcept {
  std::array<char, kNameBytes + 1> p{};
  for (std::size_t i = kIndexPattern.size(); i-- > 0;) {
    p[i] = static_cast<char>('0' + index % prim::kDecimalRadix);
    index /= prim::kDecimalRadix;
  }
  std::size_t n = kIndexPattern.size();
  for (const char c : kExtension) {
    p[n++] = c;
  }
  return p;
}

// The index of a directory entry named NNNNNNNN.mlog, or 0 for any other name.
[[nodiscard]] constexpr std::uint32_t index_of_name(std::string_view name) noexcept {
  if (name.size() != kNameBytes || name.substr(kIndexPattern.size()) != kExtension) {
    return 0;
  }
  std::uint32_t v = 0;
  for (std::size_t i = 0; i < kIndexPattern.size(); ++i) {
    const char c = name[i];
    if (c < '0' || c > '9') {
      return 0;
    }
    v = v * prim::kDecimalRadix + static_cast<std::uint32_t>(c - '0');
  }
  return v;
}

// ---- the content rule (L-10) -------------------------------------------------------------------------------------------
enum class FileContent : std::uint8_t {
  NoData,      // no valid sector of the file's session past its header: the file may be reused
  Data,        // a data sector of its session: never reused, shortened or deleted
  Unreadable   // the card failed a read: treated as Data
};

class SectorSource {
 public:
  SectorSource() = default;
  SectorSource(const SectorSource&) = delete;
  SectorSource& operator=(const SectorSource&) = delete;
  virtual ~SectorSource() = default;
  // Sector `index` of the file into `out` (kSectorBytes). False: the read failed.
  [[nodiscard]] virtual bool read(std::uint64_t index, std::span<std::uint8_t> out) noexcept = 0;
};

// The first `sectors` sectors of a file, read from `src`.
[[nodiscard]] FileContent classify_file(SectorSource& src, std::uint64_t sectors) noexcept;

// ---- the lifecycle ------------------------------------------------------------------------------------------------------
using NameVisitor = void (*)(void* ctx, std::string_view name) noexcept;

// prepare, over a volume's operations. The volume (the SD card on the board) implements the hooks and write/close/release of
// LogStorage; a hook that returns Ok on open_existing (usable) or create leaves the file open with the write cursor at its start.
class LifecycleStorage : public LogStorage {
 public:
  // Inline, so the class has no out-of-line virtual function (no key function): the firmware library is built without RTTI and
  // the host tests that derive from this class are not.
  [[nodiscard]] Prepared prepare(std::uint64_t bytes) noexcept final { return prepare_file(bytes); }

 protected:
  [[nodiscard]] Prepared prepare_file(std::uint64_t bytes) noexcept;

  // A file left open is abandoned. Called first by prepare.
  virtual void abandon() noexcept = 0;
  // The largest file the volume can hold; asked a file of more is Full.
  [[nodiscard]] virtual std::uint64_t largest_file_bytes() const noexcept = 0;
  // Mounts the volume afresh and makes the log directory. NoCard: no card or no volume; Full: no room for the directory.
  [[nodiscard]] virtual StorageStatus mount() noexcept = 0;
  // Calls `visit` with the name of every file (not directory) of the log directory.
  [[nodiscard]] virtual StorageStatus visit_names(NameVisitor visit, void* ctx) noexcept = 0;
  // Opens file `index` for writing. usable: it is at least `bytes` long and contiguous, and is left open; otherwise it is left
  // closed and unchanged.
  [[nodiscard]] virtual StorageStatus open_existing(std::uint32_t index, std::uint64_t bytes, bool& usable) noexcept = 0;
  // Sector `index` of the file open_existing opened, straight from the card.
  [[nodiscard]] virtual bool read_sector(std::uint64_t index, std::span<std::uint8_t> out) noexcept = 0;
  // Closes the file open_existing opened, leaving it unchanged.
  virtual void drop_existing() noexcept = 0;
  // Creates file `index` as one contiguous region of `bytes` and leaves it open. Full: no room.
  [[nodiscard]] virtual StorageStatus create(std::uint32_t index, std::uint64_t bytes) noexcept = 0;
  // The region's capacity for a request of `bytes`.
  [[nodiscard]] static std::uint64_t capacity_for(std::uint64_t bytes) noexcept { return bytes - bytes % kSectorBytes; }
};

}  // namespace marv::log
