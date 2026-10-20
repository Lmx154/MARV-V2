#include "marv/log/file_life.hpp"

#include <algorithm>

namespace marv::log {
namespace {

[[nodiscard]] constexpr bool is_header_kind(std::uint8_t k) noexcept {
  switch (static_cast<Kind>(k)) {
    case Kind::HeaderInfo:
    case Kind::HeaderKind:
    case Kind::HeaderParam:
    case Kind::HeaderField:
    case Kind::HeaderTelemetry:
    case Kind::HeaderImuRaw:
      return true;
    default:
      return false;
  }
}

struct Highest {
  std::uint32_t index = 0;
};

void note_name(void* ctx, std::string_view name) noexcept {
  auto* h = static_cast<Highest*>(ctx);
  h->index = std::max(h->index, index_of_name(name));
}

}  // namespace

FileContent classify_file(SectorSource& src, std::uint64_t sectors) noexcept {
  std::array<std::uint8_t, kSectorBytes> s{};
  std::uint64_t session = 0;
  for (std::uint64_t i = 0; i < sectors; ++i) {
    if (!src.read(i, s)) {
      return FileContent::Unreadable;
    }
    const std::span<const std::uint8_t> b{s};
    const bool valid = std::equal(kSectorMagic.begin(), kSectorMagic.end(), b.begin() + kSecOffMagic) &&
                       get<std::uint32_t>(b, kSecOffCrc) == crc32(b.first(kSecOffCrc)) &&
                       get<std::uint32_t>(b, kSecOffSeq) == i && (i == 0 || get<std::uint64_t>(b, kSecOffSession) == session);
    if (!valid) {
      return FileContent::NoData;
    }
    session = get<std::uint64_t>(b, kSecOffSession);
    const std::size_t first = get<std::uint16_t>(b, kSecOffFirst);
    if (first == kNoRecord) {
      continue;  // the tail of a record that began in an earlier sector
    }
    if (first > kNoRecord || !is_header_kind(b[kSectorHeaderBytes + first])) {
      return FileContent::Data;
    }
  }
  return FileContent::Data;  // every sector looks like header: more than any header is, so something else wrote here
}

Prepared LifecycleStorage::prepare_file(std::uint64_t bytes) noexcept {
  abandon();
  if (bytes < kSectorBytes) {
    return Prepared{StorageStatus::Error, 0};
  }
  if (bytes > largest_file_bytes()) {
    return Prepared{StorageStatus::Full, 0};
  }
  StorageStatus st = mount();
  if (st != StorageStatus::Ok) {
    return Prepared{st, 0};
  }
  Highest newest;
  st = visit_names(&note_name, &newest);
  if (st != StorageStatus::Ok) {
    return Prepared{st, 0};
  }
  if (newest.index != 0) {
    bool usable = false;
    st = open_existing(newest.index, bytes, usable);
    if (st != StorageStatus::Ok) {
      return Prepared{st, 0};
    }
    if (usable) {
      struct Source final : SectorSource {
        LifecycleStorage& owner;
        explicit Source(LifecycleStorage& o) : owner(o) {}
        bool read(std::uint64_t index, std::span<std::uint8_t> out) noexcept override { return owner.read_sector(index, out); }
      } src{*this};
      if (classify_file(src, capacity_for(bytes) / kSectorBytes) == FileContent::NoData) {
        return Prepared{StorageStatus::Ok, capacity_for(bytes)};
      }
      drop_existing();
    }
  }
  if (newest.index >= largest_index()) {
    return Prepared{StorageStatus::Full, 0};
  }
  st = create(newest.index + 1, bytes);
  if (st != StorageStatus::Ok) {
    return Prepared{st, 0};
  }
  return Prepared{StorageStatus::Ok, capacity_for(bytes)};
}

}  // namespace marv::log
