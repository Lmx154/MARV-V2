#include "marv/hal_sim/storage.hpp"

#include <sys/stat.h>

#include <algorithm>
#include <array>
#include <climits>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <span>

#include "marv/hal/storage.hpp"

namespace marv {
namespace {

constexpr std::uint8_t kErased = std::numeric_limits<std::uint8_t>::max();
constexpr std::array<const char*, kConfigSlots> kSlotFiles{"slot_a.bin", "slot_b.bin"};
constexpr const char* kSlotTemp = "slot.tmp";
constexpr const char* kPendingFile = "pending.mcfg";
constexpr const char* kPendingTemp = "pending.mcfg.tmp";
constexpr const char* kRequestFile = "boot_request.bin";
constexpr const char* kRequestTemp = "boot_request.tmp";

using Path = std::array<char, PATH_MAX>;
using Region = std::array<std::uint8_t, kConfigSlotBytes>;

struct Storage {
  bool open = false;
  Path dir{};
  std::array<Region, kConfigSlots> slots{};
  hal_sim::StorageFault fault = hal_sim::StorageFault::None;
  bool reboot = false;
};

Storage g_storage;

[[nodiscard]] bool path_of(Path& out, const char* name) noexcept {
  const int n = std::snprintf(out.data(), out.size(), "%s/%s", g_storage.dir.data(), name);
  return n > 0 && static_cast<std::size_t>(n) < out.size();
}

// Writes `bytes` to `name` through `temp` and a rename, so a reader sees the old file or the new one.
[[nodiscard]] bool write_file(const char* name, const char* temp, std::span<const std::uint8_t> bytes) noexcept {
  Path final_path{};
  Path temp_path{};
  if (!path_of(final_path, name) || !path_of(temp_path, temp)) {
    return false;
  }
  std::FILE* f = std::fopen(temp_path.data(), "wb");
  if (f == nullptr) {
    return false;
  }
  const bool written = std::fwrite(bytes.data(), 1, bytes.size(), f) == bytes.size();
  const bool closed = std::fclose(f) == 0;
  if (!written || !closed) {
    static_cast<void>(std::remove(temp_path.data()));
    return false;
  }
  return std::rename(temp_path.data(), final_path.data()) == 0;
}

// A read of up to buf.size() bytes of a file in the run directory: whether it exists, whether the read failed, its size.
struct FileRead {
  bool present = false;
  bool error = false;
  std::size_t bytes = 0;  // the file's size
};

[[nodiscard]] FileRead read_file(const char* name, std::span<std::uint8_t> buf) noexcept {
  FileRead r;
  Path p{};
  if (!path_of(p, name)) {
    r.error = true;
    return r;
  }
  std::FILE* f = std::fopen(p.data(), "rb");
  if (f == nullptr) {
    return r;
  }
  r.present = true;
  const std::size_t got = std::fread(buf.data(), 1, buf.size(), f);
  std::size_t extra = 0;
  std::array<std::uint8_t, 1> probe{};
  while (std::fread(probe.data(), 1, probe.size(), f) == probe.size()) {
    ++extra;
  }
  r.error = std::ferror(f) != 0;
  r.bytes = got + extra;
  static_cast<void>(std::fclose(f));
  return r;
}

[[nodiscard]] bool persist_slot(std::size_t i) noexcept {
  return write_file(kSlotFiles[i], kSlotTemp, g_storage.slots[i]);
}

[[nodiscard]] hal_sim::StorageFault take_fault() noexcept {
  const hal_sim::StorageFault f = g_storage.fault;
  g_storage.fault = hal_sim::StorageFault::None;
  return f;
}

}  // namespace

std::span<const std::uint8_t> hal_config_slot(ConfigSlot slot) noexcept {
  if (!g_storage.open) {
    return {};
  }
  return g_storage.slots[static_cast<std::size_t>(slot)];
}

bool hal_config_slot_write(ConfigSlot slot, std::span<const std::uint8_t> body) noexcept {
  if (!g_storage.open || body.size() > kConfigSlotBytes) {
    return false;
  }
  const auto i = static_cast<std::size_t>(slot);
  Region& r = g_storage.slots[i];
  std::fill(r.begin(), r.end(), kErased);
  const hal_sim::StorageFault fault = take_fault();
  if (fault == hal_sim::StorageFault::Fails) {
    static_cast<void>(persist_slot(i));
    return false;
  }
  std::copy(body.begin(), body.end(), r.begin());
  if (fault == hal_sim::StorageFault::ReadBackDiffers && !body.empty()) {
    r[0] = static_cast<std::uint8_t>(~r[0]);
  }
  return persist_slot(i);
}

bool hal_config_slot_commit(ConfigSlot slot, std::span<const std::uint8_t> record) noexcept {
  if (!g_storage.open || record.size() > kConfigSlotBytes) {
    return false;
  }
  const auto i = static_cast<std::size_t>(slot);
  Region& r = g_storage.slots[i];
  const hal_sim::StorageFault fault = take_fault();
  if (fault == hal_sim::StorageFault::Fails) {
    return false;
  }
  const std::size_t start = r.size() - record.size();
  for (std::size_t k = 0; k < record.size(); ++k) {
    r[start + k] &= record[k];  // programming flash only clears bits
  }
  if (fault == hal_sim::StorageFault::ReadBackDiffers && !record.empty()) {
    r[start] = static_cast<std::uint8_t>(~r[start]);
  }
  return persist_slot(i);
}

PendingFile hal_config_pending_read(std::span<std::uint8_t> buf) noexcept {
  if (!g_storage.open) {
    return {};
  }
  const FileRead r = read_file(kPendingFile, buf);
  if (!r.present) {
    return {};
  }
  if (r.error) {
    return PendingFile{PendingRead::Error, 0};
  }
  return PendingFile{r.bytes > buf.size() ? PendingRead::TooLarge : PendingRead::Ok, r.bytes};
}

bool hal_config_pending_write(std::span<const std::uint8_t> bytes) noexcept {
  return g_storage.open && write_file(kPendingFile, kPendingTemp, bytes);
}

bool hal_config_pending_remove() noexcept {
  if (!g_storage.open) {
    return true;
  }
  Path p{};
  if (!path_of(p, kPendingFile)) {
    return false;
  }
  static_cast<void>(std::remove(p.data()));
  std::FILE* f = std::fopen(p.data(), "rb");
  if (f == nullptr) {
    return true;
  }
  static_cast<void>(std::fclose(f));
  return false;
}

bool hal_config_request_set(const ConfigHashBytes& expected) noexcept {
  return g_storage.open && write_file(kRequestFile, kRequestTemp, expected);
}

bool hal_config_request_take(ConfigHashBytes& expected) noexcept {
  if (!g_storage.open) {
    return false;
  }
  ConfigHashBytes got{};
  const FileRead r = read_file(kRequestFile, got);
  if (!r.present) {
    return false;
  }
  Path p{};
  if (path_of(p, kRequestFile)) {
    static_cast<void>(std::remove(p.data()));
  }
  if (r.error || r.bytes != got.size()) {
    return false;
  }
  expected = got;
  return true;
}

bool hal_reboot() noexcept {
  g_storage.reboot = true;
  return true;
}

namespace hal_sim {

bool storage_open(const char* run_dir) noexcept {
  storage_close();
  if (run_dir == nullptr) {
    return false;
  }
  struct stat st {};
  if (::stat(run_dir, &st) != 0 || !S_ISDIR(st.st_mode)) {
    return false;
  }
  const int n = std::snprintf(g_storage.dir.data(), g_storage.dir.size(), "%s", run_dir);
  if (n <= 0 || static_cast<std::size_t>(n) >= g_storage.dir.size()) {
    return false;
  }
  for (std::size_t i = 0; i < kConfigSlots; ++i) {
    Region& r = g_storage.slots[i];
    std::fill(r.begin(), r.end(), kErased);
    const FileRead f = read_file(kSlotFiles[i], r);
    if (f.error || f.bytes > r.size()) {
      std::fill(r.begin(), r.end(), kErased);
      return false;
    }
    std::fill(r.begin() + static_cast<std::ptrdiff_t>(f.bytes), r.end(), kErased);
  }
  g_storage.open = true;
  return true;
}

void storage_close() noexcept {
  g_storage.open = false;
  g_storage.fault = StorageFault::None;
}

void storage_inject(StorageFault fault) noexcept { g_storage.fault = fault; }

bool reboot_requested() noexcept { return g_storage.reboot; }

}  // namespace hal_sim
}  // namespace marv
