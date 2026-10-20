// The onboard logger's storage on the SD card (marv/hal_target/log_storage_sd.hpp). Background loop only.
#include "marv/hal_target/log_storage_sd.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string_view>

#include "marv/hal_target/sd_spi.hpp"
#include "marv/log/file_life.hpp"
#include "marv/prim/constants.hpp"
#include "sd_volume.hpp"

namespace marv::hal_target {
namespace {

using log::Prepared;
using log::StorageStatus;

constexpr const char* kDir = "/marv";
constexpr const char* kLogDir = "/marv/log";
constexpr std::string_view kLogDirPrefix = "/marv/log/";
constexpr std::size_t kPathBytes = kLogDirPrefix.size() + log::kNameBytes + 1;

constexpr std::uint64_t kLargestFile = std::numeric_limits<FSIZE_t>::max();

FIL g_file;    // the log file prepare reserved
FIL g_reader;  // a log file read back (sd_log_read_file)
DIR g_dir;
FILINFO g_info_entry;
bool g_mounted = false;
std::uint32_t g_reader_index = 0;  // 0: g_reader is not open
SdLogFile g_log{};

using Path = std::array<char, kPathBytes>;

[[nodiscard]] Path path_of(std::uint32_t index) noexcept {
  Path p{};
  std::size_t n = 0;
  for (const char c : kLogDirPrefix) {
    p[n++] = c;
  }
  for (const char c : log::name_of_index(index)) {
    p[n++] = c;
  }
  return p;
}

void close_reader() noexcept {
  if (g_reader_index != 0) {
    static_cast<void>(f_close(&g_reader));
    g_reader_index = 0;
  }
}

void unmount() noexcept {
  close_reader();
  g_mounted = false;
  sd_volume_unmount();
}

[[nodiscard]] Prepared failed(StorageStatus s, FRESULT r) noexcept {
  g_log.last_fatfs_error = static_cast<int>(r);
  return Prepared{s, 0};
}

// FR_DENIED from a directory or allocation call: no free entry or cluster block.
[[nodiscard]] StorageStatus status_of(FRESULT r) noexcept {
  return r == FR_DENIED ? StorageStatus::Full : StorageStatus::Error;
}

[[nodiscard]] FRESULT make_dir(const char* path) noexcept {
  const FRESULT r = f_mkdir(path);
  return r == FR_EXIST ? FR_OK : r;
}

class SdLogStorage final : public log::LifecycleStorage {
 public:
  StorageStatus write(std::span<const std::uint8_t> chunk) noexcept override {
    if (!g_log.open || chunk.empty() || chunk.size() % prim::kSdBlockBytes != 0) {
      return StorageStatus::Error;
    }
    if (g_log.cursor_bytes + chunk.size() > g_log.capacity_bytes) {
      return StorageStatus::Full;
    }
    const auto sector = static_cast<std::uint32_t>(g_log.start_sector + g_log.cursor_bytes / prim::kSdBlockBytes);
    if (!sd::write(sector, chunk)) {
      return StorageStatus::Error;
    }
    g_log.cursor_bytes += chunk.size();
    return StorageStatus::Ok;
  }

  StorageStatus close(std::uint64_t used_bytes) noexcept override {
    if (!g_log.open) {
      return StorageStatus::Error;
    }
    g_log.open = false;
    FRESULT r = f_lseek(&g_file, static_cast<FSIZE_t>(std::min(used_bytes, g_log.capacity_bytes)));
    if (r == FR_OK) {
      r = f_truncate(&g_file);
    }
    const FRESULT c = f_close(&g_file);
    if (r == FR_OK) {
      r = c;
    }
    if (r != FR_OK) {
      g_log.last_fatfs_error = static_cast<int>(r);
      return StorageStatus::Error;
    }
    return StorageStatus::Ok;
  }

  void release() noexcept override {
    g_log.open = false;
    unmount();
  }

 protected:
  void abandon() noexcept override { g_log = SdLogFile{}; }  // a file left open: its region keeps the size it has

  std::uint64_t largest_file_bytes() const noexcept override { return kLargestFile; }

  StorageStatus mount() noexcept override {
    unmount();
    FRESULT r = sd_volume_mount();
    if (r != FR_OK) {
      sd_volume_unmount();
      return failed(r == FR_NOT_READY || r == FR_NO_FILESYSTEM ? StorageStatus::NoCard : StorageStatus::Error, r).status;
    }
    g_mounted = true;
    r = make_dir(kDir);
    if (r == FR_OK) {
      r = make_dir(kLogDir);
    }
    return r == FR_OK ? StorageStatus::Ok : failed(status_of(r), r).status;
  }

  StorageStatus visit_names(log::NameVisitor visit, void* ctx) noexcept override {
    FRESULT r = f_opendir(&g_dir, kLogDir);
    if (r != FR_OK) {
      return failed(status_of(r), r).status;
    }
    for (;;) {
      r = f_readdir(&g_dir, &g_info_entry);
      if (r != FR_OK || g_info_entry.fname[0] == '\0') {
        break;
      }
      if ((g_info_entry.fattrib & AM_DIR) == 0) {
        visit(ctx, g_info_entry.fname);
      }
    }
    static_cast<void>(f_closedir(&g_dir));
    return r == FR_OK ? StorageStatus::Ok : failed(status_of(r), r).status;
  }

  // A newest file that cannot serve (it does not open, is too short, or is not one contiguous block) is left alone and a new
  // one is created: the logger still gets a file, and nothing is deleted or shortened.
  StorageStatus open_existing(std::uint32_t index, std::uint64_t bytes, bool& usable) noexcept override {
    usable = false;
    const Path path = path_of(index);
    FRESULT r = f_open(&g_file, path.data(), FA_WRITE | FA_OPEN_EXISTING);
    if (r != FR_OK) {
      g_log.last_fatfs_error = static_cast<int>(r);
      return StorageStatus::Ok;
    }
    const std::uint64_t capacity = capacity_for(bytes);
    const FATFS& fs = sd_volume();
    const auto start = static_cast<std::uint32_t>(fs.database + static_cast<LBA_t>(fs.csize) * (g_file.obj.sclust - 2));
    bool ok = f_size(&g_file) >= bytes;
    if (ok) {
      // The last sector of the region lies where one contiguous block puts it (the chain is followed to there).
      const std::uint64_t last = capacity / prim::kSdBlockBytes - 1;
      ok = f_lseek(&g_file, static_cast<FSIZE_t>(last * prim::kSdBlockBytes + 1)) == FR_OK &&
           static_cast<std::uint64_t>(fs.database) + static_cast<std::uint64_t>(fs.csize) * (g_file.clust - 2) + last % fs.csize ==
               static_cast<std::uint64_t>(start) + last;
    }
    if (!ok) {
      static_cast<void>(f_close(&g_file));
      return StorageStatus::Ok;
    }
    g_log.index = index;
    g_log.start_sector = start;
    g_log.capacity_bytes = capacity;
    g_log.cursor_bytes = 0;
    g_log.open = true;
    usable = true;
    return StorageStatus::Ok;
  }

  bool read_sector(std::uint64_t index, std::span<std::uint8_t> out) noexcept override {
    return g_log.open && sd::read(static_cast<std::uint32_t>(g_log.start_sector + index), out);
  }

  void drop_existing() noexcept override {
    static_cast<void>(f_close(&g_file));
    g_log.open = false;
  }

  StorageStatus create(std::uint32_t index, std::uint64_t bytes) noexcept override {
    const Path path = path_of(index);
    FRESULT r = f_open(&g_file, path.data(), FA_CREATE_NEW | FA_WRITE);
    if (r != FR_OK) {
      return failed(status_of(r), r).status;
    }
    r = f_expand(&g_file, static_cast<FSIZE_t>(bytes), 1);
    if (r == FR_OK) {
      r = f_sync(&g_file);
    }
    if (r != FR_OK) {
      static_cast<void>(f_close(&g_file));
      static_cast<void>(f_unlink(path.data()));  // the file this call made, never one that held a log
      return failed(status_of(r), r).status;
    }
    g_log.index = index;
    const FATFS& fs = sd_volume();
    g_log.start_sector = static_cast<std::uint32_t>(fs.database + static_cast<LBA_t>(fs.csize) * (g_file.obj.sclust - 2));
    g_log.capacity_bytes = capacity_for(bytes);
    g_log.cursor_bytes = 0;
    g_log.open = true;
    return StorageStatus::Ok;
  }
};

SdLogStorage g_storage;

[[nodiscard]] bool open_reader(std::uint32_t index) noexcept {
  if (!g_mounted || index == 0 || (g_log.open && index == g_log.index)) {
    return false;
  }
  if (g_reader_index == index) {
    return true;
  }
  close_reader();
  const Path path = path_of(index);
  if (f_open(&g_reader, path.data(), FA_READ) != FR_OK) {
    return false;
  }
  g_reader_index = index;
  return true;
}

}  // namespace

log::LogStorage& sd_log_storage() noexcept { return g_storage; }

SdLogFile sd_log_file() noexcept { return g_log; }

long sd_log_read_file(std::uint32_t index, std::uint64_t offset, std::span<std::uint8_t> out) noexcept {
  if (!open_reader(index) || offset > kLargestFile) {
    return -1;
  }
  if (f_tell(&g_reader) != offset && f_lseek(&g_reader, static_cast<FSIZE_t>(offset)) != FR_OK) {
    close_reader();
    return -1;
  }
  UINT got = 0;
  if (f_read(&g_reader, out.data(), static_cast<UINT>(out.size()), &got) != FR_OK) {
    close_reader();
    return -1;
  }
  return static_cast<long>(got);
}

long long sd_log_file_size(std::uint32_t index) noexcept {
  if (!open_reader(index)) {
    return -1;
  }
  return static_cast<long long>(f_size(&g_reader));
}

}  // namespace marv::hal_target
