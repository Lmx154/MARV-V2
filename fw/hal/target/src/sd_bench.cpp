// The SD bench (sd_bench.hpp). Background loop only.
#include "marv/hal_target/sd_bench.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include <pico/stdio_usb.h>

extern "C" {
#include "ff.h"
}

#include "marv/hal/hal.hpp"
#include "marv/hal/storage.hpp"
#include "marv/hal_target/sd_spi.hpp"
#include "marv/prim/constants.hpp"

namespace marv::hal_target {
namespace {

constexpr std::size_t kCycles = MARV_FC3V2_SD_BENCH_CYCLES;
constexpr std::size_t kFileBytes = MARV_FC3V2_SD_BENCH_FILE_BYTES;
constexpr TimeUs kIdleUs = static_cast<TimeUs>(MARV_FC3V2_SD_BENCH_IDLE_S * prim::kMicrosecondsPerSecond);
static_assert(kCycles >= prim::kDecimalRadix, "progress is printed every tenth of the cycles");
constexpr const char* kRawPath = "/marv/bench.bin";

enum class Op : std::size_t { PendingWrite, PendingRead, Mount, RawWrite, RawSync, RawRead, Count };
constexpr std::size_t kOps = static_cast<std::size_t>(Op::Count);
constexpr std::array<const char*, kOps> kOpNames{"pending_write", "pending_read", "mount", "raw_write", "raw_sync", "raw_read"};

enum class Phase { WaitHost, Idle, Run, Done };
Phase g_phase = Phase::WaitHost;
TimeUs g_since = 0;
std::size_t g_cycle = 0;
std::size_t g_errors = 0;
std::array<std::array<std::uint32_t, kCycles>, kOps> g_times{};
std::array<std::uint8_t, kFileBytes> g_write{};
std::array<std::uint8_t, kFileBytes> g_read{};
FATFS g_fs;
FIL g_file;

void record(Op op, TimeUs t0) noexcept {
  g_times[static_cast<std::size_t>(op)][g_cycle] = static_cast<std::uint32_t>(hal_time_us() - t0);
}

void fail(const char* what, long detail) noexcept {
  ++g_errors;
  std::printf("sd_bench: ERROR cycle=%lu %s detail=%ld\n", static_cast<unsigned long>(g_cycle), what, detail);
}

const char* fs_name(BYTE t) noexcept {
  switch (t) {
    case FS_FAT12:
      return "FAT12";
    case FS_FAT16:
      return "FAT16";
    case FS_FAT32:
      return "FAT32";
    case FS_EXFAT:
      return "exFAT";
    default:
      return "unknown";
  }
}

void report_card() noexcept {
  const TimeUs t0 = hal_time_us();
  const bool init_ok = sd::init();
  const TimeUs init_us = hal_time_us() - t0;
  const sd::CardInfo info = sd::info();
  const sd::DetectLevels det = sd::detect_levels();
  std::printf("sd_bench: card_detect gpio25 pull_up=%d pull_down=%d (polarity UNVERIFIED)\n", det.with_pull_up ? 1 : 0,
              det.with_pull_down ? 1 : 0);
  std::printf("sd_bench: init %s in %lu us, type %s, %llu sectors of %lu bytes = %llu bytes (CSD)\n",
              init_ok ? "ok" : "FAILED", static_cast<unsigned long>(init_us),
              info.type == sd::CardType::Sdhc ? "SDHC/SDXC" : (info.type == sd::CardType::Sdsc ? "SDSC" : "none"),
              static_cast<unsigned long long>(info.sectors), static_cast<unsigned long>(prim::kSdBlockBytes),
              static_cast<unsigned long long>(info.sectors * prim::kSdBlockBytes));
  if (init_ok) {
    const FRESULT m = f_mount(&g_fs, "", 1);
    DWORD free_clusters = 0;
    FATFS* fsp = nullptr;
    const FRESULT g = m == FR_OK ? f_getfree("", &free_clusters, &fsp) : m;
    if (m == FR_OK && g == FR_OK && fsp != nullptr) {
      std::printf("sd_bench: mount ok, file system %s, cluster %lu sectors, %lu clusters, %lu free\n", fs_name(fsp->fs_type),
                  static_cast<unsigned long>(fsp->csize), static_cast<unsigned long>(fsp->n_fatent - 2),
                  static_cast<unsigned long>(free_clusters));
    } else {
      std::printf("sd_bench: mount FAILED mount=%d getfree=%d\n", static_cast<int>(m), static_cast<int>(g));
    }
    static_cast<void>(f_mount(nullptr, "", 0));
  }
  sd::release();
}

void fill(std::size_t cycle) noexcept {
  for (std::size_t j = 0; j < g_write.size(); ++j) {
    g_write[j] = static_cast<std::uint8_t>(j ^ (j >> prim::kBitsPerByte) ^ cycle);
  }
}

void one_cycle() noexcept {
  fill(g_cycle);
  TimeUs t0 = hal_time_us();
  const bool wrote = hal_config_pending_write(g_write);
  record(Op::PendingWrite, t0);
  if (!wrote) {
    fail("pending_write returned false", 0);
  }

  g_read.fill(0);
  t0 = hal_time_us();
  const PendingFile pf = hal_config_pending_read(g_read);
  record(Op::PendingRead, t0);
  if (pf.status != PendingRead::Ok || pf.bytes != kFileBytes) {
    fail("pending_read status/size", static_cast<long>(pf.status));
  } else if (std::memcmp(g_read.data(), g_write.data(), kFileBytes) != 0) {
    fail("pending_read content differs", 0);
  }

  t0 = hal_time_us();
  const FRESULT m = f_mount(&g_fs, "", 1);
  record(Op::Mount, t0);
  if (m != FR_OK) {
    fail("raw mount", static_cast<long>(m));
    sd::release();
    return;
  }
  UINT n = 0;
  t0 = hal_time_us();
  FRESULT r = f_open(&g_file, kRawPath, FA_CREATE_ALWAYS | FA_WRITE | FA_READ);
  if (r == FR_OK) {
    r = f_write(&g_file, g_write.data(), static_cast<UINT>(kFileBytes), &n);
  }
  record(Op::RawWrite, t0);
  if (r != FR_OK || n != kFileBytes) {
    fail("raw write", static_cast<long>(r));
  }
  t0 = hal_time_us();
  r = f_sync(&g_file);
  record(Op::RawSync, t0);
  if (r != FR_OK) {
    fail("raw sync", static_cast<long>(r));
  }
  g_read.fill(0);
  n = 0;
  t0 = hal_time_us();
  r = f_lseek(&g_file, 0);
  if (r == FR_OK) {
    r = f_read(&g_file, g_read.data(), static_cast<UINT>(kFileBytes), &n);
  }
  record(Op::RawRead, t0);
  if (r != FR_OK || n != kFileBytes) {
    fail("raw read", static_cast<long>(r));
  } else if (std::memcmp(g_read.data(), g_write.data(), kFileBytes) != 0) {
    fail("raw read content differs", 0);
  }
  static_cast<void>(f_close(&g_file));
  static_cast<void>(f_mount(nullptr, "", 0));
  sd::release();
}

void finish() noexcept {
  std::printf("sd_bench: DONE t=%llu cycles=%lu file_bytes=%lu errors=%lu\n",
              static_cast<unsigned long long>(hal_time_us()), static_cast<unsigned long>(kCycles),
              static_cast<unsigned long>(kFileBytes), static_cast<unsigned long>(g_errors));
  for (std::size_t op = 0; op < kOps; ++op) {
    std::array<std::uint32_t, kCycles>& v = g_times[op];
    const std::uint32_t mx = *std::max_element(v.begin(), v.end());
    std::sort(v.begin(), v.end());
    std::printf("sd_bench: op %s max_us=%lu median_us=%lu min_us=%lu\n", kOpNames[op], static_cast<unsigned long>(mx),
                static_cast<unsigned long>(v[kCycles / 2]), static_cast<unsigned long>(v[0]));
  }
  static_cast<void>(hal_config_pending_remove());
  if (f_mount(&g_fs, "", 1) == FR_OK) {
    static_cast<void>(f_unlink(kRawPath));
    static_cast<void>(f_mount(nullptr, "", 0));
  }
  sd::release();
}

}  // namespace

void sd_bench_step() noexcept {
  const TimeUs now = hal_time_us();
  switch (g_phase) {
    case Phase::WaitHost:
      if (stdio_usb_connected()) {
        g_since = now;
        g_phase = Phase::Idle;
        std::printf("sd_bench: host connected, idle baseline for %lu s from t=%llu\n",
                    static_cast<unsigned long>(MARV_FC3V2_SD_BENCH_IDLE_S), static_cast<unsigned long long>(now));
      }
      break;
    case Phase::Idle:
      if (now - g_since >= kIdleUs) {
        std::printf("sd_bench: START t=%llu\n", static_cast<unsigned long long>(now));
        report_card();
        g_phase = Phase::Run;
      }
      break;
    case Phase::Run:
      one_cycle();
      ++g_cycle;
      if (g_cycle % (kCycles / prim::kDecimalRadix) == 0 && g_cycle != kCycles) {
        std::printf("sd_bench: cycle %lu of %lu errors=%lu t=%llu\n", static_cast<unsigned long>(g_cycle),
                    static_cast<unsigned long>(kCycles), static_cast<unsigned long>(g_errors),
                    static_cast<unsigned long long>(hal_time_us()));
      }
      if (g_cycle == kCycles) {
        finish();
        g_phase = Phase::Done;
      }
      break;
    case Phase::Done:
      break;
  }
}

}  // namespace marv::hal_target
