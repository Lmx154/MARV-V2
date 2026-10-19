// The bench log image's logger runtime (marv/hal_target/log_bench.hpp). Background loop only.
#include "marv/hal_target/log_bench.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <optional>
#include <span>
#include <string_view>

#include "marv/hal/hal.hpp"
#include "marv/hal_target/bench_stats.hpp"
#include "marv/hal_target/console.hpp"
#include "marv/hal_target/log_storage_sd.hpp"
#include "marv/hal_target/log_tick.hpp"
#include "marv/hal_target/usb.hpp"
#include "marv/prim/constants.hpp"

#ifndef MARV_FC3V2_LOG_STAGING_BYTES
#error "MARV_FC3V2_LOG_STAGING_BYTES is set by fw/hal/target/CMakeLists.txt"
#endif

namespace marv::hal_target {
namespace {

using log::LoggerState;
using log::Prepared;
using log::StatusField;
using log::StorageStatus;

static_assert(MARV_FC3V2_LOG_STAGING_BYTES % log::kSectorBytes == 0 && MARV_FC3V2_LOG_STAGING_BYTES / log::kSectorBytes >= 2,
              "the staging chunk is at least two whole sectors (logger.hpp)");
std::array<std::uint8_t, MARV_FC3V2_LOG_STAGING_BYTES> g_staging{};

// The time of each data write the logger makes (a chunk, or the shorter flush of a header or an End), since `log start`.
struct WriteTimes {
  std::uint32_t writes = 0;
  std::uint32_t failures = 0;
  std::uint32_t max_us = 0;
  std::uint64_t sum_us = 0;
  std::uint64_t bytes = 0;
  std::array<std::uint32_t, bench::kBuckets> buckets{};  // bench::bucket_of(us)
};
WriteTimes g_times{};
// The most cells the ring held at the end of any data write since `log start` (the ring fills while a write blocks, so this is
// the ring's occupancy at the end of the worst stall; the ring's own high-water mark is since boot and includes the prepare).
std::uint32_t g_backlog_post_write_max = 0;
bool g_card_off = false;
TimeUs g_prepare_us = 0;
TimeUs g_close_us = 0;

// The SD storage, timed, with the bench's simulated card removal in front of it.
class BenchStorage final : public log::LogStorage {
 public:
  Prepared prepare(std::uint64_t bytes) noexcept override {
    if (g_card_off) {
      return Prepared{StorageStatus::NoCard, 0};
    }
    const TimeUs t0 = hal_time_us();
    const Prepared p = sd_log_storage().prepare(bytes);
    g_prepare_us = hal_time_us() - t0;
    return p;
  }

  StorageStatus write(std::span<const std::uint8_t> chunk) noexcept override {
    if (g_card_off) {
      ++g_times.failures;
      return StorageStatus::Error;
    }
    const TimeUs t0 = hal_time_us();
    const StorageStatus s = sd_log_storage().write(chunk);
    const auto dt = static_cast<std::uint32_t>(hal_time_us() - t0);
    if (s != StorageStatus::Ok) {
      ++g_times.failures;
      return s;
    }
    g_backlog_post_write_max = std::max(g_backlog_post_write_max, static_cast<std::uint32_t>(log_ring().backlog_cells()));
    ++g_times.writes;
    g_times.sum_us += dt;
    g_times.bytes += chunk.size();
    g_times.max_us = std::max(g_times.max_us, dt);
    ++g_times.buckets[bench::bucket_of(dt)];
    return s;
  }

  StorageStatus close(std::uint64_t used_bytes) noexcept override {
    if (g_card_off) {
      return StorageStatus::Error;
    }
    const TimeUs t0 = hal_time_us();
    const StorageStatus s = sd_log_storage().close(used_bytes);
    g_close_us = hal_time_us() - t0;
    return s;
  }
};

BenchStorage g_storage;
std::optional<log::Logger> g_logger;
bool g_trigger = false;
LoggerState g_last_state = LoggerState::Absent;
std::uint32_t g_last_absent = 0;

// Since `log start`: the ring's counters then, the deepest backlog a pass found before draining, the longest time between
// the starts of two passes while logging (how long the ring went undrained).
log::RingStats g_ring_at_start{};
std::uint32_t g_backlog_max = 0;
TimeUs g_pass_max_us = 0;
TimeUs g_last_pass = 0;

struct Dump {
  bool active = false;
  std::uint32_t index = 0;
  std::uint64_t limit = 0;
  std::uint64_t offset = 0;
  std::uint64_t sent = 0;
  std::size_t have = 0;
  std::size_t pos = 0;
  TimeUs start = 0;
};
Dump g_dump{};
std::array<std::uint8_t, prim::kSdBlockBytes> g_dump_buf{};

std::array<char, kLineCapacity> g_line{};
std::size_t g_line_len = 0;
bool g_line_overflow = false;

[[nodiscard]] const char* state_text(LoggerState s) noexcept {
  switch (s) {
    case LoggerState::Absent:
      return "Absent";
    case LoggerState::Ready:
      return "Ready";
    case LoggerState::Logging:
      return "Logging";
    case LoggerState::PostRoll:
      return "PostRoll";
    case LoggerState::Full:
      return "Full";
    case LoggerState::Fault:
      return "Fault";
  }
  return "";
}

[[nodiscard]] std::uint32_t counter(StatusField f) noexcept {
  return g_logger->counters()[static_cast<std::size_t>(f)];
}

[[nodiscard]] bool logging(LoggerState s) noexcept { return s == LoggerState::Logging || s == LoggerState::PostRoll; }

// [lo, hi) of a histogram bucket, the inverse of bench::bucket_of.
struct Range {
  std::uint64_t lo = 0;
  std::uint64_t hi = 0;
};
[[nodiscard]] Range bucket_range(std::size_t b) noexcept {
  if (b < 2 * bench::kSub) {
    return Range{b, b + 1};
  }
  const std::size_t shift = b / bench::kSub - 1;
  const std::uint64_t top = b % bench::kSub + bench::kSub;
  return Range{top << shift, (top + 1) << shift};
}

void print_hist() noexcept {
  const double mean = g_times.writes == 0 ? 0.0 : static_cast<double>(g_times.sum_us) / static_cast<double>(g_times.writes);
  std::printf("log hist n=%lu fail=%lu bytes=%llu max_us=%lu mean_us=%.1f us:", static_cast<unsigned long>(g_times.writes),
              static_cast<unsigned long>(g_times.failures), static_cast<unsigned long long>(g_times.bytes),
              static_cast<unsigned long>(g_times.max_us), mean);
  for (std::size_t b = 0; b < g_times.buckets.size(); ++b) {
    if (g_times.buckets[b] != 0) {
      const Range r = bucket_range(b);
      std::printf(" %llu-%llu:%lu", static_cast<unsigned long long>(r.lo), static_cast<unsigned long long>(r.hi),
                  static_cast<unsigned long>(g_times.buckets[b]));
    }
  }
  std::printf("\n");
}

void print_transition(LoggerState from, LoggerState to) noexcept {
  const SdLogFile f = sd_log_file();
  std::printf("log: %s -> %s t=%llu file=%lu start_sector=%lu capacity=%llu written=%llu session=%016llx prepare_us=%llu "
              "close_us=%llu fatfs_error=%d\n",
              state_text(from), state_text(to), static_cast<unsigned long long>(hal_time_us()),
              static_cast<unsigned long>(f.index), static_cast<unsigned long>(f.start_sector),
              static_cast<unsigned long long>(f.capacity_bytes), static_cast<unsigned long long>(g_logger->bytes_written()),
              static_cast<unsigned long long>(g_logger->session_id()), static_cast<unsigned long long>(g_prepare_us),
              static_cast<unsigned long long>(g_close_us), f.last_fatfs_error);
  if (logging(from) && !logging(to)) {
    print_hist();
  }
}

void dump_step() noexcept {
  Dump& d = g_dump;
  if (!d.active) {
    return;
  }
  if (!link_usb_connected()) {
    std::printf("log dump: ABORTED index=%lu sent=%llu (the link port closed)\n", static_cast<unsigned long>(d.index),
                static_cast<unsigned long long>(d.sent));
    d.active = false;
    return;
  }
  if (d.pos == d.have) {
    if (d.offset >= d.limit) {
      std::printf("log dump: DONE index=%lu sent=%llu us=%llu\n", static_cast<unsigned long>(d.index),
                  static_cast<unsigned long long>(d.sent), static_cast<unsigned long long>(hal_time_us() - d.start));
      d.active = false;
      return;
    }
    const std::size_t want = static_cast<std::size_t>(std::min<std::uint64_t>(g_dump_buf.size(), d.limit - d.offset));
    const long n = sd_log_read_file(d.index, d.offset, std::span<std::uint8_t>{g_dump_buf}.first(want));
    if (n <= 0) {
      std::printf("log dump: %s index=%lu sent=%llu\n", n == 0 ? "END OF FILE" : "READ ERROR",
                  static_cast<unsigned long>(d.index), static_cast<unsigned long long>(d.sent));
      d.active = false;
      return;
    }
    d.have = static_cast<std::size_t>(n);
    d.pos = 0;
    d.offset += static_cast<std::uint64_t>(n);
  }
  const std::size_t sent = link_usb_send(std::span<const std::uint8_t>{g_dump_buf}.subspan(d.pos, d.have - d.pos));
  d.pos += sent;
  d.sent += sent;
}

// Decimal digits only, as the motor-test console (console.hpp).
[[nodiscard]] bool parse_u64(std::string_view s, std::uint64_t& out) noexcept {
  if (s.empty()) {
    return false;
  }
  std::uint64_t v = 0;
  for (const char c : s) {
    if (c < '0' || c > '9') {
      return false;
    }
    v = v * prim::kDecimalRadix + static_cast<std::uint64_t>(c - '0');
  }
  out = v;
  return true;
}

// The line's space-separated tokens, at most tokens.size(); returns the count (one more than fits: too many).
template <std::size_t N>
[[nodiscard]] std::size_t split(std::string_view line, std::array<std::string_view, N>& tokens) noexcept {
  std::size_t n = 0;
  std::size_t i = 0;
  while (i < line.size()) {
    while (i < line.size() && line[i] == ' ') {
      ++i;
    }
    if (i == line.size()) {
      break;
    }
    const std::size_t j = std::min(line.find(' ', i), line.size());
    if (n == N) {
      return N + 1;
    }
    tokens[n++] = line.substr(i, j - i);
    i = j;
  }
  return n;
}

// "log <verb>", then at most two arguments ("log dump <index> <bytes>").
constexpr std::size_t kVerbTokens = 2;
constexpr std::size_t kMaxTokens = kVerbTokens + 2;

void command(std::string_view line, bool armed) noexcept {
  std::array<std::string_view, kMaxTokens> t{};
  const std::size_t n = split(line, t);
  const LoggerState state = g_logger.has_value() ? g_logger->state() : LoggerState::Absent;
  if (n == kVerbTokens && t[1] == "start") {
    if (armed) {
      std::printf("log start: REFUSED: the vehicle is armed (arming starts the logger)\n");
    } else if (g_dump.active) {
      std::printf("log start: REFUSED: a dump is running\n");
    } else if (state != LoggerState::Ready) {
      std::printf("log start: REFUSED: the logger is %s, not Ready\n", state_text(state));
    } else {
      g_times = WriteTimes{};
      g_backlog_post_write_max = 0;
      g_ring_at_start = log_ring().stats();
      g_backlog_max = 0;
      g_pass_max_us = 0;
      g_last_pass = 0;
      g_trigger = true;
      std::printf("log start: bench trigger on (the logger is serviced as if armed; nothing is armed) t=%llu\n",
                  static_cast<unsigned long long>(hal_time_us()));
    }
  } else if (n == kVerbTokens && t[1] == "stop") {
    std::printf("log stop: bench trigger %s t=%llu\n", g_trigger ? "off" : "was already off",
                static_cast<unsigned long long>(hal_time_us()));
    g_trigger = false;
  } else if (n == kVerbTokens + 1 && t[1] == "card" && (t[kVerbTokens] == "off" || t[kVerbTokens] == "on")) {
    g_card_off = t[kVerbTokens] == "off";
    std::printf("log card: %s\n", g_card_off ? "off (simulated removal: prepare answers NoCard)" : "on");
  } else if (n == kVerbTokens && t[1] == "hist") {
    print_hist();
  } else if ((n == kVerbTokens + 1 || n == kMaxTokens) && t[1] == "dump") {
    std::uint64_t index = 0;
    std::uint64_t limit = 0;
    const bool ok = parse_u64(t[kVerbTokens], index) && (n < kMaxTokens || parse_u64(t[kVerbTokens + 1], limit));
    if (!ok || index == 0 || index > std::numeric_limits<std::uint32_t>::max()) {
      std::printf("log dump: REFUSED: an index (1..) and an optional byte count, decimal\n");
    } else if (armed || g_trigger || logging(state)) {
      std::printf("log dump: REFUSED: the logger is logging or the trigger is on\n");
    } else if (!link_usb_connected()) {
      std::printf("log dump: REFUSED: open the link port (interface 2) first\n");
    } else {
      const long long size = sd_log_file_size(static_cast<std::uint32_t>(index));
      if (size < 0) {
        std::printf("log dump: REFUSED: no file %llu on a mounted card (or it is the prepared one)\n",
                    static_cast<unsigned long long>(index));
      } else {
        const auto bytes = static_cast<std::uint64_t>(size);
        g_dump = Dump{};
        g_dump.active = true;
        g_dump.index = static_cast<std::uint32_t>(index);
        g_dump.limit = n == kMaxTokens ? std::min(limit, bytes) : bytes;
        g_dump.start = hal_time_us();
        std::printf("log dump: START index=%lu file_bytes=%llu bytes=%llu\n", static_cast<unsigned long>(g_dump.index),
                    static_cast<unsigned long long>(bytes), static_cast<unsigned long long>(g_dump.limit));
      }
    }
  } else {
    std::printf("log: unknown command (log start | stop | card off|on | hist | dump <index> [<bytes>])\n");
  }
}

}  // namespace

void log_bench_start(const log::LoggerConfig& config, const log::HeaderInfo& header) noexcept {
  g_logger.emplace(log_ring(), g_storage, std::span<std::uint8_t>{g_staging}, config, header);
  std::printf("log: logger started: ring %lu cells of %lu B, pre-roll %lu cells, staging %lu B, file %llu B, post-roll %llu us\n",
              static_cast<unsigned long>(log_ring().capacity_cells()), static_cast<unsigned long>(log::kCellBytes),
              static_cast<unsigned long>(config.pre_roll_cells), static_cast<unsigned long>(g_staging.size()),
              static_cast<unsigned long long>(config.file_bytes), static_cast<unsigned long long>(config.post_roll_us));
}

void log_bench_service(TimeUs now, bool armed) noexcept {
  if (!g_logger.has_value()) {
    return;
  }
  const LoggerState before = g_logger->state();
  if (logging(before)) {
    g_backlog_max = std::max(g_backlog_max, static_cast<std::uint32_t>(log_ring().backlog_cells()));
    if (g_last_pass != 0) {
      g_pass_max_us = std::max(g_pass_max_us, now - g_last_pass);
    }
    g_last_pass = now;
  } else {
    g_last_pass = 0;
  }
  g_logger->service(now, armed || g_trigger);
  const LoggerState after = g_logger->state();
  if (after != g_last_state) {
    print_transition(g_last_state, after);
    g_last_state = after;
  }
  const std::uint32_t absent = counter(StatusField::PrepareAbsent);
  if (absent != g_last_absent) {
    g_last_absent = absent;
    std::printf("log: probe found no card (prepare_absent=%lu, fatfs_error=%d) t=%llu\n", static_cast<unsigned long>(absent),
                sd_log_file().last_fatfs_error, static_cast<unsigned long long>(now));
  }
  dump_step();
}

bool log_bench_console_byte(char c, bool armed) noexcept {
  if (c != '\n') {
    if (g_line_len < g_line.size()) {
      g_line[g_line_len++] = c;
    } else {
      g_line_overflow = true;
    }
    return false;
  }
  std::string_view line{g_line.data(), g_line_len};
  const bool overflow = g_line_overflow;
  g_line_len = 0;
  g_line_overflow = false;
  if (!line.empty() && line.back() == '\r') {
    line.remove_suffix(1);
  }
  constexpr std::string_view kCmd = "log";
  if (overflow || line.substr(0, kCmd.size()) != kCmd || (line.size() > kCmd.size() && line[kCmd.size()] != ' ')) {
    return false;
  }
  command(line, armed);
  return true;
}

void log_bench_print(TimeUs now) noexcept {
  if (!g_logger.has_value()) {
    return;
  }
  const log::RingStats r = log_ring().stats();
  const SdLogFile f = sd_log_file();
  const double mean = g_times.writes == 0 ? 0.0 : static_cast<double>(g_times.sum_us) / static_cast<double>(g_times.writes);
  std::printf(
      "log state=%s trigger=%d card=%s file=%lu start_sector=%lu capacity=%llu cursor=%llu | ring pushed=%lu,%lu,%lu "
      "dropped=%lu,%lu,%lu gaps=%lu high_water=%lu backlog=%lu corrupt=%lu | session dropped=%lu,%lu,%lu backlog_max=%lu "
      "pass_max_us=%llu backlog_post_write_max=%lu | writes n=%lu fail=%lu bytes=%llu max_us=%lu mean_us=%.1f | logger chunks=%lu write_failures=%lu "
      "absent=%lu full=%lu error=%lu close_failures=%lu discarded=%lu sessions=%lu bytes=%llu | t=%llu\n",
      state_text(g_logger->state()), g_trigger ? 1 : 0, g_card_off ? "off" : "on", static_cast<unsigned long>(f.index),
      static_cast<unsigned long>(f.start_sector), static_cast<unsigned long long>(f.capacity_bytes),
      static_cast<unsigned long long>(f.cursor_bytes), static_cast<unsigned long>(r.pushed[0]),
      static_cast<unsigned long>(r.pushed[1]), static_cast<unsigned long>(r.pushed[2]),
      static_cast<unsigned long>(r.dropped[0]), static_cast<unsigned long>(r.dropped[1]),
      static_cast<unsigned long>(r.dropped[2]), static_cast<unsigned long>(r.gaps),
      static_cast<unsigned long>(r.high_water_cells), static_cast<unsigned long>(log_ring().backlog_cells()),
      static_cast<unsigned long>(r.corrupt), static_cast<unsigned long>(r.dropped[0] - g_ring_at_start.dropped[0]),
      static_cast<unsigned long>(r.dropped[1] - g_ring_at_start.dropped[1]),
      static_cast<unsigned long>(r.dropped[2] - g_ring_at_start.dropped[2]), static_cast<unsigned long>(g_backlog_max),
      static_cast<unsigned long long>(g_pass_max_us), static_cast<unsigned long>(g_backlog_post_write_max),
      static_cast<unsigned long>(g_times.writes),
      static_cast<unsigned long>(g_times.failures), static_cast<unsigned long long>(g_times.bytes),
      static_cast<unsigned long>(g_times.max_us), mean, static_cast<unsigned long>(counter(StatusField::ChunksWritten)),
      static_cast<unsigned long>(counter(StatusField::WriteFailures)),
      static_cast<unsigned long>(counter(StatusField::PrepareAbsent)),
      static_cast<unsigned long>(counter(StatusField::PrepareFull)),
      static_cast<unsigned long>(counter(StatusField::PrepareError)),
      static_cast<unsigned long>(counter(StatusField::CloseFailures)),
      static_cast<unsigned long>(counter(StatusField::DiscardedCells)),
      static_cast<unsigned long>(counter(StatusField::Sessions)),
      static_cast<unsigned long long>(g_logger->bytes_written()), static_cast<unsigned long long>(now));
}

}  // namespace marv::hal_target
