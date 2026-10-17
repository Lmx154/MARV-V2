// The bench HIL image's link and tick (marv/hal_target/hil.hpp, format in hil_format.hpp; HIL stage S3a, decision 0046).
//
// Receive, as the S1 link (hil_link.cpp): the USB CDC read goes through the SDK driver's in_chars (stdio_usb.c); the bytes
// are appended to a buffer and parsed from its start: a magic, a legal count for that magic, the whole record, the CRC. A
// mismatch drops one byte and rescans, so a corrupt record costs its bytes and the stream resyncs on the next magic. The
// buffer holds two of the longest record, so after a parse (which leaves less than one record) a read always has the
// longest record's room. A record completed by a read is stamped with the board clock read once after the read returns.
//
// ManualControl: a valid record is stamped with that read time (t_us when frames is not 0, and link_t_us), as the receiver
// stamps a frame with hal_time_us() (receiver.cpp), and published through manual_control_publish. This loop is its one
// writer and runs below the tick, the one reader (manual_control.hpp). A record that breaks the ManualControl contract
// (manual_control.hpp: sticks and channels in [-1, 1], throttle and link quality in [0, 1], all finite; frames 0 means
// sticks, throttle and channels 0) is counted and not published.
#include "marv/hal_target/hil.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <climits>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

#include <pico/stdio/driver.h>
#include <pico/stdio_usb.h>

#include "marv/hal/hal.hpp"
#include "marv/hal_target/hil_format.hpp"
#include "marv/hal_target/manual_control.hpp"
#include "marv/types/imu_sample.hpp"
#include "marv/types/manual_control.hpp"

#ifndef MARV_FC3V2_HIL_LINK_MAX_BATCH
#error "MARV_FC3V2_HIL_LINK_MAX_BATCH is set by fw/hal/target/CMakeLists.txt"
#endif

namespace marv::hal_target {
namespace {

namespace fmt = hil;

constexpr std::size_t kMaxBatch = MARV_FC3V2_HIL_LINK_MAX_BATCH;
static_assert(kMaxBatch >= 1);
constexpr std::size_t kMaxRecord = std::max(fmt::record_bytes(kMaxBatch, fmt::kSampleBytes),
                                            fmt::record_bytes(1, fmt::kManualBytes));

// Background loop only.
struct Counters {
  std::uint32_t sample_records = 0;
  std::uint32_t samples = 0;
  std::uint32_t manual_records = 0;
  std::uint32_t manual_refused = 0;
  std::uint32_t crc_failures = 0;
  std::uint32_t bad_headers = 0;
  std::uint32_t seq_gaps = 0;
  std::uint32_t seq_lost = 0;
  std::uint32_t tick_gaps = 0;
  std::uint32_t bytes_dropped = 0;
  std::uint32_t rx_bytes = 0;
};

HilConfig g_cfg{};
Counters g_c{};
std::array<std::uint8_t, 2 * kMaxRecord> g_rx{};
std::size_t g_rx_n = 0;
bool g_have_seq = false;
std::uint32_t g_down_seq = 0;
bool g_have_tick = false;
std::uint64_t g_last_tick = 0;

// Written by the background loop, read in the tick (hal_imu_deadline_misses).
std::atomic<std::uint32_t> g_deadline_misses{0};

std::uint32_t get_u32(const std::uint8_t* p) noexcept {
  std::uint32_t v = 0;
  for (std::size_t i = 0; i < sizeof(v); ++i) {
    v |= static_cast<std::uint32_t>(p[i]) << (CHAR_BIT * i);
  }
  return v;
}

std::uint64_t get_u64(const std::uint8_t* p) noexcept {
  return static_cast<std::uint64_t>(get_u32(p)) |
         (static_cast<std::uint64_t>(get_u32(p + sizeof(std::uint32_t))) << (CHAR_BIT * sizeof(std::uint32_t)));
}

float get_f32(const std::uint8_t* p) noexcept { return std::bit_cast<float>(get_u32(p)); }

bool magic_at(const std::uint8_t* p, const std::array<std::uint8_t, fmt::base::kMagicBytes>& m) noexcept {
  return std::memcmp(p, m.data(), m.size()) == 0;
}

bool in_range(float v, float lo) noexcept { return std::isfinite(v) && v >= lo && v <= 1.0F; }

bool manual_valid(const ManualControl& m) noexcept {
  bool ok = in_range(m.roll, -1.0F) && in_range(m.pitch, -1.0F) && in_range(m.yaw, -1.0F) &&
            in_range(m.throttle, 0.0F) && in_range(m.link_quality, 0.0F);
  bool all_zero = m.roll == 0.0F && m.pitch == 0.0F && m.yaw == 0.0F && m.throttle == 0.0F;
  for (const float c : m.channels) {
    ok = ok && in_range(c, -1.0F);
    all_zero = all_zero && c == 0.0F;
  }
  return ok && (m.frames != 0 || all_zero);
}

// The downlink sequence check over both record kinds.
void sequence(const std::uint8_t* p) noexcept {
  const std::uint32_t seq = get_u32(p + fmt::base::kMagicBytes);
  if (g_have_seq && seq != g_down_seq) {
    ++g_c.seq_gaps;
    g_c.seq_lost += seq - g_down_seq;
  }
  g_have_seq = true;
  g_down_seq = seq + 1;
}

void accept_manual(const std::uint8_t* p, TimeUs rx_us) noexcept {
  const std::uint8_t* q = p + fmt::base::kHeaderBytes;
  ManualControl m{};
  m.frames = get_u32(q);
  q += sizeof(m.frames);
  for (float* f : {&m.roll, &m.pitch, &m.yaw, &m.throttle}) {
    *f = get_f32(q);
    q += sizeof(float);
  }
  for (float& c : m.channels) {
    c = get_f32(q);
    q += sizeof(float);
  }
  m.link_quality = get_f32(q);
  if (!manual_valid(m)) {
    ++g_c.manual_refused;
    return;
  }
  m.t_us = m.frames != 0 ? rx_us : 0;
  m.link_t_us = rx_us;
  manual_control_publish(m);
  ++g_c.manual_records;
}

// A valid sample record at `p` holding `count` samples.
void accept_samples(const std::uint8_t* p, std::size_t count) noexcept {
  ++g_c.sample_records;
  g_c.samples += static_cast<std::uint32_t>(count);
  const std::uint8_t* item = p + fmt::base::kHeaderBytes;
  for (std::size_t i = 0; i < count; ++i, item += fmt::kSampleBytes) {
    const std::uint8_t* q = item;
    const std::uint64_t tick = get_u64(q);
    q += sizeof(tick);
    ImuSample s{};
    s.t_us = get_u64(q);
    q += sizeof(s.t_us);
    for (std::size_t a = 0; a < prim::kSpatialDim; ++a, q += sizeof(float)) {
      s.gyro_rad_s[a] = get_f32(q);
    }
    for (std::size_t a = 0; a < prim::kSpatialDim; ++a, q += sizeof(float)) {
      s.accel_m_s2[a] = get_f32(q);
    }
    s.flags = get_u32(q);
    if (g_have_tick && tick != g_last_tick + 1) {
      ++g_c.tick_gaps;
    }
    g_have_tick = true;
    g_last_tick = tick;
  }
}

void parse(TimeUs rx_us) noexcept {
  std::size_t pos = 0;
  for (;;) {
    const std::size_t avail = g_rx_n - pos;
    const std::uint8_t* p = g_rx.data() + pos;
    if (avail < fmt::base::kMagicBytes) {
      break;
    }
    const bool sample = magic_at(p, fmt::kSampleMagic);
    if (!sample && !magic_at(p, fmt::kManualMagic)) {
      ++g_c.bytes_dropped;
      ++pos;
      continue;
    }
    if (avail < fmt::base::kHeaderBytes) {
      break;
    }
    const std::uint32_t count = get_u32(p + fmt::base::kMagicBytes + fmt::base::kWordBytes);
    const std::size_t max_count = sample ? kMaxBatch : 1;
    if (count < 1 || count > max_count) {
      ++g_c.bad_headers;
      ++g_c.bytes_dropped;
      ++pos;
      continue;
    }
    const std::size_t len = fmt::record_bytes(count, sample ? fmt::kSampleBytes : fmt::kManualBytes);
    if (avail < len) {
      break;
    }
    if (fmt::base::crc32(std::span{p, len - fmt::base::kCrcBytes}) != get_u32(p + len - fmt::base::kCrcBytes)) {
      ++g_c.crc_failures;
      ++g_c.bytes_dropped;
      ++pos;
      continue;
    }
    sequence(p);
    if (sample) {
      accept_samples(p, count);
    } else {
      accept_manual(p, rx_us);
    }
    pos += len;
  }
  g_rx_n -= pos;
  std::memmove(g_rx.data(), g_rx.data() + pos, g_rx_n);
}

}  // namespace

void hil_start(const HilConfig& config) noexcept {
  if (config.sink == nullptr || config.rate_counts == nullptr || config.composition_status == nullptr ||
      config.version == nullptr) {
    hal_panic("hil_start: a function or the version is missing");
  }
  g_cfg = config;
}

void hil_connect() noexcept {
  g_rx_n = 0;
  g_have_seq = false;
  g_down_seq = 0;
}

void hil_service(bool connected) noexcept {
  if (!connected) {
    return;
  }
  for (;;) {
    const int n = stdio_usb.in_chars(reinterpret_cast<char*>(g_rx.data() + g_rx_n), static_cast<int>(g_rx.size() - g_rx_n));
    if (n <= 0) {
      break;
    }
    const TimeUs rx_us = hal_time_us();
    g_rx_n += static_cast<std::size_t>(n);
    g_c.rx_bytes += static_cast<std::uint32_t>(n);
    parse(rx_us);
  }
}

}  // namespace marv::hal_target

namespace marv {

std::uint32_t hal_imu_deadline_misses() noexcept {
  return hal_target::g_deadline_misses.load(std::memory_order_relaxed);
}

}  // namespace marv
