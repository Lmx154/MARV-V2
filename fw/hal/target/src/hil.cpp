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
//
// Tick (decision 0046 item 3). A sample record's samples go into the tick queue in order and the tick interrupt is
// pended: a spare (user) interrupt at PICO_HIGHEST_IRQ_PRIORITY, the priority the flight image gives the IMU interrupt and
// its deadline alarm (lsm6dsv.cpp, imu_start). It preempts this background loop at once (the pend is followed by a data
// and an instruction barrier) and hands each queued sample, in order, to the sink, the call the flight IMU interrupt makes
// (lsm6dsv.cpp deliver: g_sink(s)), then reads the rate counts as that interrupt does after a batch. USB keeps the SDK's
// default priority (USBCTRL and the stdio worker), as in flight; the receiver interrupt is not started. The tick's
// hal_actuators_write lands in hil_actuators_write (hal_target.cpp): the values go into the report ring with the tick
// number of the sample being ticked, and this loop sends them once the record's ticks have run. The cycle counts are the
// DWT CYCCNT at clk_sys (bench_stats.hpp: the register, its enable bits and the NOCYCCNT check).
//
// Deadline (decision 0046 item 3): from the first sample record on, a gap of at least D_HIL (HilConfig::deadline_us)
// between the reads that complete two sample records is an IMU deadline miss: hal_imu_deadline_misses() increments once
// per gap, checked by this loop on every pass and, before the record's ticks run, when the next record arrives, so the
// first tick after the gap already sees it (as the flight driver counts a FIFO overrun before it delivers the batch,
// lsm6dsv.cpp). The composition latches it and disarms (arming.hpp). D_HIL is the flight image's read deadline D = FIFO
// depth / ODR (imu_read_deadline_us, core EMB-6), computed by main from the table's tick period: the longest the flight
// image lets the composition go without a fresh sample before counting a miss.
//
// Hello: on each port open the record names the image (version), the parameter table (kParamSchemaHash, count, the tick
// period, the rate divisor) and the link (batch limit, D_HIL, clk_sys), so the host can refuse a mismatch.
//
// Send: the DShot reports a record's ticks wrote go out in one write as one record, the status record once per second.
// The SDK's write waits for FIFO room and gives up after PICO_STDIO_USB_STDOUT_TIMEOUT_US (stdio_usb.c), so a host that
// does not read loses records (its sequence check sees it); MaxSendUs records the longest write.
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
#include <limits>
#include <span>
#include <tuple>

#include <hardware/clocks.h>
#include <hardware/irq.h>
#include <hardware/regs/m33.h>
#include <hardware/structs/m33.h>
#include <hardware/sync.h>
#include <pico.h>
#include <pico/stdio/driver.h>
#include <pico/stdio_usb.h>

#include "marv/hal/hal.hpp"
#include "marv/hal_target/hil_format.hpp"
#include "marv/hal_target/manual_control.hpp"
#include "marv/prim/constants.hpp"
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
constexpr TimeUs kStatusPeriodUs = static_cast<TimeUs>(prim::kMicrosecondsPerSecond);
constexpr std::size_t kU16Bits = CHAR_BIT * sizeof(std::uint16_t);
static_assert(fmt::kMotors % 2 == 0, "the DShot values are packed two to a word");

// The tick queue and the report ring hold two records' worth (a read can complete two records before the tick runs; the
// reports of a record are at most its samples), rounded up to a power of two so the free-running indices wrap cleanly.
constexpr std::size_t kQueueSize = std::bit_ceil(2 * kMaxBatch);
constexpr std::uint32_t kQueueMask = static_cast<std::uint32_t>(kQueueSize - 1);
static_assert(fmt::record_bytes(kQueueSize, fmt::kDshotItemBytes) >= fmt::kStatusRecordBytes);

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
  std::uint32_t queue_overflows = 0;
  std::uint32_t reports = 0;
  std::uint32_t max_arrival_gap_us = 0;
  std::uint32_t max_send_us = 0;
};

struct Queued {
  std::uint64_t tick = 0;
  ImuSample sample{};
};

struct Report {
  std::uint64_t tick = 0;
  std::array<std::uint16_t, fmt::kMotors> dshot{};
};

HilConfig g_cfg{};
unsigned g_tick_irq = 0;
Counters g_c{};
std::array<std::uint8_t, 2 * kMaxRecord> g_rx{};
std::size_t g_rx_n = 0;
std::array<std::uint8_t, fmt::record_bytes(kQueueSize, fmt::kDshotItemBytes)> g_up{};
std::uint32_t g_up_seq = 0;
bool g_have_seq = false;
std::uint32_t g_down_seq = 0;
bool g_have_tick = false;
std::uint64_t g_last_tick = 0;
bool g_have_arrival = false;
TimeUs g_last_arrival = 0;
bool g_gap_counted = false;
TimeUs g_status_due = 0;

// Background loop to tick: the background loop writes a slot and then advances g_queue_head; the tick reads it and then
// advances g_queue_tail.
std::array<Queued, kQueueSize> g_queue{};
std::atomic<std::uint32_t> g_queue_head{0};
std::atomic<std::uint32_t> g_queue_tail{0};

// Tick to background loop, the same way round: the tick writes and advances g_report_head, the loop reads and advances
// g_report_tail.
std::array<Report, kQueueSize> g_reports{};
std::atomic<std::uint32_t> g_report_head{0};
std::atomic<std::uint32_t> g_report_tail{0};

// The tick's own state, and what it publishes to the background loop.
std::uint64_t g_tick_now = 0;
std::atomic<std::uint32_t> g_ticks_run{0};
std::atomic<std::uint32_t> g_report_overflows{0};
std::atomic<std::uint32_t> g_max_tick_cycles{0};
std::atomic<std::uint32_t> g_max_frame_cycles{0};
std::atomic<std::uint32_t> g_rate_executions{0};
std::atomic<std::uint32_t> g_rate_timing_faults{0};

// Written by the background loop, read in the tick (hal_imu_deadline_misses).
std::atomic<std::uint32_t> g_deadline_misses{0};

std::uint32_t cycles() noexcept { return m33_hw->dwt_cyccnt; }

void raise(std::atomic<std::uint32_t>& max, std::uint32_t v) noexcept {
  if (v > max.load(std::memory_order_relaxed)) {
    max.store(v, std::memory_order_relaxed);
  }
}

// The tick interrupt.
void on_tick() {
  const std::uint32_t entry = cycles();
  std::uint32_t tail = g_queue_tail.load(std::memory_order_relaxed);
  const std::uint32_t head = g_queue_head.load(std::memory_order_acquire);
  for (; tail != head; ++tail) {
    const Queued& q = g_queue[tail & kQueueMask];
    g_tick_now = q.tick;
    const std::uint32_t t0 = cycles();
    g_cfg.sink(q.sample);
    raise(g_max_tick_cycles, cycles() - t0);
    g_ticks_run.fetch_add(1, std::memory_order_relaxed);
    g_queue_tail.store(tail + 1, std::memory_order_release);
  }
  const RateCounts rc = g_cfg.rate_counts();
  g_rate_executions.store(static_cast<std::uint32_t>(rc.executions), std::memory_order_relaxed);
  g_rate_timing_faults.store(rc.timing_faults, std::memory_order_relaxed);
  raise(g_max_frame_cycles, cycles() - entry);
}

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

void send(std::span<const std::uint8_t> bytes) noexcept {
  const TimeUs t0 = hal_time_us();
  stdio_usb.out_chars(reinterpret_cast<const char*>(bytes.data()), static_cast<int>(bytes.size()));
  g_c.max_send_us = std::max(g_c.max_send_us, static_cast<std::uint32_t>(hal_time_us() - t0));
}

// The reports the tick has queued, in one DShot record.
void send_reports() noexcept {
  std::uint32_t tail = g_report_tail.load(std::memory_order_relaxed);
  const std::uint32_t head = g_report_head.load(std::memory_order_acquire);
  if (tail == head) {
    return;
  }
  fmt::base::RecordWriter w{g_up};
  w.begin(fmt::kDshotMagic, g_up_seq++, head - tail);
  for (; tail != head; ++tail) {
    const Report& r = g_reports[tail & kQueueMask];
    w.put_u64(r.tick);
    for (std::size_t m = 0; m < fmt::kMotors; m += 2) {
      w.put_u32(static_cast<std::uint32_t>(r.dshot[m]) | (static_cast<std::uint32_t>(r.dshot[m + 1]) << kU16Bits));
    }
    ++g_c.reports;
  }
  g_report_tail.store(tail, std::memory_order_release);
  send(std::span{g_up}.first(w.end()));
}

void send_hello() noexcept {
  constexpr std::size_t kWord = fmt::base::kWordBytes;
  constexpr std::size_t kRoomWords =
      (std::tuple_size_v<decltype(g_up)> - fmt::base::kHeaderBytes - fmt::base::kCrcBytes) / kWord - fmt::kHelloFields;
  const std::size_t bytes = std::min(std::strlen(g_cfg.version), kRoomWords * kWord);
  const std::size_t words = (bytes + kWord - 1) / kWord;
  std::array<std::uint32_t, fmt::kHelloFields> f{};
  const auto at = [&f](fmt::HelloField k) -> std::uint32_t& { return f[static_cast<std::size_t>(k)]; };
  at(fmt::HelloField::SchemaHashLow) = static_cast<std::uint32_t>(g_cfg.param_schema_hash);
  at(fmt::HelloField::SchemaHashHigh) = static_cast<std::uint32_t>(g_cfg.param_schema_hash >> (CHAR_BIT * kWord));
  at(fmt::HelloField::ParamCount) = g_cfg.param_count;
  at(fmt::HelloField::TickPeriodNumUs) = static_cast<std::uint32_t>(g_cfg.tick_period_num_us);
  at(fmt::HelloField::TickPeriodDen) = static_cast<std::uint32_t>(g_cfg.tick_period_den);
  at(fmt::HelloField::RateLoopDivisor) = static_cast<std::uint32_t>(g_cfg.rate_loop_divisor);
  at(fmt::HelloField::MaxBatch) = static_cast<std::uint32_t>(kMaxBatch);
  at(fmt::HelloField::DeadlineUs) = static_cast<std::uint32_t>(g_cfg.deadline_us);
  at(fmt::HelloField::ClkSysHz) = clock_get_hz(clk_sys);
  at(fmt::HelloField::VersionBytes) = static_cast<std::uint32_t>(bytes);
  fmt::base::RecordWriter w{g_up};
  w.begin(fmt::kHelloMagic, g_up_seq++, static_cast<std::uint32_t>(f.size() + words));
  for (const std::uint32_t v : f) {
    w.put_u32(v);
  }
  for (std::size_t i = 0; i < words; ++i) {
    std::uint32_t v = 0;
    for (std::size_t b = 0; b < kWord && i * kWord + b < bytes; ++b) {
      v |= static_cast<std::uint32_t>(static_cast<unsigned char>(g_cfg.version[i * kWord + b])) << (CHAR_BIT * b);
    }
    w.put_u32(v);
  }
  send(std::span{g_up}.first(w.end()));
}

// Counts a miss once per gap of at least D_HIL since the last sample record (none before the first).
void check_deadline(TimeUs now) noexcept {
  if (g_have_arrival && !g_gap_counted && now - g_last_arrival >= g_cfg.deadline_us) {
    g_deadline_misses.fetch_add(1, std::memory_order_relaxed);
    g_gap_counted = true;
  }
}

void send_status() noexcept {
  std::array<std::uint32_t, fmt::kStatusFields> f{};
  const auto at = [&f](fmt::StatusField k) -> std::uint32_t& { return f[static_cast<std::size_t>(k)]; };
  const HilCompositionStatus comp = g_cfg.composition_status();
  at(fmt::StatusField::SampleRecords) = g_c.sample_records;
  at(fmt::StatusField::Samples) = g_c.samples;
  at(fmt::StatusField::ManualRecords) = g_c.manual_records;
  at(fmt::StatusField::ManualRefused) = g_c.manual_refused;
  at(fmt::StatusField::CrcFailures) = g_c.crc_failures;
  at(fmt::StatusField::BadHeaders) = g_c.bad_headers;
  at(fmt::StatusField::SeqGaps) = g_c.seq_gaps;
  at(fmt::StatusField::SeqLost) = g_c.seq_lost;
  at(fmt::StatusField::TickGaps) = g_c.tick_gaps;
  at(fmt::StatusField::BytesDropped) = g_c.bytes_dropped;
  at(fmt::StatusField::RxBytes) = g_c.rx_bytes;
  at(fmt::StatusField::TicksRun) = g_ticks_run.load(std::memory_order_relaxed);
  at(fmt::StatusField::QueueOverflows) = g_c.queue_overflows;
  at(fmt::StatusField::Reports) = g_c.reports;
  at(fmt::StatusField::ReportOverflows) = g_report_overflows.load(std::memory_order_relaxed);
  at(fmt::StatusField::ArmingState) = comp.arming_state;
  at(fmt::StatusField::RateExecutions) = g_rate_executions.load(std::memory_order_relaxed);
  at(fmt::StatusField::RateTimingFaults) = g_rate_timing_faults.load(std::memory_order_relaxed);
  at(fmt::StatusField::AttitudeFaults) = comp.attitude_faults;
  at(fmt::StatusField::AttitudeTimingFaults) = comp.attitude_timing_faults;
  at(fmt::StatusField::DeadlineMisses) = g_deadline_misses.load(std::memory_order_relaxed);
  at(fmt::StatusField::MaxTickCycles) = g_max_tick_cycles.load(std::memory_order_relaxed);
  at(fmt::StatusField::MaxFrameCycles) = g_max_frame_cycles.load(std::memory_order_relaxed);
  at(fmt::StatusField::MaxArrivalGapUs) = g_c.max_arrival_gap_us;
  at(fmt::StatusField::MaxSendUs) = g_c.max_send_us;
  at(fmt::StatusField::BoardTimeUs) = static_cast<std::uint32_t>(hal_time_us());
  fmt::base::RecordWriter w{g_up};
  w.begin(fmt::kStatusMagic, g_up_seq++, static_cast<std::uint32_t>(f.size()));
  for (const std::uint32_t v : f) {
    w.put_u32(v);
  }
  send(std::span{g_up}.first(w.end()));
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

// A valid sample record at `p` holding `count` samples, read at `rx_us`: queues the samples, runs their ticks (the
// interrupt), then sends the reports they wrote.
void accept_samples(const std::uint8_t* p, std::size_t count, TimeUs rx_us) noexcept {
  check_deadline(rx_us);
  if (g_have_arrival) {
    g_c.max_arrival_gap_us = std::max(g_c.max_arrival_gap_us, static_cast<std::uint32_t>(rx_us - g_last_arrival));
  }
  g_have_arrival = true;
  g_last_arrival = rx_us;
  g_gap_counted = false;
  ++g_c.sample_records;
  g_c.samples += static_cast<std::uint32_t>(count);
  const std::uint8_t* item = p + fmt::base::kHeaderBytes;
  std::uint32_t head = g_queue_head.load(std::memory_order_relaxed);
  for (std::size_t i = 0; i < count; ++i, item += fmt::kSampleBytes) {
    const std::uint8_t* q = item;
    Queued in{};
    in.tick = get_u64(q);
    q += sizeof(in.tick);
    in.sample.t_us = get_u64(q);
    q += sizeof(in.sample.t_us);
    for (std::size_t a = 0; a < prim::kSpatialDim; ++a, q += sizeof(float)) {
      in.sample.gyro_rad_s[a] = get_f32(q);
    }
    for (std::size_t a = 0; a < prim::kSpatialDim; ++a, q += sizeof(float)) {
      in.sample.accel_m_s2[a] = get_f32(q);
    }
    in.sample.flags = get_u32(q);
    if (g_have_tick && in.tick != g_last_tick + 1) {
      ++g_c.tick_gaps;
    }
    g_have_tick = true;
    g_last_tick = in.tick;
    if (head - g_queue_tail.load(std::memory_order_acquire) >= kQueueSize) {
      ++g_c.queue_overflows;
      continue;
    }
    g_queue[head & kQueueMask] = in;
    ++head;
    g_queue_head.store(head, std::memory_order_release);
  }
  irq_set_pending(g_tick_irq);
  __dsb();
  __isb();
  send_reports();
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
      accept_samples(p, count, rx_us);
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
  if (config.deadline_us < 1 || config.deadline_us >= std::numeric_limits<std::uint32_t>::max()) {
    hal_panic("hil_start: D_HIL = FIFO depth / ODR is not a positive 32-bit microsecond count");
  }
  g_cfg = config;
  m33_hw->demcr = m33_hw->demcr | M33_DEMCR_TRCENA_BITS;
  if ((m33_hw->dwt_ctrl & M33_DWT_CTRL_NOCYCCNT_BITS) != 0) {
    hal_panic("hil_start: DWT_CTRL.NOCYCCNT is set, this core has no cycle counter");
  }
  m33_hw->dwt_ctrl = m33_hw->dwt_ctrl | M33_DWT_CTRL_CYCCNTENA_BITS;
  g_tick_irq = static_cast<unsigned>(user_irq_claim_unused(true));
  irq_set_exclusive_handler(g_tick_irq, &on_tick);
  irq_set_priority(g_tick_irq, PICO_HIGHEST_IRQ_PRIORITY);
  irq_set_enabled(g_tick_irq, true);
}

void hil_connect() noexcept {
  g_rx_n = 0;
  g_have_seq = false;
  g_down_seq = 0;
  g_up_seq = 0;
  g_status_due = hal_time_us() + kStatusPeriodUs;
  send_hello();
}

void hil_service(bool connected) noexcept {
  if (connected) {
    for (;;) {
      const int n =
          stdio_usb.in_chars(reinterpret_cast<char*>(g_rx.data() + g_rx_n), static_cast<int>(g_rx.size() - g_rx_n));
      if (n <= 0) {
        break;
      }
      const TimeUs rx_us = hal_time_us();
      g_rx_n += static_cast<std::size_t>(n);
      g_c.rx_bytes += static_cast<std::uint32_t>(n);
      parse(rx_us);
    }
  }
  check_deadline(hal_time_us());
  send_reports();
  if (connected && hal_time_us() >= g_status_due) {
    g_status_due += kStatusPeriodUs;
    send_status();
  }
}

void hil_actuators_write(std::span<const DshotValue> motor) noexcept {
  const std::uint32_t head = g_report_head.load(std::memory_order_relaxed);
  if (head - g_report_tail.load(std::memory_order_acquire) >= kQueueSize) {
    g_report_overflows.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  Report& r = g_reports[head & kQueueMask];
  r.tick = g_tick_now;
  for (std::size_t m = 0; m < fmt::kMotors; ++m) {
    r.dshot[m] = motor[m].raw();
  }
  g_report_head.store(head + 1, std::memory_order_release);
}

}  // namespace marv::hal_target

namespace marv {

std::uint32_t hal_imu_deadline_misses() noexcept {
  return hal_target::g_deadline_misses.load(std::memory_order_relaxed);
}

}  // namespace marv
