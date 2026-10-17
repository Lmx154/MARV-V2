// The bench HIL link measurement (fw/hal/target/include/marv/hal_target/hil_link.hpp, format in hil_link_format.hpp).
//
// Receive. The USB CDC read goes through the SDK driver's in_chars (stdio_usb.c), which returns what the TinyUSB receive FIFO
// holds (CFG_TUD_CDC_RX_BUFSIZE bytes at most; the device NAKs the host while it is full, so a slow reader shows as lower
// throughput, not as loss). The bytes are appended to a buffer and parsed from its start: a magic, a legal count, the whole
// record, the CRC. A mismatch drops one byte and rescans, so a corrupt record costs its bytes and the stream resyncs on the
// next magic (the decoder in tools/target/imu_log.py does the same). The buffer holds two of the longest record, so after a
// parse (which leaves less than one record) a read always has the longest record's room.
//
// Arrival time. The board clock is read once after each read returns; a record completed by that read is stamped with it.
//
// Send. All reports of one downlink record go out in one write, each its own framed record, then the status record when the
// second is due. The SDK's write waits for FIFO room and gives up after PICO_STDIO_USB_STDOUT_TIMEOUT_US (stdio_usb.c), so
// a host that does not read loses reports (the host's sequence check sees it); MaxSendUs records the longest write.
#include <algorithm>
#include <array>
#include <climits>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

#include <pico/stdio/driver.h>
#include <pico/stdio_usb.h>

#include "marv/hal/hal.hpp"
#include "marv/hal_target/hil_link.hpp"
#include "marv/hal_target/hil_link_format.hpp"
#include "marv/prim/constants.hpp"

#ifndef MARV_FC3V2_HIL_LINK_MAX_BATCH
#error "MARV_FC3V2_HIL_LINK_MAX_BATCH is set by fw/hal/target/CMakeLists.txt"
#endif

namespace marv::hal_target {
namespace {

namespace fmt = hil_link;

constexpr std::size_t kMaxBatch = MARV_FC3V2_HIL_LINK_MAX_BATCH;
constexpr std::size_t kMaxRecord = fmt::base::data_record_bytes(kMaxBatch);
constexpr TimeUs kStatusPeriodUs = static_cast<TimeUs>(prim::kMicrosecondsPerSecond);
constexpr std::size_t kU16Bits = CHAR_BIT * sizeof(std::uint16_t);
static_assert(kMaxBatch >= 1);

struct Counters {
  std::uint32_t down_records = 0;
  std::uint32_t down_samples = 0;
  std::uint32_t crc_failures = 0;
  std::uint32_t bad_headers = 0;
  std::uint32_t seq_gaps = 0;
  std::uint32_t seq_lost = 0;
  std::uint32_t tick_gaps = 0;
  std::uint32_t bytes_dropped = 0;
  std::uint32_t rx_bytes = 0;
  std::uint32_t up_reports = 0;
  std::uint32_t max_arrival_gap_us = 0;
  std::uint32_t max_send_us = 0;
};

Counters g_c{};
std::array<std::uint8_t, 2 * kMaxRecord> g_rx{};
std::size_t g_rx_n = 0;
std::array<std::uint8_t, kMaxBatch * fmt::kReportRecordBytes> g_up{};
std::uint32_t g_up_seq = 0;
bool g_have_seq = false;
std::uint32_t g_down_seq = 0;
bool g_have_tick = false;
std::uint64_t g_last_tick = 0;
bool g_have_arrival = false;
TimeUs g_last_arrival = 0;
TimeUs g_status_due = 0;

std::uint32_t get_u32(const std::uint8_t* p) noexcept {
  std::uint32_t v = 0;
  for (std::size_t i = 0; i < sizeof(v); ++i) {
    v |= static_cast<std::uint32_t>(p[i]) << (CHAR_BIT * i);
  }
  return v;
}

std::uint64_t get_u64(const std::uint8_t* p) noexcept {
  return static_cast<std::uint64_t>(get_u32(p)) | (static_cast<std::uint64_t>(get_u32(p + sizeof(std::uint32_t))) << (CHAR_BIT * sizeof(std::uint32_t)));
}

bool magic_at(const std::uint8_t* p, const std::array<std::uint8_t, fmt::base::kMagicBytes>& m) noexcept {
  return std::memcmp(p, m.data(), m.size()) == 0;
}

void send(std::span<const std::uint8_t> bytes) noexcept {
  const TimeUs t0 = hal_time_us();
  stdio_usb.out_chars(reinterpret_cast<const char*>(bytes.data()), static_cast<int>(bytes.size()));
  g_c.max_send_us = std::max(g_c.max_send_us, static_cast<std::uint32_t>(hal_time_us() - t0));
}

void send_status() noexcept {
  std::array<std::uint32_t, fmt::kStatusFields> f{};
  const auto at = [&f](fmt::StatusField k) -> std::uint32_t& { return f[static_cast<std::size_t>(k)]; };
  at(fmt::StatusField::DownRecords) = g_c.down_records;
  at(fmt::StatusField::DownSamples) = g_c.down_samples;
  at(fmt::StatusField::CrcFailures) = g_c.crc_failures;
  at(fmt::StatusField::BadHeaders) = g_c.bad_headers;
  at(fmt::StatusField::SeqGaps) = g_c.seq_gaps;
  at(fmt::StatusField::SeqLost) = g_c.seq_lost;
  at(fmt::StatusField::TickGaps) = g_c.tick_gaps;
  at(fmt::StatusField::BytesDropped) = g_c.bytes_dropped;
  at(fmt::StatusField::RxBytes) = g_c.rx_bytes;
  at(fmt::StatusField::UpReports) = g_c.up_reports;
  at(fmt::StatusField::MaxArrivalGapUs) = g_c.max_arrival_gap_us;
  at(fmt::StatusField::MaxSendUs) = g_c.max_send_us;
  at(fmt::StatusField::BoardTimeUs) = static_cast<std::uint32_t>(hal_time_us());
  at(fmt::StatusField::MaxBatch) = static_cast<std::uint32_t>(kMaxBatch);
  std::array<std::uint8_t, fmt::kStatusRecordBytes> buf{};
  fmt::base::RecordWriter w{buf};
  w.begin(fmt::kStatusMagic, g_up_seq++, static_cast<std::uint32_t>(f.size()));
  for (const std::uint32_t v : f) {
    w.put_u32(v);
  }
  send(std::span{buf}.first(w.end()));
}

// A valid downlink record at `p` holding `count` samples, whose last byte arrived at `rx_us`: checks the sequence and the
// ticks, and answers each sample with a report.
void accept(const std::uint8_t* p, std::size_t count, TimeUs rx_us) noexcept {
  const std::uint32_t seq = get_u32(p + fmt::base::kMagicBytes);
  if (g_have_seq && seq != g_down_seq) {
    ++g_c.seq_gaps;
    g_c.seq_lost += seq - g_down_seq;
  }
  g_have_seq = true;
  g_down_seq = seq + 1;
  if (g_have_arrival) {
    g_c.max_arrival_gap_us = std::max(g_c.max_arrival_gap_us, static_cast<std::uint32_t>(rx_us - g_last_arrival));
  }
  g_have_arrival = true;
  g_last_arrival = rx_us;
  ++g_c.down_records;
  g_c.down_samples += static_cast<std::uint32_t>(count);

  const std::uint8_t* item = p + fmt::base::kHeaderBytes;
  std::size_t out = 0;
  for (std::size_t i = 0; i < count; ++i, item += fmt::kSampleBytes) {
    const std::uint64_t tick = get_u64(item);
    if (g_have_tick && tick != g_last_tick + 1) {
      ++g_c.tick_gaps;
    }
    g_have_tick = true;
    g_last_tick = tick;
    fmt::base::RecordWriter w{std::span{g_up}.subspan(out)};
    w.begin(fmt::kReportMagic, g_up_seq++, 1);
    w.put_u64(tick);
    for (std::size_t m = 0; m < fmt::kMotors; m += 2) {
      const auto v0 = static_cast<std::uint16_t>(tick + m);
      const auto v1 = static_cast<std::uint16_t>(tick + m + 1);
      w.put_u32(static_cast<std::uint32_t>(v0) | (static_cast<std::uint32_t>(v1) << kU16Bits));
    }
    w.put_u64(rx_us);
    out += w.end();
    ++g_c.up_reports;
  }
  send(std::span{g_up}.first(out));
}

void parse(TimeUs rx_us) noexcept {
  std::size_t pos = 0;
  for (;;) {
    const std::size_t avail = g_rx_n - pos;
    const std::uint8_t* p = g_rx.data() + pos;
    if (avail < fmt::base::kMagicBytes) {
      break;
    }
    if (!magic_at(p, fmt::kDownMagic)) {
      ++g_c.bytes_dropped;
      ++pos;
      continue;
    }
    if (avail < fmt::base::kHeaderBytes) {
      break;
    }
    const std::uint32_t count = get_u32(p + fmt::base::kMagicBytes + fmt::base::kWordBytes);
    if (count < 1 || count > kMaxBatch) {
      ++g_c.bad_headers;
      ++g_c.bytes_dropped;
      ++pos;
      continue;
    }
    const std::size_t len = fmt::base::data_record_bytes(count);
    if (avail < len) {
      break;
    }
    if (fmt::base::crc32(std::span{p, len - fmt::base::kCrcBytes}) != get_u32(p + len - fmt::base::kCrcBytes)) {
      ++g_c.crc_failures;
      ++g_c.bytes_dropped;
      ++pos;
      continue;
    }
    accept(p, count, rx_us);
    pos += len;
  }
  g_rx_n -= pos;
  std::memmove(g_rx.data(), g_rx.data() + pos, g_rx_n);
}

}  // namespace

void hil_link_reset() noexcept {
  g_c = Counters{};
  g_rx_n = 0;
  g_up_seq = 0;
  g_have_seq = false;
  g_down_seq = 0;
  g_have_tick = false;
  g_last_tick = 0;
  g_have_arrival = false;
  g_last_arrival = 0;
  g_status_due = hal_time_us() + kStatusPeriodUs;
}

void hil_link_service() noexcept {
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
  if (hal_time_us() >= g_status_due) {
    g_status_due += kStatusPeriodUs;
    send_status();
  }
}

}  // namespace marv::hal_target
