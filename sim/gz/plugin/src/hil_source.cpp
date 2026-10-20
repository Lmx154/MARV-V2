// The HIL command source (hil_source.hpp; HIL stage S4, decision 0046 items 2-4).
#include "hil_source.hpp"

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cerrno>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstring>
#include <limits>
#include <numbers>
#include <span>
#include <string_view>

#include <marv/params/param_ids.hpp>

#include "constants.hpp"
#include "marv/arming/arming.hpp"
#include "marv/hal_target/hil_format.hpp"

namespace marv::gz {
namespace {

namespace fmt = marv::hal_target::hil;
namespace base = fmt::base;

constexpr auto kNsPerS = static_cast<std::int64_t>(kNanosecondsPerSecond);
constexpr auto kNsPerUs = static_cast<std::int64_t>(kNanosecondsPerMicrosecond);
constexpr std::int64_t kUsPerMs = static_cast<std::int64_t>(kNanosecondsPerMicrosecond);  // 1000 us in a ms, as ns in a us
constexpr std::size_t kReadChunk = std::size_t{1} << (2 * CHAR_BIT);  // bytes per read() of the uplink (64 KiB)
// The largest DShot record and the longest version string the parser accepts: a byte's range of reports and of words (the
// board sends at most two records' worth of reports, hil.cpp kQueueSize, and a version of a few dozen bytes).
constexpr std::uint32_t kMaxCountSpan = std::numeric_limits<std::uint8_t>::max();
constexpr double kDegPerRad = 180.0 / std::numbers::pi;  // a half turn is 180 degrees and pi radians
// The status line's touchdown text (<ground>): " | touchdown ", up to 20 digits, " (last ", a %.1f speed and " m/s)" fit with
// room; a longer text is cut, never overrun (snprintf). Labelled harness value.
constexpr std::size_t kTouchdownTextBytes = 96;
// Labelled harness values of open(): DTR is held low for 50 ms (many polls of the board's background loop, which reads the
// line state on every pass) before it is raised, and the edge is repeated after 1.5 s without a hello (longer than the
// board's hello and receiver-boot records take); a poll waits at most 10 ms.
constexpr long kDtrLowNs = 50'000'000;
constexpr std::int64_t kDtrRetryNs = 1'500'000'000;
constexpr int kPollMs = 10;

std::uint32_t get_u32(const std::uint8_t* p) {
  std::uint32_t v = 0;
  for (std::size_t i = 0; i < sizeof(v); ++i) {
    v |= static_cast<std::uint32_t>(p[i]) << (CHAR_BIT * i);
  }
  return v;
}

std::uint64_t get_u64(const std::uint8_t* p) {
  return static_cast<std::uint64_t>(get_u32(p)) |
         (static_cast<std::uint64_t>(get_u32(p + sizeof(std::uint32_t))) << (CHAR_BIT * sizeof(std::uint32_t)));
}

float get_f32(const std::uint8_t* p) { return std::bit_cast<float>(get_u32(p)); }

std::int64_t mono_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

const char* arm_text(std::uint32_t code) {
  using arming::ArmState;
  switch (static_cast<ArmState>(code)) {
    case ArmState::Boot:
      return "boot";
    case ArmState::SwitchOff:
      return "switch_off";
    case ArmState::SwitchNotCycled:
      return "switch_not_cycled";
    case ArmState::ThrottleUp:
      return "throttle_up";
    case ArmState::ImuStale:
      return "imu_stale";
    case ArmState::ImuInvalid:
      return "imu_invalid";
    case ArmState::Saturated:
      return "saturated";
    case ArmState::NoLink:
      return "no_link";
    case ArmState::Failsafe:
      return "failsafe";
    case ArmState::MotorTest:
      return "motor_test";
    case ArmState::Armed:
      return "ARMED";
    case ArmState::ArmedHold:
      return "ARMED_hold";
    case ArmState::ArmedFailsafeCentred:
      return "ARMED_failsafe_centred";
    case ArmState::EstimatorInvalid:
      return "estimator_invalid";
    case ArmState::Calibrating:
      return "calibrating";
    case ArmState::NotUpright:
      return "not_upright";
    case ArmState::ConfigRefused:
      return "config_refused";
    case ArmState::Maintenance:
      return "maintenance";
  }
  return "unknown";
}

enum class Kind { kHello, kDshot, kArrival, kStatus, kReceiverBoot, kReceiverFrame, kReceiverStatus };

struct KindInfo {
  const std::array<std::uint8_t, base::kMagicBytes>* magic;
  Kind kind;
};

const std::array<KindInfo, 7> kKinds{{{&fmt::kHelloMagic, Kind::kHello},
                                      {&fmt::kDshotMagic, Kind::kDshot},
                                      {&fmt::kArrivalMagic, Kind::kArrival},
                                      {&fmt::kStatusMagic, Kind::kStatus},
                                      {&fmt::kReceiverBootMagic, Kind::kReceiverBoot},
                                      {&fmt::kReceiverFrameMagic, Kind::kReceiverFrame},
                                      {&fmt::kReceiverStatusMagic, Kind::kReceiverStatus}}};

// The item size of a kind and whether `count` is legal for it.
bool legal(Kind k, std::uint32_t count, std::size_t& item) {
  switch (k) {
    case Kind::kHello:
      item = base::kWordBytes;
      return count >= fmt::kHelloFields && count - fmt::kHelloFields <= kMaxCountSpan;
    case Kind::kDshot:
      item = fmt::kDshotItemBytes;
      return count >= 1 && count <= kMaxCountSpan;
    case Kind::kArrival:
      item = fmt::kArrivalItemBytes;
      return count == 1;
    case Kind::kStatus:
      item = base::kWordBytes;
      return count == fmt::kStatusFields;
    case Kind::kReceiverBoot:
      item = base::kWordBytes;
      return count == fmt::kReceiverBootFields;
    case Kind::kReceiverFrame:
      item = fmt::kReceiverFrameBytes;
      return count == 1;
    case Kind::kReceiverStatus:
      item = base::kWordBytes;
      return count == fmt::kReceiverStatusFields;
  }
  return false;
}

std::uint32_t field(const std::vector<std::uint32_t>& v, auto f) {
  const auto i = static_cast<std::size_t>(f);
  return i < v.size() ? v[i] : 0;
}

}  // namespace

HilCommandSource::~HilCommandSource() { finish(); }

std::int64_t HilCommandSource::now_ns() const { return mono_ns() - t0_abs_ns_; }

std::uint64_t HilCommandSource::due_ns(std::uint64_t tick) const {
  __extension__ using Wide = unsigned __int128;
  const Wide ns = static_cast<Wide>(tick) * cfg_.num_us * kNanosecondsPerMicrosecond / cfg_.den;
  return static_cast<std::uint64_t>(ns);
}

bool HilCommandSource::fail(const std::string& why) {
  if (failure_.empty()) {
    failure_ = why;
  }
  for (std::FILE* f : {uplink_, frames_, reports_}) {
    if (f != nullptr) {
      std::fflush(f);
    }
  }
  return false;
}

std::string HilCommandSource::open(const HilConfig& cfg) {
  cfg_ = cfg;
  // For the status line's flight mode: quad_fc selects angle iff rc_channel_angle is not 0 and that channel is >=
  // angle_switch_threshold (a NaN is acro; quad_fc.cpp), read by name from this plugin's parameter table, the one the board runs
  // (its values CRC is checked below). A table without them (not quad_fc) shows no mode.
  for (std::size_t i = 0; i < kParamCount; ++i) {
    const std::string_view name = generated::kParamNames[i];
    const ParamValue& v = generated::kParamDefaults[i].value;
    if (name == "rc_channel_angle" && v.i32 > 0 && static_cast<std::size_t>(v.i32) <= prim::kCrsfRcChannels) {
      angle_channel_ = static_cast<std::size_t>(v.i32);
    } else if (name == "angle_switch_threshold") {
      angle_threshold_ = v.f32;
      have_angle_threshold_ = true;
    }
  }
  if (!have_angle_threshold_) {
    angle_channel_ = 0;
  }
  if (cfg_.samples_per_record < 1 || cfg_.num_us == 0 || cfg_.den == 0) {
    return "<hil> samples_per_record, or the tick period, is 0";
  }
  read_buf_.assign(kReadChunk, 0);
  fd_ = ::open(cfg_.device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    return "cannot open the board's port " + cfg_.device + ": " + std::strerror(errno);
  }
  termios t{};
  if (tcgetattr(fd_, &t) != 0) {
    return "tcgetattr " + cfg_.device + ": " + std::strerror(errno);
  }
  cfmakeraw(&t);
  t.c_cflag |= CLOCAL | CREAD;
  if (tcsetattr(fd_, TCSANOW, &t) != 0 || tcflush(fd_, TCIOFLUSH) != 0) {
    return "cannot set " + cfg_.device + " raw: " + std::strerror(errno);
  }
  uplink_ = std::fopen(cfg_.uplink_path.c_str(), "wb");
  frames_ = std::fopen(cfg_.frames_path.c_str(), "w");
  reports_ = std::fopen(cfg_.reports_path.c_str(), "w");
  if (uplink_ == nullptr || frames_ == nullptr || reports_ == nullptr) {
    return "cannot open the HIL record files (" + cfg_.uplink_path + ", " + cfg_.frames_path + ", " + cfg_.reports_path + ")";
  }
  std::fprintf(frames_, "first_tick,count,last_due_ns,lag_ns,sent_ns\n");
  std::fprintf(reports_, "tick,arrival_ns,needed_ns\n");

  // The board sends its hello on a rising edge of DTR (hil.cpp hil_connect). Another process that has the port open (a
  // ModemManager probe of the newly enumerated port, seen on the bench) can hold DTR up, so no edge comes, or read the hello
  // first: until the hello and the receiver-boot record are here, DTR is dropped and raised again once per
  // kDtrRetryNs; then the first status record (once per second) is awaited.
  const std::int64_t start = mono_ns();
  const auto wait_ns = static_cast<std::int64_t>(cfg_.hello_wait_us) * kNsPerUs;
  std::int64_t next_edge = start;
  while (!(hello_ && receiver_boot_ && status_records_ > 0) && mono_ns() - start < wait_ns && failure_.empty()) {
    if (!(hello_ && receiver_boot_) && mono_ns() >= next_edge) {
      int dtr = TIOCM_DTR;
      if (ioctl(fd_, TIOCMBIC, &dtr) != 0) {
        return "cannot drop DTR on " + cfg_.device + ": " + std::strerror(errno);
      }
      const timespec low{0, kDtrLowNs};
      nanosleep(&low, nullptr);
      if (ioctl(fd_, TIOCMBIS, &dtr) != 0) {
        return "cannot assert DTR on " + cfg_.device + ": " + std::strerror(errno);
      }
      next_edge = mono_ns() + kDtrRetryNs;
      hello_ = false;
      receiver_boot_ = false;
      status_records_ = 0;
    }
    pollfd p{fd_, POLLIN, 0};
    static_cast<void>(poll(&p, 1, kPollMs));
    service();
  }
  if (!failure_.empty()) {
    return failure_;
  }
  if (!hello_) {
    return "no hello record from " + cfg_.device + " within " + std::to_string(cfg_.hello_wait_us) +
           " us (is the MARV_FC3V2_HIL_RECEIVER image running?)";
  }
  if (!receiver_boot_) {
    return "the board sent no receiver-boot record after its hello: not a MARV_FC3V2_HIL_RECEIVER image (decision 0046 "
           "item 4: S4 arms from the board's own receiver)";
  }
  if (status_records_ == 0) {
    return "no status record from the board within " + std::to_string(cfg_.hello_wait_us) + " us";
  }
  using fmt::HelloField;
  const std::uint64_t hash = static_cast<std::uint64_t>(field(hello_fields_, HelloField::SchemaHashLow)) |
                             (static_cast<std::uint64_t>(field(hello_fields_, HelloField::SchemaHashHigh))
                              << (CHAR_BIT * sizeof(std::uint32_t)));
  const std::uint32_t crc = field(hello_fields_, HelloField::ParamValuesCrc);
  const std::uint32_t num = field(hello_fields_, HelloField::TickPeriodNumUs);
  const std::uint32_t den = field(hello_fields_, HelloField::TickPeriodDen);
  const std::uint32_t max_batch = field(hello_fields_, HelloField::MaxBatch);
  rate_divisor_ = field(hello_fields_, HelloField::RateLoopDivisor);
  if (hash != cfg_.schema_hash) {
    return "the board's parameter schema hash " + std::to_string(hash) + " is not the SIL library's " +
           std::to_string(cfg_.schema_hash);
  }
  if (crc != cfg_.values_crc) {
    return "the board's parameter values CRC " + std::to_string(crc) + " is not <values_crc> " + std::to_string(cfg_.values_crc);
  }
  if (num != cfg_.num_us || den != cfg_.den) {
    return "the board's tick period " + std::to_string(num) + "/" + std::to_string(den) + " us is not the world's " +
           std::to_string(cfg_.num_us) + "/" + std::to_string(cfg_.den);
  }
  if (max_batch < cfg_.samples_per_record) {
    return "<samples_per_record> " + std::to_string(cfg_.samples_per_record) + " is above the board's batch limit " +
           std::to_string(max_batch);
  }
  if (rate_divisor_ < 1) {
    return "the board's rate_loop_divisor is 0";
  }
  using fmt::StatusField;
  if (field(status_, StatusField::TicksRun) != 0 || field(status_, StatusField::SampleRecords) != 0 ||
      field(status_, StatusField::DeadlineMisses) != 0) {
    return "the board has run " + std::to_string(field(status_, StatusField::TicksRun)) + " ticks and counted " +
           std::to_string(field(status_, StatusField::DeadlineMisses)) +
           " deadline misses since boot: reset it before a run (tools/target/flash.sh --probe-only reboots it)";
  }
  std::string version;
  const std::uint32_t vbytes = field(hello_fields_, HelloField::VersionBytes);
  for (std::size_t i = 0; i < vbytes && fmt::kHelloFields + i / base::kWordBytes < hello_fields_.size(); ++i) {
    version.push_back(static_cast<char>(hello_fields_[fmt::kHelloFields + i / base::kWordBytes] >> (CHAR_BIT * (i % base::kWordBytes))));
  }
  std::fprintf(stderr,
               "marv_gz_lockstep: hil board %s: version %s, schema hash %016llx, values CRC %08x, tick %u/%u us, rate divisor "
               "%u, batch limit %u, D_HIL %u us; F = %u samples per record, N = %u ticks\n",
               cfg_.device.c_str(), version.c_str(), static_cast<unsigned long long>(hash), crc, num, den, rate_divisor_,
               max_batch, field(hello_fields_, HelloField::DeadlineUs), cfg_.samples_per_record, cfg_.delay_ticks);
  return {};
}

bool HilCommandSource::service() {
  if (fd_ < 0) {
    return fail("the board's port is not open");
  }
  for (;;) {
    std::vector<std::uint8_t>& buf = read_buf_;
    const ssize_t n = ::read(fd_, buf.data(), buf.size());
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
        return failure_.empty();
      }
      return fail(std::string("reading the board's port: ") + std::strerror(errno));
    }
    if (n == 0) {
      return fail("the board's port closed (end of file)");
    }
    const auto len = static_cast<std::size_t>(n);
    if (uplink_ != nullptr && std::fwrite(buf.data(), 1, len, uplink_) != len) {
      return fail("cannot write " + cfg_.uplink_path);
    }
    rx_.insert(rx_.end(), buf.begin(), buf.begin() + n);
    parse();
    if (!failure_.empty()) {
      return false;
    }
  }
}

void HilCommandSource::parse() {
  const std::int64_t arrival = now_ns();
  std::size_t pos = 0;
  while (failure_.empty()) {
    const std::size_t avail = rx_.size() - pos;
    const std::uint8_t* p = rx_.data() + pos;
    if (avail < base::kMagicBytes) {
      break;
    }
    const KindInfo* info = nullptr;
    for (const KindInfo& k : kKinds) {
      if (std::memcmp(p, k.magic->data(), base::kMagicBytes) == 0) {
        info = &k;
      }
    }
    if (info == nullptr) {
      ++up_unframed_;
      ++pos;
      continue;
    }
    if (avail < base::kHeaderBytes) {
      break;
    }
    const std::uint32_t count = get_u32(p + base::kMagicBytes + base::kWordBytes);
    std::size_t item = 0;
    if (!legal(info->kind, count, item)) {
      ++up_unframed_;
      ++pos;
      continue;
    }
    const std::size_t len = fmt::record_bytes(count, item);
    if (avail < len) {
      break;
    }
    if (base::crc32(std::span{p, len - base::kCrcBytes}) != get_u32(p + len - base::kCrcBytes)) {
      ++up_crc_failures_;
      ++pos;
      continue;
    }
    const std::uint32_t seq = get_u32(p + base::kMagicBytes);
    if (info->kind == Kind::kHello) {
      if (started_) {
        fail("the board sent a hello in the middle of the run (it reset, or its port was reopened)");
        break;
      }
      have_up_seq_ = false;
    }
    if (have_up_seq_ && seq != up_seq_next_) {
      ++up_seq_gaps_;
    }
    have_up_seq_ = true;
    up_seq_next_ = seq + 1;
    const std::uint8_t* q = p + base::kHeaderBytes;
    switch (info->kind) {
      case Kind::kHello:
        hello_fields_.assign(count, 0);
        for (std::uint32_t i = 0; i < count; ++i) {
          hello_fields_[i] = get_u32(q + i * base::kWordBytes);
        }
        hello_ = true;
        break;
      case Kind::kDshot:
        for (std::uint32_t i = 0; i < count && failure_.empty(); ++i, q += fmt::kDshotItemBytes) {
          Report r;
          r.tick = get_u64(q);
          for (std::size_t m = 0; m < fmt::kMotors; m += 2) {
            const std::uint32_t w = get_u32(q + sizeof(std::uint64_t) + m * sizeof(std::uint16_t));
            r.dshot[m] = static_cast<std::uint16_t>(w);
            r.dshot[m + 1] = static_cast<std::uint16_t>(w >> (CHAR_BIT * sizeof(std::uint16_t)));
          }
          r.arrival_ns = arrival;
          const std::uint64_t expected = reports_received_ * rate_divisor_;
          if (rate_divisor_ == 0 || r.tick != expected) {
            fail("the board's DShot report is for tick " + std::to_string(r.tick) + ", the schedule expects tick " +
                 std::to_string(expected) + " (report " + std::to_string(reports_received_) +
                 " of a fresh board, rate divisor " + std::to_string(rate_divisor_) + ")");
            break;
          }
          pending_.push_back(r);
          ++reports_received_;
        }
        break;
      case Kind::kStatus:
        status_.assign(count, 0);
        for (std::uint32_t i = 0; i < count; ++i) {
          status_[i] = get_u32(q + i * base::kWordBytes);
        }
        ++status_records_;
        break;
      case Kind::kReceiverBoot:
        receiver_boot_ = true;
        break;
      case Kind::kReceiverFrame:
        ++receiver_frames_;
        rx_frames_ = get_u32(q);
        rx_throttle_ = get_f32(q + sizeof(std::uint32_t) + sizeof(std::uint64_t) + 3 * sizeof(float));
        for (std::size_t i = 0; i < rx_sticks_.size(); ++i) {
          rx_sticks_[i] = get_f32(q + sizeof(std::uint32_t) + sizeof(std::uint64_t) + i * sizeof(float));
        }
        for (std::size_t i = 0; i < rx_channels_.size(); ++i) {
          rx_channels_[i] = get_f32(q + sizeof(std::uint32_t) + sizeof(std::uint64_t) + (rx_sticks_.size() + i) * sizeof(float));
        }
        rx_lq_ = get_f32(q + sizeof(std::uint32_t) + sizeof(std::uint64_t) + (4 + prim::kCrsfRcChannels) * sizeof(float));
        // The flight mode the board's quad_fc selects from this frame (open(): the angle channel and threshold).
        rx_angle_ = angle_channel_ > 0 && get_f32(q + sizeof(std::uint32_t) + sizeof(std::uint64_t) +
                                                  (4 + angle_channel_ - 1) * sizeof(float)) >= angle_threshold_;
        break;
      case Kind::kReceiverStatus:
        receiver_status_.assign(count, 0);
        for (std::uint32_t i = 0; i < count; ++i) {
          receiver_status_[i] = get_u32(q + i * base::kWordBytes);
        }
        break;
      case Kind::kArrival:
        break;
    }
    pos += len;
  }
  rx_.erase(rx_.begin(), rx_.begin() + static_cast<std::ptrdiff_t>(pos));
}

bool HilCommandSource::send_frame(std::uint64_t first_tick) {
  const std::uint64_t last = first_tick + frame_count_ - 1;
  const std::size_t len = frame_.size();
  std::size_t off = 0;
  const std::int64_t start = now_ns();
  while (off < len) {
    const ssize_t n = ::write(fd_, frame_.data() + off, len - off);
    if (n > 0) {
      off += static_cast<std::size_t>(n);
      continue;
    }
    if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
      return fail(std::string("writing the board's port: ") + std::strerror(errno));
    }
    if (!service()) {
      return false;
    }
    if (now_ns() - start > static_cast<std::int64_t>(cfg_.late_wait_us) * kNsPerUs) {
      return fail("the write of the sample record of ticks " + std::to_string(first_tick) + ".." + std::to_string(last) +
                  " stalled for more than " + std::to_string(cfg_.late_wait_us) + " us");
    }
  }
  const std::int64_t sent = now_ns();
  const auto due = static_cast<std::int64_t>(due_ns(last)) + lag_ns_;
  max_send_late_ns_ = std::max(max_send_late_ns_, sent - due);
  ++frames_sent_;
  std::fprintf(frames_, "%llu,%u,%lld,%lld,%lld\n", static_cast<unsigned long long>(first_tick), frame_count_,
               static_cast<long long>(due_ns(last)), static_cast<long long>(lag_ns_), static_cast<long long>(sent));
  frame_count_ = 0;
  frame_.clear();
  return true;
}

bool HilCommandSource::dshot(std::uint64_t tick, Dshot& out) {
  static_cast<void>(tick);
  static_cast<void>(out);
  return fail("the HIL source needs the tick's IMU sample (<gyro_source>model</gyro_source>)");
}

bool HilCommandSource::dshot(std::uint64_t tick, const marv_imu_meas* imu, Dshot& out) {
  if (!failure_.empty()) {
    return false;
  }
  if (imu == nullptr) {
    return dshot(tick, out);
  }
  if (!started_) {
    t0_abs_ns_ = mono_ns();
    started_ = true;
  }
  // Tick j's instant: the schedule plus the lag absorbed so far. A tick reached after its instant (gz was slow) moves the
  // schedule by the lateness instead of catching up, so the samples never go out faster than the tick period and every
  // record keeps the full N - F + 1 ticks between its send and the need of its command; the lag is counted.
  const std::int64_t due = static_cast<std::int64_t>(due_ns(tick)) + lag_ns_;
  std::int64_t now = now_ns();
  if (now > due) {
    lag_ns_ += now - due;
    max_lag_step_ns_ = std::max(max_lag_step_ns_, now - due);
    ++lag_events_;
  }
  while (now < due) {
    if (!service()) {
      return false;
    }
    now = now_ns();
  }
  stamp_us_ = (tick / cfg_.den) * cfg_.num_us + ((tick % cfg_.den) * cfg_.num_us) / cfg_.den;

  // Queue the sample (hil_format.hpp sample item); the record goes out with its F-th sample.
  const auto put32 = [this](std::uint32_t v) {
    for (std::size_t i = 0; i < sizeof(v); ++i) {
      frame_.push_back(static_cast<std::uint8_t>(v >> (CHAR_BIT * i)));
    }
  };
  const auto put64 = [&put32](std::uint64_t v) {
    put32(static_cast<std::uint32_t>(v));
    put32(static_cast<std::uint32_t>(v >> (CHAR_BIT * sizeof(std::uint32_t))));
  };
  if (frame_count_ == 0) {
    frame_.assign(base::kHeaderBytes, 0);  // the header is written when the record is complete
  }
  put64(tick);
  put64(stamp_us_);
  for (const float v : {imu->gyro_rad_s.x, imu->gyro_rad_s.y, imu->gyro_rad_s.z, imu->accel_m_s2.x, imu->accel_m_s2.y,
                        imu->accel_m_s2.z}) {
    put32(std::bit_cast<std::uint32_t>(v));
  }
  put32(imu->flags);
  ++frame_count_;
  if (frame_count_ == cfg_.samples_per_record) {
    std::array<std::uint8_t, base::kHeaderBytes> header{};
    base::RecordWriter w{std::span{header}};
    w.begin(fmt::kSampleMagic, down_seq_++, frame_count_);
    std::copy(header.begin(), header.end(), frame_.begin());
    put32(base::crc32(std::span{frame_}));
    if (!send_frame(tick + 1 - frame_count_)) {
      return false;
    }
  }

  if (tick >= next_status_tick_) {
    status_line(tick);
    next_status_tick_ = tick + (static_cast<std::uint64_t>(kNsPerS) * cfg_.den + cfg_.num_us * kNanosecondsPerMicrosecond - 1) /
                                   (cfg_.num_us * kNanosecondsPerMicrosecond);
  }

  // The command of tick j - N: the board's latch, its report of rate tick k d.
  if (tick < cfg_.delay_ticks) {
    out = Dshot{};
    return true;
  }
  const std::uint64_t need = tick - cfg_.delay_ticks;
  const std::uint64_t kk = need / rate_divisor_;
  if (!service()) {
    return false;
  }
  const std::int64_t needed = now_ns();
  if (reports_received_ <= kk) {
    while (reports_received_ <= kk && now_ns() - needed <= static_cast<std::int64_t>(cfg_.late_wait_us) * kNsPerUs) {
      if (!service()) {
        return false;
      }
    }
    const std::string what = "HIL late command: tick " + std::to_string(tick) + " needs the board's command of tick " +
                             std::to_string(need) + " (j - N, N = " + std::to_string(cfg_.delay_ticks) +
                             "), its DShot report of rate tick " + std::to_string(kk * rate_divisor_) + ", ";
    if (reports_received_ <= kk) {
      return fail(what + "which had not arrived " + std::to_string(cfg_.late_wait_us) + " us after it was needed");
    }
    const Report& r = pending_[static_cast<std::size_t>(kk - pending_base_)];
    return fail(what + "which arrived " + std::to_string((r.arrival_ns - needed) / kNsPerUs) +
                " us after it was needed (no command is reused: the run stops)");
  }
  while (pending_base_ < kk) {
    pending_.pop_front();
    ++pending_base_;
  }
  const Report& r = pending_.front();
  if (!have_applied_ || kk != applied_k_) {
    const std::int64_t margin = needed - r.arrival_ns;
    min_margin_ns_ = reports_used_ == 0 ? margin : std::min(min_margin_ns_, margin);
    max_margin_ns_ = reports_used_ == 0 ? margin : std::max(max_margin_ns_, margin);
    ++reports_used_;
    std::fprintf(reports_, "%llu,%lld,%lld\n", static_cast<unsigned long long>(r.tick), static_cast<long long>(r.arrival_ns),
                 static_cast<long long>(needed));
    applied_ = r.dshot;
    applied_k_ = kk;
    have_applied_ = true;
  }
  out = applied_;
  return true;
}

void HilCommandSource::status_line(std::uint64_t tick) {
  using fmt::StatusField;
  const double t_s = static_cast<double>(due_ns(tick)) / kNanosecondsPerSecond;
  // The angle between body z and NED z: R_zz = 1 - 2 (x^2 + y^2) of the body -> NED quaternion.
  const double x = body_.q_wxyz[1];
  const double y = body_.q_wxyz[2];
  const double cos_tilt = std::clamp(1.0 - 2.0 * (x * x + y * y), -1.0, 1.0);
  std::uint32_t link_faults = 0;
  for (const StatusField f : {StatusField::CrcFailures, StatusField::BadHeaders, StatusField::SeqGaps, StatusField::TickGaps,
                              StatusField::BytesDropped, StatusField::QueueOverflows, StatusField::ReportOverflows}) {
    link_faults += field(status_, f);
  }
  std::string ground;
  if (show_touchdowns_) {
    std::array<char, kTouchdownTextBytes> text{};
    std::snprintf(text.data(), text.size(), " | touchdown %llu (last %.1f m/s)", static_cast<unsigned long long>(touchdowns_),
                  last_touchdown_m_s_);
    ground = text.data();
  }
  std::fprintf(stderr,
               "marv_gz_lockstep: hil t=%7.1f s | board %s %s | rc frames %u lq %.2f throttle %.2f | dshot %u %u %u %u | "
               "height %+.2f m vz %+.2f m/s tilt %.1f deg | margin min %.0f us lag %.1f ms | deadline misses %u link faults %u%s\n",
               t_s, arm_text(field(status_, StatusField::ArmingState)),
               receiver_frames_ == 0 || !have_angle_threshold_ ? "-" : rx_angle_ ? "ANGLE" : "ACRO", rx_frames_,
               static_cast<double>(rx_lq_),
               static_cast<double>(rx_throttle_), applied_[0], applied_[1], applied_[2], applied_[3], -body_.pos_ned_m[2],
               -body_.vel_ned_m_s[2], std::acos(cos_tilt) * kDegPerRad,
               reports_used_ == 0 ? 0.0 : static_cast<double>(min_margin_ns_) / static_cast<double>(kNsPerUs),
               static_cast<double>(lag_ns_) / static_cast<double>(kNsPerUs * kUsPerMs), field(status_, StatusField::DeadlineMisses),
               link_faults, ground.c_str());
}

HilView HilCommandSource::view() const {
  using fmt::StatusField;
  const auto sat = [](std::uint64_t v) {
    return static_cast<std::uint32_t>(std::min<std::uint64_t>(v, std::numeric_limits<std::uint32_t>::max()));
  };
  HilView v;
  v.arming_state = field(status_, StatusField::ArmingState);
  v.status_records = sat(status_records_);
  v.receiver_frames = sat(receiver_frames_);
  v.frame_counter = rx_frames_;
  v.mode = receiver_frames_ == 0 || !have_angle_threshold_ ? 0 : rx_angle_ ? 2 : 1;
  v.deadline_misses = field(status_, StatusField::DeadlineMisses);
  std::uint32_t link_faults = 0;
  for (const StatusField f : {StatusField::CrcFailures, StatusField::BadHeaders, StatusField::SeqGaps, StatusField::TickGaps,
                              StatusField::BytesDropped, StatusField::QueueOverflows, StatusField::ReportOverflows}) {
    link_faults += field(status_, f);
  }
  v.link_faults = link_faults;
  v.reports_used = sat(reports_used_);
  v.lag_events = sat(lag_events_);
  v.sticks = rx_sticks_;
  v.channels = rx_channels_;
  v.link_quality = rx_lq_;
  v.margin_min_us = reports_used_ == 0 ? 0.0F : static_cast<float>(static_cast<double>(min_margin_ns_) / static_cast<double>(kNsPerUs));
  v.margin_max_us = reports_used_ == 0 ? 0.0F : static_cast<float>(static_cast<double>(max_margin_ns_) / static_cast<double>(kNsPerUs));
  v.lag_us = static_cast<float>(static_cast<double>(lag_ns_) / static_cast<double>(kNsPerUs));
  return v;
}

void HilCommandSource::finish() {
  if (finished_) {
    return;
  }
  finished_ = true;
  if (fd_ >= 0 && failure_.empty() && started_) {
    const std::int64_t start = mono_ns();
    const auto wait_ns = static_cast<std::int64_t>(cfg_.drain_us) * kNsPerUs;
    while (mono_ns() - start < wait_ns && failure_.empty()) {
      pollfd p{fd_, POLLIN, 0};
      const std::int64_t left_ms = (wait_ns - (mono_ns() - start)) / (kNsPerUs * kUsPerMs);
      static_cast<void>(poll(&p, 1, static_cast<int>(std::max<std::int64_t>(left_ms, 1))));
      service();
    }
  }
  if (started_) {
    using fmt::StatusField;
    std::fprintf(stderr,
                 "marv_gz_lockstep: hil summary: N %u, F %u, rate divisor %u; sample records sent %llu, DShot reports received "
                 "%llu, used %llu; command margin (needed - arrived) min %.1f us, max %.1f us; send lateness max %.1f us; "
                 "lag absorbed %.1f us in %llu ticks (largest %.1f us); "
                 "uplink CRC failures %llu, sequence gaps %llu, unframed bytes %llu; board: arming %s, deadline misses %u, "
                 "ticks %u, receiver frames %llu%s%s\n",
                 cfg_.delay_ticks, cfg_.samples_per_record, rate_divisor_, static_cast<unsigned long long>(frames_sent_),
                 static_cast<unsigned long long>(reports_received_), static_cast<unsigned long long>(reports_used_),
                 static_cast<double>(min_margin_ns_) / static_cast<double>(kNsPerUs),
                 static_cast<double>(max_margin_ns_) / static_cast<double>(kNsPerUs),
                 static_cast<double>(max_send_late_ns_) / static_cast<double>(kNsPerUs),
                 static_cast<double>(lag_ns_) / static_cast<double>(kNsPerUs), static_cast<unsigned long long>(lag_events_),
                 static_cast<double>(max_lag_step_ns_) / static_cast<double>(kNsPerUs),
                 static_cast<unsigned long long>(up_crc_failures_), static_cast<unsigned long long>(up_seq_gaps_),
                 static_cast<unsigned long long>(up_unframed_), arm_text(field(status_, StatusField::ArmingState)),
                 field(status_, StatusField::DeadlineMisses), field(status_, StatusField::TicksRun),
                 static_cast<unsigned long long>(receiver_frames_), failure_.empty() ? "" : "; FAILED: ", failure_.c_str());
  }
  for (std::FILE** f : {&uplink_, &frames_, &reports_}) {
    if (*f != nullptr) {
      std::fclose(*f);
      *f = nullptr;
    }
  }
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

}  // namespace marv::gz
