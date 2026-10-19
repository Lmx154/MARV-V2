// The sim channel's socket and record writer (sim_telemetry.hpp has the record layout and the SC-1 argument).
#include "sim_telemetry.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <bit>
#include <cerrno>
#include <chrono>
#include <climits>
#include <cstdlib>
#include <cstring>
#include <limits>

#include "constants.hpp"

namespace marv::gz {
namespace {

// A little-endian writer into a fixed buffer; the record is built in place, so there is no allocation in the step.
class Writer {
 public:
  explicit Writer(std::array<std::uint8_t, kSimRecordBytes>& buf) : buf_(buf) {}
  void u8(std::uint8_t v) { buf_[pos_++] = v; }
  void u16(std::uint16_t v) {
    u8(static_cast<std::uint8_t>(v));
    u8(static_cast<std::uint8_t>(v >> CHAR_BIT));
  }
  void u32(std::uint32_t v) {
    u16(static_cast<std::uint16_t>(v));
    u16(static_cast<std::uint16_t>(v >> (CHAR_BIT * sizeof(std::uint16_t))));
  }
  void u64(std::uint64_t v) {
    u32(static_cast<std::uint32_t>(v));
    u32(static_cast<std::uint32_t>(v >> (CHAR_BIT * sizeof(std::uint32_t))));
  }
  void f32(float v) { u32(std::bit_cast<std::uint32_t>(v)); }
  void f64(double v) { u64(std::bit_cast<std::uint64_t>(v)); }
  std::size_t size() const { return pos_; }

 private:
  std::array<std::uint8_t, kSimRecordBytes>& buf_;
  std::size_t pos_ = 0;
};

std::int64_t wall_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace

SimTelemetry::~SimTelemetry() {
  if (fd_ >= 0) {
    ::close(fd_);
  }
}

std::string SimTelemetry::open(std::uint32_t port, std::uint64_t period_us) {
  if (port < 1 || port > std::numeric_limits<std::uint16_t>::max()) {
    return "<sim_telemetry_udp_port> is not a UDP port 1..65535";
  }
  if (period_us == 0) {
    return "<sim_telemetry_period_us> is 0";
  }
  fd_ = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (fd_ < 0) {
    return std::string("cannot open the sim channel socket: ") + std::strerror(errno);
  }
  dst_.sin_family = AF_INET;
  dst_.sin_port = htons(static_cast<std::uint16_t>(port));
  dst_.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  period_ns_ = period_us * kNanosecondsPerMicrosecond;
  return {};
}

void SimTelemetry::publish(const SimSample& s) {
  std::array<std::uint8_t, kSimRecordBytes> buf{};
  Writer w(buf);
  for (const char c : {'M', 'S', 'I', 'M'}) {
    w.u8(static_cast<std::uint8_t>(c));
  }
  w.u16(kSimRecordVersion);
  w.u16(static_cast<std::uint16_t>(kSimRecordBytes));
  w.u32(seq_);
  w.u32(s.flags);
  w.u64(s.sim_time_ns);
  w.u64(s.tick);
  const std::int64_t wall = wall_ns();
  double rtf = 0.0;
  if (have_prev_ && wall > prev_wall_ns_) {
    rtf = static_cast<double>(s.sim_time_ns - prev_sim_ns_) / static_cast<double>(wall - prev_wall_ns_);
  }
  w.f64(rtf);
  for (const double v : s.pos_ned_m) {
    w.f64(v);
  }
  for (const double v : s.vel_ned_m_s) {
    w.f64(v);
  }
  for (const double v : s.q_wxyz) {
    w.f64(v);
  }
  for (const double v : s.omega_frd_rad_s) {
    w.f64(v);
  }
  for (const double v : s.specific_force_frd_m_s2) {
    w.f64(v);
  }
  for (const double v : s.rotor_speed_rad_s) {
    w.f64(v);
  }
  for (const float v : s.imu_gyro_rad_s) {
    w.f32(v);
  }
  for (const float v : s.imu_accel_m_s2) {
    w.f32(v);
  }
  w.u32(s.imu_flags);
  for (const std::uint16_t v : s.dshot_commanded) {
    w.u16(v);
  }
  for (const std::uint16_t v : s.dshot_applied) {
    w.u16(v);
  }
  w.u32(s.touchdowns);
  w.f32(s.last_touchdown_down_m_s);
  const HilView& h = s.hil;
  for (const std::uint32_t v : {h.arming_state, h.status_records, h.receiver_frames, h.frame_counter, h.mode, h.deadline_misses,
                                h.link_faults, h.reports_used, h.lag_events}) {
    w.u32(v);
  }
  for (const float v : h.sticks) {
    w.f32(v);
  }
  for (const float v : h.channels) {
    w.f32(v);
  }
  for (const float v : {h.link_quality, h.margin_min_us, h.margin_max_us, h.lag_us}) {
    w.f32(v);
  }
  // The layout in sim_telemetry.hpp is exactly kSimRecordBytes; a mismatch is a defect of this file, not of a run.
  if (w.size() != kSimRecordBytes) {
    std::abort();
  }
  have_prev_ = true;
  prev_sim_ns_ = s.sim_time_ns;
  prev_wall_ns_ = wall;
  ++seq_;
  next_ns_ = (s.sim_time_ns / period_ns_ + 1) * period_ns_;
  const ssize_t n = ::sendto(fd_, buf.data(), buf.size(), MSG_DONTWAIT, reinterpret_cast<const sockaddr*>(&dst_), sizeof(dst_));
  if (n != static_cast<ssize_t>(buf.size())) {
    ++failures_;  // a refused or full socket loses the record; the run does not care
  }
}

}  // namespace marv::gz
