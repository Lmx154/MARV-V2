// Encoders of the ring records (format.hpp): each fills a fixed array with the record's bytes, kind first, every byte not
// named by the layout zero. Pure, wait-free, allocation-free: the tick context calls them.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "marv/log/format.hpp"
#include "marv/types/actuator.hpp"
#include "marv/types/imu_sample.hpp"
#include "marv/types/manual_control.hpp"
#include "marv/types/time.hpp"

namespace marv::log {

// The rate loop's tick as logged: the arming and gate states are the composition's own codes (opaque bytes here).
struct RateTickRecord {
  TimeUs now_us = 0;
  std::uint8_t arm_state = 0;
  std::uint8_t gate_state = 0;
  std::array<DshotValue, kQuadXMotors> dshot{};
};

[[nodiscard]] constexpr std::uint32_t low32(TimeUs t) noexcept { return static_cast<std::uint32_t>(t); }

[[nodiscard]] constexpr std::array<std::uint8_t, imu::kBytes> encode_imu(const ImuSample& s) noexcept {
  std::array<std::uint8_t, imu::kBytes> r{};
  r[kLeadOffKind] = static_cast<std::uint8_t>(Kind::Imu);
  put<std::uint16_t>(r, imu::kOffFlags, static_cast<std::uint16_t>(s.flags));
  put<std::uint32_t>(r, imu::kOffTime, low32(s.t_us));
  for (std::size_t i = 0; i < prim::kSpatialDim; ++i) {
    put_f32(r, imu::kOffGyro + i * sizeof(float), s.gyro_rad_s[i]);
    put_f32(r, imu::kOffAccel + i * sizeof(float), s.accel_m_s2[i]);
  }
  return r;
}

[[nodiscard]] constexpr std::array<std::uint8_t, kCellBytes> encode_rate_tick(const RateTickRecord& t) noexcept {
  std::array<std::uint8_t, kCellBytes> r{};
  r[kLeadOffKind] = static_cast<std::uint8_t>(Kind::RateTick);
  r[rate::kOffArm] = t.arm_state;
  r[rate::kOffGate] = t.gate_state;
  put<std::uint32_t>(r, rate::kOffTime, low32(t.now_us));
  for (std::size_t i = 0; i < rate::kMotors; ++i) {
    put<std::uint16_t>(r, rate::kOffDshot + i * sizeof(std::uint16_t), t.dshot[i].raw());
  }
  return r;
}

[[nodiscard]] constexpr std::array<std::uint8_t, manual::kBytes> encode_manual(const ManualControl& m) noexcept {
  std::array<std::uint8_t, manual::kBytes> r{};
  r[kLeadOffKind] = static_cast<std::uint8_t>(Kind::ManualControl);
  put<std::uint32_t>(r, manual::kOffFrames, m.frames);
  put<std::uint64_t>(r, manual::kOffTime, m.t_us);
  put_f32(r, manual::kOffSticks + static_cast<std::size_t>(Stick::Roll) * sizeof(float), m.roll);
  put_f32(r, manual::kOffSticks + static_cast<std::size_t>(Stick::Pitch) * sizeof(float), m.pitch);
  put_f32(r, manual::kOffSticks + static_cast<std::size_t>(Stick::Yaw) * sizeof(float), m.yaw);
  put_f32(r, manual::kOffSticks + static_cast<std::size_t>(Stick::Throttle) * sizeof(float), m.throttle);
  for (std::size_t i = 0; i < m.channels.size(); ++i) {
    put_f32(r, manual::kOffChannels + i * sizeof(float), m.channels[i]);
  }
  put_f32(r, manual::kOffLinkQuality, m.link_quality);
  put<std::uint32_t>(r, manual::kOffLinkTime, low32(m.link_t_us));
  return r;
}

[[nodiscard]] constexpr std::array<std::uint8_t, gap::kBytes> encode_gap(
    std::uint32_t t_lo, const std::array<std::uint32_t, kRingKinds>& dropped) noexcept {
  std::array<std::uint8_t, gap::kBytes> r{};
  r[kLeadOffKind] = static_cast<std::uint8_t>(Kind::Gap);
  put<std::uint32_t>(r, gap::kOffTime, t_lo);
  for (std::size_t i = 0; i < kRingKinds; ++i) {
    put<std::uint32_t>(r, gap::kOffDropped + i * sizeof(std::uint32_t), dropped[i]);
  }
  return r;
}

}  // namespace marv::log
