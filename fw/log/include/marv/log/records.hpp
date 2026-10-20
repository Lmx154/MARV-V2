// Encoders of the ring records (format.hpp): each fills a fixed array with the record's bytes, kind first, every byte not
// named by the layout zero. Pure, wait-free, allocation-free: the tick context calls them.
#pragma once

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>

#include "marv/log/format.hpp"
#include "marv/types/actuator.hpp"
#include "marv/types/imu_sample.hpp"
#include "marv/types/manual_control.hpp"
#include "marv/prim/vec.hpp"
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

// What the ImuRaw records need to be turned back into SI (the HeaderImuRaw record): the SI per count of each sensor at the
// full scales it runs at, and the mounting sign of each FRD axis (1 or -1). FRD axis i = sign[i] x count[i] x scale.
struct ImuRawScales {
  float gyro_rad_s_per_lsb = 0.0F;
  float accel_m_s2_per_lsb = 0.0F;
  std::array<std::int8_t, prim::kSpatialDim> sign{};
};

// What an ImuRaw record holds, as the Imu record would: flags in the imu_flag bits (the saturation bits are the counts at an
// int16 limit), the SI vectors, and the low 16 bits of the time.
struct ImuRawValue {
  std::uint32_t flags = 0;
  prim::Vec3<float> gyro_rad_s{};
  prim::Vec3<float> accel_m_s2{};
  std::uint16_t t_lo16 = 0;
};

namespace detail {
[[nodiscard]] constexpr bool at_int16_limit(std::int16_t v) noexcept {
  return v == std::numeric_limits<std::int16_t>::max() || v == std::numeric_limits<std::int16_t>::min();
}

// The lsm6dsv_decode.cpp rule: sign x count x scale, binary32, one rounding.
[[nodiscard]] constexpr float scaled(std::int8_t sign, std::int16_t count, float scale) noexcept {
  return static_cast<float>(sign) * static_cast<float>(count) * scale;
}

// The count whose scaled value `v` is (the nearest integer to sign x v / scale), if it is an int16. Named so that it does not
// collide with the Pico SDK's count_of macro; the open interval keeps the rounding cast inside int16.
[[nodiscard]] constexpr std::optional<std::int16_t> nearest_count(float v, std::int8_t sign, float scale) noexcept {
  const float x = static_cast<float>(sign) * v / scale;
  const float lo = static_cast<float>(std::numeric_limits<std::int16_t>::min());
  const float hi = static_cast<float>(std::numeric_limits<std::int16_t>::max());
  if (!(x > lo - 0.5F && x < hi + 0.5F)) {
    return std::nullopt;
  }
  return static_cast<std::int16_t>(x >= 0.0F ? x + 0.5F : x - 0.5F);
}
}  // namespace detail

// The inverse of encode_imu_raw: `r` is an ImuRaw record (imu_raw::kBytes). A sensor whose valid bit is clear reads as +0.
[[nodiscard]] constexpr ImuRawValue decode_imu_raw(std::span<const std::uint8_t> r, const ImuRawScales& sc) noexcept {
  ImuRawValue v;
  const std::uint8_t f = r[imu_raw::kOffFlags];
  const bool gyro = (f & imu_raw::kFlagGyroValid) != 0;
  const bool accel = (f & imu_raw::kFlagAccelValid) != 0;
  v.t_lo16 = get<std::uint16_t>(r, imu_raw::kOffTime);
  if (gyro) {
    v.flags |= imu_flag(ImuFlag::GyroValid);
  }
  if (accel) {
    v.flags |= imu_flag(ImuFlag::AccelValid);
  }
  if ((f & imu_raw::kFlagTempValid) != 0) {
    v.flags |= imu_flag(ImuFlag::TempValid);
  }
  for (std::size_t i = 0; i < prim::kSpatialDim; ++i) {
    const auto g = static_cast<std::int16_t>(get<std::uint16_t>(r, imu_raw::kOffGyro + i * sizeof(std::int16_t)));
    const auto a = static_cast<std::int16_t>(get<std::uint16_t>(r, imu_raw::kOffAccel + i * sizeof(std::int16_t)));
    if (gyro) {
      v.gyro_rad_s[i] = detail::scaled(sc.sign[i], g, sc.gyro_rad_s_per_lsb);
      if (detail::at_int16_limit(g)) {
        v.flags |= imu_flag(static_cast<ImuFlag>(static_cast<unsigned>(ImuFlag::GyroSatX) + i));
      }
    }
    if (accel) {
      v.accel_m_s2[i] = detail::scaled(sc.sign[i], a, sc.accel_m_s2_per_lsb);
      if (detail::at_int16_limit(a)) {
        v.flags |= imu_flag(static_cast<ImuFlag>(static_cast<unsigned>(ImuFlag::AccelSatX) + i));
      }
    }
  }
  return v;
}

// The ImuRaw record of `s`, or nothing if the record would not give `s` back bit for bit (the sample is not a whole number of
// counts of these scales, or its flags are not those the counts imply): the caller then writes the Imu record instead.
[[nodiscard]] constexpr std::optional<std::array<std::uint8_t, imu_raw::kBytes>> encode_imu_raw(
    const ImuSample& s, const ImuRawScales& sc) noexcept {
  if (!(sc.gyro_rad_s_per_lsb > 0.0F) || !(sc.accel_m_s2_per_lsb > 0.0F)) {
    return std::nullopt;
  }
  std::array<std::uint8_t, imu_raw::kBytes> r{};
  r[kLeadOffKind] = static_cast<std::uint8_t>(Kind::ImuRaw);
  std::uint8_t f = 0;
  if ((s.flags & imu_flag(ImuFlag::GyroValid)) != 0) {
    f |= imu_raw::kFlagGyroValid;
  }
  if ((s.flags & imu_flag(ImuFlag::AccelValid)) != 0) {
    f |= imu_raw::kFlagAccelValid;
  }
  if ((s.flags & imu_flag(ImuFlag::TempValid)) != 0) {
    f |= imu_raw::kFlagTempValid;
  }
  r[imu_raw::kOffFlags] = f;
  put<std::uint16_t>(r, imu_raw::kOffTime, static_cast<std::uint16_t>(s.t_us));
  for (std::size_t i = 0; i < prim::kSpatialDim; ++i) {
    if (sc.sign[i] != 1 && sc.sign[i] != -1) {
      return std::nullopt;
    }
    if ((f & imu_raw::kFlagGyroValid) != 0) {
      const auto g = detail::nearest_count(s.gyro_rad_s[i], sc.sign[i], sc.gyro_rad_s_per_lsb);
      if (!g) {
        return std::nullopt;
      }
      put<std::uint16_t>(r, imu_raw::kOffGyro + i * sizeof(std::int16_t), static_cast<std::uint16_t>(*g));
    }
    if ((f & imu_raw::kFlagAccelValid) != 0) {
      const auto a = detail::nearest_count(s.accel_m_s2[i], sc.sign[i], sc.accel_m_s2_per_lsb);
      if (!a) {
        return std::nullopt;
      }
      put<std::uint16_t>(r, imu_raw::kOffAccel + i * sizeof(std::int16_t), static_cast<std::uint16_t>(*a));
    }
  }
  const ImuRawValue back = decode_imu_raw(r, sc);
  if (back.flags != s.flags) {
    return std::nullopt;
  }
  for (std::size_t i = 0; i < prim::kSpatialDim; ++i) {
    if (std::bit_cast<std::uint32_t>(back.gyro_rad_s[i]) != std::bit_cast<std::uint32_t>(s.gyro_rad_s[i]) ||
        std::bit_cast<std::uint32_t>(back.accel_m_s2[i]) != std::bit_cast<std::uint32_t>(s.accel_m_s2[i])) {
      return std::nullopt;
    }
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
