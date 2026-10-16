#include "marv/hal_target/lsm6dsv_decode.hpp"

#include <array>
#include <climits>
#include <cstddef>
#include <cstdint>
#include <limits>

#include <parts/lsm6dsv.hpp>

#include "marv/prim/constants.hpp"
#include "marv/prim/vec.hpp"
#include "marv/types/imu_sample.hpp"
#include "marv/types/time.hpp"

namespace marv::hal_target {
namespace {

// Byte offset of the X low byte in a FIFO word (after the tag byte, DS13476 §6.10.8).
constexpr std::size_t kDataOffset = 1;

[[nodiscard]] std::int16_t le16(std::uint8_t lo, std::uint8_t hi) noexcept {
  return static_cast<std::int16_t>(static_cast<std::uint16_t>(lo | (hi << CHAR_BIT)));
}

[[nodiscard]] bool at_limit(std::int16_t v) noexcept {
  return v == std::numeric_limits<std::int16_t>::max() || v == std::numeric_limits<std::int16_t>::min();
}

// Writes the FRD vector of chip values `raw` (scale per LSB) and returns its saturation bits, the X bit at `sat_x`.
std::uint32_t to_frd(const ImuMounting& m, const std::array<std::int16_t, kImuAxes>& raw, float scale,
                     prim::Vec3<float>& out, ImuFlag sat_x) noexcept {
  std::uint32_t flags = 0;
  for (std::size_t i = 0; i < kImuAxes; ++i) {
    const std::int16_t v = raw[m.chip_axis[i]];
    out[i] = static_cast<float>(m.sign[i]) * static_cast<float>(v) * scale;
    if (at_limit(v)) {
      flags |= imu_flag(static_cast<ImuFlag>(static_cast<unsigned>(sat_x) + i));
    }
  }
  return flags;
}

}  // namespace

bool valid(const ImuMounting& m) noexcept {
  std::array<bool, kImuAxes> seen{};
  for (std::size_t i = 0; i < kImuAxes; ++i) {
    if (m.chip_axis[i] >= kImuAxes || seen[m.chip_axis[i]] || (m.sign[i] != 1 && m.sign[i] != -1)) {
      return false;
    }
    seen[m.chip_axis[i]] = true;
  }
  return true;
}

Lsm6dsvScales lsm6dsv_scales() noexcept {
  Lsm6dsvScales s;
  s.gyro_rad_s_per_lsb = static_cast<float>(lsm6dsv::kGyroDpsPerLsb * prim::kPi / prim::kDegreesPerHalfTurn);
  s.accel_m_s2_per_lsb = static_cast<float>(lsm6dsv::kAccelGPerLsb * prim::kStandardGravity);
  s.temp_k_per_lsb = static_cast<float>(1.0 / lsm6dsv::kTempLsbPerDegC);
  s.temp_k_at_zero_lsb = static_cast<float>(lsm6dsv::kTempDegCAtZeroLsb + prim::kKelvinAtZeroCelsius);
  return s;
}

void Lsm6dsvAssembler::set_temperature(std::int16_t raw) noexcept {
  temp_k_ = scales_.temp_k_at_zero_lsb + static_cast<float>(raw) * scales_.temp_k_per_lsb;
  have_temp_ = true;
}

ImuSample Lsm6dsvAssembler::complete() noexcept {
  ImuSample s{};
  if (have_gyro_) {
    s.flags |= imu_flag(ImuFlag::GyroValid) |
               to_frd(mount_, gyro_, scales_.gyro_rad_s_per_lsb, s.gyro_rad_s, ImuFlag::GyroSatX);
  }
  if (have_accel_) {
    s.flags |= imu_flag(ImuFlag::AccelValid) |
               to_frd(mount_, accel_, scales_.accel_m_s2_per_lsb, s.accel_m_s2, ImuFlag::AccelSatX);
  }
  if (have_temp_) {
    s.temp_k = temp_k_;
    s.flags |= imu_flag(ImuFlag::TempValid);
  }
  have_gyro_ = false;
  have_accel_ = false;
  return s;
}

WordResult Lsm6dsvAssembler::push(const FifoWord& w, ImuSample& out) noexcept {
  const auto sensor = static_cast<std::uint8_t>(w[0] >> lsm6dsv::kTagSensorShift);
  const auto slot = static_cast<std::uint8_t>((w[0] >> lsm6dsv::kTagCntShift) & lsm6dsv::kTagCntMask);
  const bool gyro = sensor == lsm6dsv::kTagGyroNc;
  if (!gyro && sensor != lsm6dsv::kTagAccelNc) {
    return WordResult::BadTag;
  }
  WordResult r = WordResult::Held;
  const bool held = have_gyro_ || have_accel_;
  if (held && (slot != slot_ || (gyro ? have_gyro_ : have_accel_))) {
    out = complete();
    r = WordResult::Sample;
  }
  std::array<std::int16_t, kImuAxes>& dst = gyro ? gyro_ : accel_;
  for (std::size_t a = 0; a < kImuAxes; ++a) {
    dst[a] = le16(w[kDataOffset + 2 * a], w[kDataOffset + 2 * a + 1]);
  }
  (gyro ? have_gyro_ : have_accel_) = true;
  slot_ = slot;
  if (have_gyro_ && have_accel_) {
    out = complete();
    r = WordResult::Sample;
  }
  return r;
}

TickPeriodUs tick_period_from_trim(std::int8_t freq_fine) noexcept {
  const auto us_per_s = static_cast<std::int64_t>(prim::kMicrosecondsPerSecond);
  TickPeriodUs t;
  t.num = us_per_s * lsm6dsv::kFreqFineStepDen;
  t.den = static_cast<std::int64_t>(lsm6dsv::kOdrHz) *
          (lsm6dsv::kFreqFineStepDen + lsm6dsv::kFreqFineStepNum * static_cast<std::int64_t>(freq_fine));
  return t;
}

bool same_period(std::int64_t num_a, std::int64_t den_a, std::int64_t num_b, std::int64_t den_b) noexcept {
  return num_a >= 1 && den_a >= 1 && num_b >= 1 && den_b >= 1 && num_a * den_b == num_b * den_a;
}

TimeUs imu_read_deadline_us(std::int64_t period_num_us, std::int64_t period_den) noexcept {
  if (period_num_us < 1 || period_den < 1) {
    return 0;
  }
  return static_cast<TimeUs>((static_cast<std::int64_t>(kFifoDepthSamples) * period_num_us) / period_den);
}

TimeUs ImuStamper::stamp(TimeUs t_irq, std::size_t back) noexcept {
  const auto us_per_s = static_cast<std::uint64_t>(prim::kMicrosecondsPerSecond);
  const std::uint64_t step = (2 * static_cast<std::uint64_t>(back) * us_per_s + odr_hz_) / (2 * odr_hz_);
  TimeUs t = step <= t_irq ? t_irq - step : 0;
  if (have_last_ && t <= last_) {
    t = last_ + 1;
    ++clamps_;
  }
  last_ = t;
  have_last_ = true;
  return t;
}

}  // namespace marv::hal_target
