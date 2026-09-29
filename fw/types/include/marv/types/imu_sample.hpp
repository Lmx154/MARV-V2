#pragma once

#include <cstdint>
#include <type_traits>

#include "marv/prim/vec.hpp"
#include "marv/types/time.hpp"

namespace marv {

enum class ImuFlag : std::uint8_t {
  GyroSatX = 0,
  GyroSatY,
  GyroSatZ,
  AccelSatX,
  AccelSatY,
  AccelSatZ,
  GyroValid,
  AccelValid,
  TempValid,
  Count
};

[[nodiscard]] constexpr std::uint32_t imu_flag(ImuFlag f) noexcept {
  return std::uint32_t{1} << static_cast<unsigned>(f);
}

inline constexpr std::uint32_t kImuFlagsDefined = imu_flag(ImuFlag::Count) - 1;

// Samples are already in body FRD (mounting pose applied upstream). A saturation bit is set per FRD component if
// any contributing sensor axis hit full scale. If a valid bit is clear the field is exactly 0 and its saturation
// bits are clear. TempValid implies temp_k > 0.
struct ImuSample {
  TimeUs t_us;                   // acquisition stamp, hal_time_us() timebase
  prim::Vec3<float> gyro_rad_s;  // body angular rate, FRD axes, rad/s
  prim::Vec3<float> accel_m_s2;  // specific force at the IMU location, FRD axes, m/s^2
  float temp_k;                  // K
  std::uint32_t flags;           // imu_flag(...) bits; bits outside kImuFlagsDefined are 0
};

static_assert(std::is_trivially_copyable_v<ImuSample> && std::is_standard_layout_v<ImuSample>);

}  // namespace marv
