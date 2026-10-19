#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "marv/prim/constants.hpp"
#include "marv/prim/quat.hpp"
#include "marv/types/actuator.hpp"
#include "marv/types/time.hpp"

namespace marv {

// The layout version of Telemetry: a consumer that stores or sends the bytes (the ground link, an onboard log) records it
// and refuses a version it does not know. Changed with any change of the fields, their order or their meaning.
inline constexpr std::uint32_t kTelemetryVersion = 1;

// Components of a quaternion as Telemetry stores it, w x y z (prim::Quat's layout: four scalars, no padding).
inline constexpr std::size_t kQuatComponents = sizeof(prim::Quat<float>) / sizeof(float);

// Bits of Telemetry::flags: bit f is telemetry_flag(f).
enum class TelemetryFlag : std::uint8_t {
  AngleMode,         // the rate tick ran the attitude law (armed, angle mode selected)
  Airmode,           // the airmode latch is set
  ItermHeld,         // armed below the minimum throttle before the latch: the rate integrator was zeroed
  GyroCalibrated,    // the boot gyro calibration has completed
  EstimatorValid,    // the estimate in use is valid
  Upright,           // the arming machine's upright check holds
  ImuInvalid,        // a sample since the previous rate tick lacked a valid gyro or accelerometer
  ImuSaturated,      // a sample since the previous rate tick was saturated
  RateFaultActive,   // the rate execution was a fault (zero torque)
  RateFaultLatched,  // the rate loop's fault latch is set
  MahonyValid,       // Mahony's estimate is valid
  EskfPresent,       // the image steps the ESKF beside Mahony; without it the ESKF fields are zero
  EskfValid,         // the ESKF's estimate is valid
  EskfInUse,         // the ESKF, not Mahony, is the estimate in use
  Count
};

[[nodiscard]] constexpr std::uint32_t telemetry_flag(TelemetryFlag f) noexcept {
  return std::uint32_t{1} << static_cast<unsigned>(f);
}

// One telemetry snapshot of the flight composition (decision 0053 Q4; ground-segment unit 11): what one rate tick read,
// computed and wrote, published by the tick through a seqlock (marv/types/seqlock.hpp) once per rate execution and read
// outside the tick (the ground link in the background loop; later an onboard log). Self-contained and fixed: plain
// fixed-width fields and arrays only (no pointer, no bool, no nested type), ordered by size so there is no padding
// (checked below), little-endian on every MARV target. version is kTelemetryVersion; sequence counts the snapshots
// published since init (1 for the first), so a consumer that sees it skip has missed snapshots. SI units, FRD body axes,
// quaternions body to NED as w x y z (core 3), motor arrays indexed by logical motor - 1.
struct Telemetry {
  std::uint64_t sequence = 0;
  TimeUs time_us = 0;    // the rate tick's clock (hal_time_us), the arming machine's now
  TimeUs sample_us = 0;  // the stamp of the tick's IMU sample
  std::uint64_t rate_executions = 0;
  TimeUs rc_t_us = 0;       // the stamp of the newest valid RC frame (ManualControl::t_us)
  TimeUs rc_link_t_us = 0;  // the stamp of the newest link-statistics frame (ManualControl::link_t_us); 0 before any
  TimeUs rotor_t_us = 0;    // the rotor-speed sample's stamp

  std::uint32_t version = kTelemetryVersion;
  std::uint32_t imu_flags = 0;    // the sample's imu_flag bits
  std::uint32_t flags = 0;        // telemetry_flag bits
  std::uint32_t rc_frames = 0;    // valid RC frames since init (ManualControl::frames; 0: none yet)
  std::uint32_t rotor_flags = 0;  // rotor_speed_flag bits: which rotor speeds are valid
  std::uint32_t rate_faults = 0;
  std::uint32_t rate_timing_faults = 0;
  std::uint32_t attitude_faults = 0;
  std::uint32_t attitude_timing_faults = 0;
  std::uint32_t imu_deadline_misses = 0;

  // The tick's IMU sample as the estimator and the gyro chain received it: the gyro less the boot calibration's bias (the
  // raw gyro until the calibration completes), the accelerometer and temperature unchanged.
  std::array<float, prim::kSpatialDim> gyro_rad_s{};
  std::array<float, prim::kSpatialDim> accel_m_s2{};
  float temp_k = 0.0F;
  std::array<float, prim::kSpatialDim> rate_gyro_rad_s{};  // the rate loop's input: the gyro chain's output

  // The estimators: Mahony always; the ESKF in the image that steps it (EskfPresent), else zero.
  std::array<float, kQuatComponents> q_mahony{};
  std::array<float, kQuatComponents> q_eskf{};
  std::array<float, prim::kSpatialDim> gyro_bias_cal{};  // the boot calibration's bias (zero until it completes)
  std::array<float, prim::kSpatialDim> gyro_bias_mahony{};
  std::array<float, prim::kSpatialDim> gyro_bias_eskf{};

  // The rate execution's setpoints: the rate setpoint (in angle mode the attitude law's output), the attitude setpoint
  // (angle mode only, else identity) and the collective thrust with the largest the throttle commands.
  std::array<float, prim::kSpatialDim> rate_setpoint_rad_s{};
  std::array<float, kQuatComponents> q_setpoint{};
  float thrust_n = 0.0F;
  float thrust_max_n = 0.0F;

  // The pilot's input as the tick read it (ManualControl): sticks, channels and link quality.
  float rc_roll = 0.0F;
  float rc_pitch = 0.0F;
  float rc_yaw = 0.0F;
  float rc_throttle = 0.0F;
  std::array<float, prim::kCrsfRcChannels> rc_channels{};
  float rc_link_quality = 0.0F;

  // The rotor speeds (bidirectional DShot eRPM through hal_rotor_speed), rad/s; 0 where rotor_flags has no valid bit.
  std::array<float, kQuadXMotors> rotor_speed_rad_s{};

  std::array<std::uint16_t, kQuadXMotors> motor_dshot{};  // the DShot values written (DshotValue::raw); 0 is stop
  std::uint16_t reserved = 0;                             // 0; pads the layout to a multiple of 8 bytes
  std::uint8_t arm_state = 0;                             // arming::ArmState
  std::uint8_t motor_gate = 0;                            // 0 Disarmed, 1 Armed, 2 MotorTest
};

static_assert(std::is_trivially_copyable_v<Telemetry> && std::is_standard_layout_v<Telemetry>);
// No padding: none at the two places the field size steps down (inside a run of one size there can be none), none at the end.
static_assert(offsetof(Telemetry, version) == offsetof(Telemetry, rotor_t_us) + sizeof(Telemetry::rotor_t_us));
static_assert(offsetof(Telemetry, motor_dshot) ==
              offsetof(Telemetry, rotor_speed_rad_s) + sizeof(Telemetry::rotor_speed_rad_s));
static_assert(offsetof(Telemetry, motor_gate) + sizeof(Telemetry::motor_gate) == sizeof(Telemetry),
              "Telemetry has tail padding: the fields no longer fill whole 8-byte words");
static_assert(sizeof(Telemetry) % alignof(std::uint64_t) == 0);

}  // namespace marv
