#pragma once

#include <array>
#include <cstdint>
#include <type_traits>

#include "marv/prim/quat.hpp"
#include "marv/prim/vec.hpp"
#include "marv/types/actuator.hpp"
#include "marv/types/manual_control.hpp"
#include "marv/types/time.hpp"

namespace marv {

// One telemetry snapshot of the flight composition (decision 0053 Q4; ground-segment unit 11): what one rate tick read,
// computed and wrote, published by the tick through a seqlock (marv/types/seqlock.hpp) once per rate execution and read by
// the ground link in the background loop. SI units, FRD body axes, quaternions body to NED (core 3). Fixed size, no
// pointers.
struct Telemetry {
  TimeUs time_us = 0;    // the rate tick's clock (hal_time_us), the arming machine's now
  TimeUs sample_us = 0;  // the stamp of the tick's IMU sample
  std::uint64_t rate_executions = 0;

  // The tick's IMU sample as the estimator and the gyro chain received it: the gyro less the boot calibration's bias (the
  // raw gyro until the calibration completes), the accelerometer and temperature unchanged.
  prim::Vec3<float> gyro_rad_s{};
  prim::Vec3<float> accel_m_s2{};
  float temp_k = 0.0F;
  std::uint32_t imu_flags = 0;  // the sample's imu_flag bits
  prim::Vec3<float> rate_gyro_rad_s{};  // the rate loop's input: the gyro chain's output

  // The estimators: Mahony always; the ESKF in the image that runs it beside Mahony (eskf_present), else zero.
  prim::Quat<float> q_mahony{};
  prim::Quat<float> q_eskf{};
  prim::Vec3<float> gyro_bias_cal{};  // the boot calibration's bias (zero until it completes)
  prim::Vec3<float> gyro_bias_mahony{};
  prim::Vec3<float> gyro_bias_eskf{};

  // The setpoints of the rate execution: the rate setpoint (or the attitude law's output in angle mode), the attitude
  // setpoint (angle mode only, else identity) and the collective thrust with the largest the throttle commands.
  prim::Vec3<float> rate_setpoint_rad_s{};
  prim::Quat<float> q_setpoint{};
  float thrust_n = 0.0F;
  float thrust_max_n = 0.0F;

  std::array<std::uint16_t, kQuadXMotors> motor_dshot{};  // the values written (DshotValue::raw), index = motor - 1
  ManualControl manual{};                                 // the pilot's input as the tick read it

  std::uint32_t rate_faults = 0;
  std::uint32_t rate_timing_faults = 0;
  std::uint32_t attitude_faults = 0;
  std::uint32_t attitude_timing_faults = 0;
  std::uint32_t imu_deadline_misses = 0;
  std::uint8_t arm_state = 0;   // arming::ArmState
  std::uint8_t motor_gate = 0;  // 0 Disarmed, 1 Armed, 2 MotorTest

  bool angle_mode = false;       // the attitude law ran
  bool airmode = false;          // the airmode latch
  bool iterm_held = false;       // armed below the minimum throttle before the latch: the integrator was zeroed
  bool gyro_calibrated = false;
  bool estimator_valid = false;  // the estimate in use
  bool upright = false;
  bool imu_invalid = false;      // since the previous rate tick
  bool imu_saturated = false;    // since the previous rate tick
  bool rate_fault_active = false;
  bool rate_fault_latched = false;
  bool mahony_valid = false;
  bool eskf_present = false;
  bool eskf_valid = false;
  bool eskf_in_use = false;
};

static_assert(std::is_trivially_copyable_v<Telemetry>);

}  // namespace marv
