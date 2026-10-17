#pragma once

// The arming and link-loss failsafe state machine (quad spec section 4.0 M2; decisions 0040 and 0041). Pure: stepped once
// per rate tick with plain inputs, it reads no clock and no HAL. Time is the caller's uint64 microsecond `now_us`
// (hal_time_us() at the tick, core section 3).
//
// Link. A frame stamp later than `now_us`, or frames == 0, is no valid frame. The link is valid iff a frame exists and its
// age (now - stamp) is below `loss_us`. While armed: age < loss_us live sticks (Armed); loss_us <= age < hold_us the sticks
// of the last frame, unchanged (ArmedHold); hold_us <= age < disarm_us roll, pitch, yaw and throttle 0
// (ArmedFailsafeCentred); age >= disarm_us disarm, reason Failsafe. A link valid again before the disarm returns live
// sticks on that tick. With no valid frame while armed the age is unbounded, so the machine disarms (Failsafe).
//
// Arming happens on a rising edge of the arm switch (a tick at or above `arm_threshold` after a tick below it; the
// first tick counts as preceded by a high switch) when, on that tick: the link has been valid on every tick for at least
// `recovery_us` (the run starts on the first tick that sees the link valid), the throttle is below `throttle_min`, no IMU
// read deadline has been missed since init, the IMU samples are valid, no saturation bit was set since the previous
// tick, the gyro calibration is complete, the attitude estimate is valid, the vehicle is upright, and no motor test runs. A switch already high when the conditions become true does not arm; a refused edge needs
// the switch cycled again. Disarm on the same tick: switch below the threshold, Failsafe, an IMU deadline miss (latched
// until reset(): arming stays refused), a motor test running. Saturation refuses arming and never disarms. Invalid IMU
// samples refuse arming and do not disarm. A gyro calibration still running, an invalid estimate and a tilt above the small angle (quad spec section 4.0 M3,
// decision 0042; Betaflight's ARMING_DISABLED_CALIBRATING and ARMING_DISABLED_ANGLE) refuse arming and never disarm. A NaN switch or throttle is a low switch and a throttle that is not below the
// minimum.
//
// While disarmed the effective sticks and throttle are 0. The state names the cause of the last disarm on that tick,
// and otherwise the first refusing condition in the order ImuStale, MotorTest, Failsafe (set by a failsafe disarm,
// cleared on the first tick with a valid link), NoLink (no valid link, or valid for less than `recovery_us`),
// ImuInvalid, Saturated, Calibrating, EstimatorInvalid, NotUpright, ThrottleUp, then SwitchOff (switch low, everything else ready) or SwitchNotCycled (switch high
// without a rising edge, everything else ready). Boot is the state before the first step.
//
// Ground segment refusals (decision 0053, Luis 2026-10-17); both only refuse arming and neither disarms an armed vehicle.
// ConfigRefused: the config status given to init is not Ok (the stored config is corrupt, of an unknown schema, invalid, or
// not validated for this image; the reason is config_status()). Fixed at init. Maintenance: a maintenance request (the
// input `maintenance_request`) seen on a step that starts disarmed grants maintenance, which refuses arming until reset();
// there is no un-grant. A request on a step that starts armed is not granted and changes nothing. In the refusal order both
// come right after MotorTest, before Failsafe and NoLink, so they name themselves whatever the link and the sensors do:
// Maintenance first (a reboot is expected), then ConfigRefused.

#include <cstdint>

#include "marv/params/param_types.hpp"

namespace marv::arming {

enum class ArmState : std::uint8_t {
  Boot,
  SwitchOff,
  SwitchNotCycled,
  ThrottleUp,
  ImuStale,
  ImuInvalid,
  Saturated,
  NoLink,
  Failsafe,
  MotorTest,
  Armed,
  ArmedHold,
  ArmedFailsafeCentred,
  EstimatorInvalid,
  Calibrating,
  NotUpright,
  ConfigRefused,
  Maintenance,
};

// Why the config the machine was initialised with cannot be used (core 4, ground spec GS-11). Ok: the factory table or a
// stored config that passed every check.
enum class ConfigStatus : std::uint8_t {
  Ok,
  Corrupt,
  SchemaMismatch,
  Invalid,
  NotValidated,
};

[[nodiscard]] constexpr bool is_armed(ArmState s) noexcept {
  return s == ArmState::Armed || s == ArmState::ArmedHold || s == ArmState::ArmedFailsafeCentred;
}

template <class T>
struct ArmingConfig {
  std::uint64_t loss_us = 0;      // link lost at this age
  std::uint64_t hold_us = 0;      // sticks centred and throttle 0 from this age
  std::uint64_t disarm_us = 0;    // disarm (Failsafe) from this age
  std::uint64_t recovery_us = 0;  // valid link needed before arming
  T arm_threshold{};              // arm switch (normalized) at or above this is on
  T throttle_min{};               // throttle (normalized) must be below this to arm
};

// true iff 0 < loss_us < hold_us < disarm_us, recovery_us > 0 and both thresholds are finite.
template <class T>
[[nodiscard]] constexpr bool valid(const ArmingConfig<T>& c) noexcept {
  const bool finite = c.arm_threshold == c.arm_threshold && c.throttle_min == c.throttle_min;
  return finite && c.loss_us > 0 && c.loss_us < c.hold_us && c.hold_us < c.disarm_us && c.recovery_us > 0;
}

template <class T>
struct ArmingInputs {
  std::uint64_t now_us = 0;
  std::uint32_t frames = 0;  // valid RC frames received; 0 = none yet
  std::uint64_t last_frame_t_us = 0;
  T roll{};      // [-1, 1]
  T pitch{};     // [-1, 1]
  T yaw{};       // [-1, 1]
  T throttle{};  // [0, 1]
  T arm{};       // arm channel, [-1, 1]
  bool imu_deadline_missed = false;  // any missed IMU read deadline since init
  bool imu_samples_valid = false;    // both valid bits of the sample
  bool imu_saturated = false;        // any saturation bit since the previous rate tick
  bool gyro_calibrated = false;      // the boot gyro calibration is complete (marv::estimator::GyroCal::complete)
  bool estimator_valid = false;      // the attitude estimate is valid (marv::estimator::Mahony::valid)
  bool upright = false;              // the estimate's tilt is below the small angle
  bool motor_test_active = false;
  bool maintenance_request = false;  // a maintenance request is pending this step; granted only if the step starts disarmed
};

template <class T>
struct ArmingOutput {
  bool armed = false;
  T roll{};
  T pitch{};
  T yaw{};
  T throttle{};
  ArmState state = ArmState::Boot;
};

template <class T>
class Arming {
 public:
  void init(const ArmingConfig<T>& config, ConfigStatus config_status = ConfigStatus::Ok) noexcept {
    cfg_ = config;
    config_status_ = config_status;
    reset();
  }

  void reset() noexcept {
    armed_ = false;
    deadline_ = false;
    failsafe_ = false;
    link_run_ = false;
    link_run_start_us_ = 0;
    prev_switch_on_ = true;
    maintenance_ = false;
    state_ = ArmState::Boot;
  }

  [[nodiscard]] ArmingOutput<T> step(const ArmingInputs<T>& in) noexcept {
    if (in.imu_deadline_missed) {
      deadline_ = true;
    }
    const bool have_frame = in.frames > 0 && in.last_frame_t_us <= in.now_us;
    const std::uint64_t age = have_frame ? in.now_us - in.last_frame_t_us : kNoFrame;
    const bool link = have_frame && age < cfg_.loss_us;
    if (link) {
      if (!link_run_) {
        link_run_ = true;
        link_run_start_us_ = in.now_us;
      }
      failsafe_ = false;
    } else {
      link_run_ = false;
    }
    const bool switch_on = in.arm >= cfg_.arm_threshold;
    const bool edge = switch_on && !prev_switch_on_;
    prev_switch_on_ = switch_on;
    if (in.maintenance_request && !armed_) {
      maintenance_ = true;
    }

    if (armed_) {
      ArmState cause = ArmState::Armed;
      if (deadline_) {
        cause = ArmState::ImuStale;
      } else if (in.motor_test_active) {
        cause = ArmState::MotorTest;
      } else if (age >= cfg_.disarm_us) {
        cause = ArmState::Failsafe;
        failsafe_ = true;
      } else if (!switch_on) {
        cause = ArmState::SwitchOff;
      }
      if (cause == ArmState::Armed) {
        if (age < cfg_.loss_us) {
          return output(true, in, ArmState::Armed);
        }
        if (age < cfg_.hold_us) {
          return output(true, in, ArmState::ArmedHold);
        }
        return centred();
      }
      armed_ = false;
      return output(false, in, cause);
    }

    const bool recovered = link && in.now_us >= link_run_start_us_ && in.now_us - link_run_start_us_ >= cfg_.recovery_us;
    ArmState refusal = ArmState::Boot;
    if (deadline_) {
      refusal = ArmState::ImuStale;
    } else if (in.motor_test_active) {
      refusal = ArmState::MotorTest;
    } else if (maintenance_) {
      refusal = ArmState::Maintenance;
    } else if (config_status_ != ConfigStatus::Ok) {
      refusal = ArmState::ConfigRefused;
    } else if (failsafe_) {
      refusal = ArmState::Failsafe;
    } else if (!recovered) {
      refusal = ArmState::NoLink;
    } else if (!in.imu_samples_valid) {
      refusal = ArmState::ImuInvalid;
    } else if (in.imu_saturated) {
      refusal = ArmState::Saturated;
    } else if (!in.gyro_calibrated) {
      refusal = ArmState::Calibrating;
    } else if (!in.estimator_valid) {
      refusal = ArmState::EstimatorInvalid;
    } else if (!in.upright) {
      refusal = ArmState::NotUpright;
    } else if (!(in.throttle < cfg_.throttle_min)) {
      refusal = ArmState::ThrottleUp;
    } else if (!switch_on) {
      refusal = ArmState::SwitchOff;
    } else if (!edge) {
      refusal = ArmState::SwitchNotCycled;
    }
    if (refusal == ArmState::Boot) {
      armed_ = true;
      return output(true, in, ArmState::Armed);
    }
    return output(false, in, refusal);
  }

  [[nodiscard]] bool armed() const noexcept { return armed_; }
  [[nodiscard]] ArmState state() const noexcept { return state_; }
  [[nodiscard]] bool maintenance() const noexcept { return maintenance_; }
  [[nodiscard]] ConfigStatus config_status() const noexcept { return config_status_; }

 private:
  static constexpr std::uint64_t kNoFrame = ~std::uint64_t{0};

  [[nodiscard]] ArmingOutput<T> output(bool armed, const ArmingInputs<T>& in, ArmState s) noexcept {
    state_ = s;
    ArmingOutput<T> o;
    o.armed = armed;
    o.state = s;
    if (armed) {
      o.roll = in.roll;
      o.pitch = in.pitch;
      o.yaw = in.yaw;
      o.throttle = in.throttle;
    }
    return o;
  }

  [[nodiscard]] ArmingOutput<T> centred() noexcept {
    state_ = ArmState::ArmedFailsafeCentred;
    ArmingOutput<T> o;
    o.armed = true;
    o.state = state_;
    return o;
  }

  ArmingConfig<T> cfg_{};
  bool armed_ = false;
  bool deadline_ = false;
  bool failsafe_ = false;
  bool link_run_ = false;
  std::uint64_t link_run_start_us_ = 0;
  bool prev_switch_on_ = true;
  bool maintenance_ = false;
  ConfigStatus config_status_ = ConfigStatus::Ok;
  ArmState state_ = ArmState::Boot;
};

// The six parameters of the quad_fc register; panics (hal_panic) on a configuration that is not valid().
[[nodiscard]] ArmingConfig<float> load_config() noexcept;

// The same read from `table` (a candidate table of the set's kParamCount records) instead of the runtime table.
[[nodiscard]] ArmingConfig<float> load_config(ParamTable table) noexcept;

// nullptr if load_config(table) would not panic, else the text it panics with.
[[nodiscard]] const char* reject_reason(ParamTable table) noexcept;

extern template class Arming<float>;

}  // namespace marv::arming
