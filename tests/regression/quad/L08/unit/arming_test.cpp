// The arming and link-loss failsafe state machine (fw/arming; M2 P1; decisions 0040 and 0041). Every transition, each timing
// boundary at -1 and 0 us, the switch-cycle requirement, and the registered parameter values. The inputs are plain
// structs, so the test needs no HAL beyond the parameter table. Nothing else covers the machine: the CRSF decoder test
// (crsf_test.cpp) stops at raw channels and the quad_fc gate test (L09) has no arming.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>

#include <marv/arming/arming.hpp>
#include <marv/params/param.hpp>

namespace marv::arming {
namespace {

// The safety numbers as Luis ruled them (decision 0041; the thresholds are the contract (us - 1500) / 512 and
// (us - 988) / 1024 at 1700 us and 1050 us).
constexpr std::uint64_t kLoss = 100000;
constexpr std::uint64_t kHold = 300000;
constexpr std::uint64_t kDisarm = 1500000;
constexpr std::uint64_t kRecovery = 500000;
constexpr float kArmThreshold = 0.390625F;
constexpr float kThrottleMin = 0.060546875F;

constexpr std::uint64_t kT0 = 1000000;
constexpr float kOn = 1.0F;
constexpr float kOff = -1.0F;

void init_params_once() {
  static const bool ok = params_init(param_defaults());
  ASSERT_TRUE(ok);
}

ArmingConfig<float> test_config() {
  ArmingConfig<float> c;
  c.loss_us = kLoss;
  c.hold_us = kHold;
  c.disarm_us = kDisarm;
  c.recovery_us = kRecovery;
  c.arm_threshold = kArmThreshold;
  c.throttle_min = kThrottleMin;
  return c;
}

// Calls step() with a mutable input record.
class Rig {
 public:
  Rig() {
    machine_.init(test_config());
    in_.imu_samples_valid = true;
    in_.estimator_valid = true;
    in_.arm = kOff;
    in_.throttle = 0.0F;
  }

  // A frame received at `t` carrying the sticks currently in the record.
  void frame(std::uint64_t t) {
    ++in_.frames;
    in_.last_frame_t_us = t;
  }

  void sticks(float roll, float pitch, float yaw, float throttle) {
    in_.roll = roll;
    in_.pitch = pitch;
    in_.yaw = yaw;
    in_.throttle = throttle;
  }

  // A tick at `now` with no new frame.
  ArmingOutput<float> at(std::uint64_t now) {
    in_.now_us = now;
    return machine_.step(in_);
  }

  // A frame and a tick at `now`.
  ArmingOutput<float> live(std::uint64_t now) {
    frame(now);
    return at(now);
  }

  // A frame and a tick every 10 ms from `from` to `to` (to - from a multiple of 10 ms); returns the last output.
  ArmingOutput<float> run(std::uint64_t from, std::uint64_t to) {
    ArmingOutput<float> o;
    for (std::uint64_t t = from; t <= to; t += kStepUs) {
      o = live(t);
    }
    return o;
  }

  // The link valid from kT0 for kRecovery with the switch low and the throttle at 0, then the switch on at
  // kT0 + kRecovery: armed on that tick, the last frame at kT0 + kRecovery.
  ArmingOutput<float> arm() {
    in_.arm = kOff;
    (void)run(kT0, kT0 + kRecovery - kStepUs);
    in_.arm = kOn;
    sticks(0.25F, -0.5F, 0.125F, 0.0F);
    return live(kT0 + kRecovery);
  }

  static constexpr std::uint64_t kStepUs = 10000;
  static constexpr std::uint64_t kArmedAt = kT0 + kRecovery;
  ArmingInputs<float> in_{};
  Arming<float> machine_{};
};

void expect_zero_sticks(const ArmingOutput<float>& o) {
  EXPECT_EQ(o.roll, 0.0F);
  EXPECT_EQ(o.pitch, 0.0F);
  EXPECT_EQ(o.yaw, 0.0F);
  EXPECT_EQ(o.throttle, 0.0F);
}

TEST(Arming, RegisteredParametersAreLuisValues) {
  init_params_once();
  const ArmingConfig<float> c = load_config();
  EXPECT_EQ(c.loss_us, kLoss);
  EXPECT_EQ(c.hold_us, kHold);
  EXPECT_EQ(c.disarm_us, kDisarm);
  EXPECT_EQ(c.recovery_us, kRecovery);
  EXPECT_EQ(c.arm_threshold, kArmThreshold);
  EXPECT_EQ(c.throttle_min, kThrottleMin);
  EXPECT_TRUE(valid(c));
}

TEST(Arming, ConfigValidity) {
  EXPECT_TRUE(valid(test_config()));
  ArmingConfig<float> c = test_config();
  c.loss_us = 0;
  EXPECT_FALSE(valid(c));
  c = test_config();
  c.hold_us = c.loss_us;
  EXPECT_FALSE(valid(c));
  c = test_config();
  c.disarm_us = c.hold_us;
  EXPECT_FALSE(valid(c));
  c = test_config();
  c.recovery_us = 0;
  EXPECT_FALSE(valid(c));
  c = test_config();
  c.arm_threshold = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(valid(c));
  c = test_config();
  c.throttle_min = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(valid(c));
}

TEST(Arming, BootStateAndNoFrameRefuses) {
  Rig r;
  EXPECT_EQ(r.machine_.state(), ArmState::Boot);
  EXPECT_FALSE(r.machine_.armed());
  // frames == 0: a rising edge with everything else ready is refused as NoLink, however long it lasts.
  (void)r.at(kT0);
  r.in_.arm = kOn;
  for (std::uint64_t t = kT0 + 1; t <= kT0 + 2 * kRecovery; t += 100000) {
    const ArmingOutput<float> o = r.at(t);
    EXPECT_FALSE(o.armed);
    EXPECT_EQ(o.state, ArmState::NoLink);
  }
}

TEST(Arming, ArmsOnRisingEdgeAndPassesSticksThrough) {
  Rig r;
  const ArmingOutput<float> o = r.arm();
  EXPECT_TRUE(o.armed);
  EXPECT_EQ(o.state, ArmState::Armed);
  EXPECT_EQ(o.roll, 0.25F);
  EXPECT_EQ(o.pitch, -0.5F);
  EXPECT_EQ(o.yaw, 0.125F);
  EXPECT_EQ(o.throttle, 0.0F);
  EXPECT_TRUE(r.machine_.armed());
  EXPECT_EQ(r.machine_.state(), ArmState::Armed);
  // The switch stays on and throttle up: still armed with live sticks.
  r.sticks(0.0F, 0.0F, 0.0F, 0.75F);
  const ArmingOutput<float> up = r.live(r.kArmedAt + Rig::kStepUs);
  EXPECT_TRUE(up.armed);
  EXPECT_EQ(up.throttle, 0.75F);
}

TEST(Arming, DisarmedOutputsAreZero) {
  Rig r;
  r.sticks(0.5F, 0.5F, 0.5F, 0.5F);
  const ArmingOutput<float> o = r.live(kT0);
  EXPECT_FALSE(o.armed);
  expect_zero_sticks(o);
  EXPECT_EQ(o.state, ArmState::NoLink);
}

TEST(Arming, RecoveryBoundary) {
  // The valid-link run starts on the first tick that sees the link valid (kT0).
  {
    Rig r;  // an edge at recovery - 1 us: refused as NoLink
    (void)r.run(kT0, kT0 + kRecovery - Rig::kStepUs);
    r.in_.arm = kOn;
    const ArmingOutput<float> o = r.live(kT0 + kRecovery - 1);
    EXPECT_FALSE(o.armed);
    EXPECT_EQ(o.state, ArmState::NoLink);
    // The edge is spent: the switch held on at recovery does not arm, it needs a cycle.
    const ArmingOutput<float> held = r.live(kT0 + kRecovery);
    EXPECT_FALSE(held.armed);
    EXPECT_EQ(held.state, ArmState::SwitchNotCycled);
    r.in_.arm = kOff;
    EXPECT_EQ(r.live(kT0 + kRecovery + 1).state, ArmState::SwitchOff);
    r.in_.arm = kOn;
    EXPECT_TRUE(r.live(kT0 + kRecovery + 2).armed);
  }
  {
    Rig r;  // an edge at exactly recovery arms
    (void)r.run(kT0, kT0 + kRecovery - Rig::kStepUs);
    r.in_.arm = kOn;
    EXPECT_TRUE(r.live(kT0 + kRecovery).armed);
  }
}

TEST(Arming, LinkDropoutRestartsTheRecoveryRun) {
  Rig r;
  (void)r.run(kT0, kT0 + 3 * Rig::kStepUs);  // valid, last frame at kT0 + 30 ms
  // No frame for kLoss: the link is lost at kT0 + 30 ms + kLoss, the run is over.
  const std::uint64_t lost = kT0 + 3 * Rig::kStepUs + kLoss;
  EXPECT_EQ(r.at(lost).state, ArmState::NoLink);
  const std::uint64_t back = lost + Rig::kStepUs;
  (void)r.run(back, back + kRecovery - Rig::kStepUs);
  r.in_.arm = kOn;
  EXPECT_EQ(r.live(back + kRecovery - 1).state, ArmState::NoLink);  // recovery counted from the return
  r.in_.arm = kOff;
  (void)r.live(back + kRecovery);
  r.in_.arm = kOn;
  EXPECT_TRUE(r.live(back + kRecovery + 1).armed);
}

TEST(Arming, SwitchHighAtBootNeverArms) {
  Rig r;
  r.in_.arm = kOn;  // on from the first tick
  for (std::uint64_t t = kT0; t <= kT0 + 3 * kRecovery; t += Rig::kStepUs) {
    const ArmingOutput<float> o = r.live(t);
    EXPECT_FALSE(o.armed) << t;
    EXPECT_EQ(o.state, t - kT0 < kRecovery ? ArmState::NoLink : ArmState::SwitchNotCycled) << t;
  }
  // Cycled after recovery: arms on the rising edge.
  const std::uint64_t t = kT0 + 3 * kRecovery + Rig::kStepUs;
  r.in_.arm = kOff;
  EXPECT_EQ(r.live(t).state, ArmState::SwitchOff);
  r.in_.arm = kOn;
  EXPECT_TRUE(r.live(t + Rig::kStepUs).armed);
}

TEST(Arming, ArmThresholdBoundary) {
  Rig below;
  (void)below.run(kT0, kT0 + kRecovery - Rig::kStepUs);
  below.in_.arm = std::nextafter(kArmThreshold, 0.0F);
  const ArmingOutput<float> o = below.live(kT0 + kRecovery);
  EXPECT_FALSE(o.armed);
  EXPECT_EQ(o.state, ArmState::SwitchOff);
  Rig at;
  (void)at.run(kT0, kT0 + kRecovery - Rig::kStepUs);
  at.in_.arm = kArmThreshold;
  EXPECT_TRUE(at.live(kT0 + kRecovery).armed);
}

TEST(Arming, ThrottleUpRefuses) {
  Rig r;
  (void)r.run(kT0, kT0 + kRecovery - Rig::kStepUs);
  r.in_.arm = kOn;
  r.in_.throttle = kThrottleMin;  // not below the minimum
  ArmingOutput<float> o = r.live(kT0 + kRecovery);
  EXPECT_FALSE(o.armed);
  EXPECT_EQ(o.state, ArmState::ThrottleUp);
  // Throttle brought down with the switch still on: no new edge, no arming.
  r.in_.throttle = 0.0F;
  o = r.live(kT0 + kRecovery + Rig::kStepUs);
  EXPECT_FALSE(o.armed);
  EXPECT_EQ(o.state, ArmState::SwitchNotCycled);
  // Cycled with the throttle just below the minimum: arms.
  r.in_.arm = kOff;
  (void)r.live(kT0 + kRecovery + 2 * Rig::kStepUs);
  r.in_.arm = kOn;
  r.in_.throttle = std::nextafter(kThrottleMin, 0.0F);
  EXPECT_TRUE(r.live(kT0 + kRecovery + 3 * Rig::kStepUs).armed);
}

TEST(Arming, NaNSwitchAndThrottleAreSafe) {
  const float nan = std::numeric_limits<float>::quiet_NaN();
  Rig r;
  (void)r.run(kT0, kT0 + kRecovery - Rig::kStepUs);
  r.in_.arm = kOn;
  r.in_.throttle = nan;
  EXPECT_EQ(r.live(kT0 + kRecovery).state, ArmState::ThrottleUp);

  Rig a;
  (void)a.arm();
  a.in_.arm = nan;
  const ArmingOutput<float> o = a.live(Rig::kArmedAt + Rig::kStepUs);
  EXPECT_FALSE(o.armed);
  EXPECT_EQ(o.state, ArmState::SwitchOff);
}

TEST(Arming, SwitchOffDisarmsOnTheSameTick) {
  Rig r;
  (void)r.arm();
  r.in_.arm = kOff;
  const ArmingOutput<float> o = r.live(Rig::kArmedAt + 1);
  EXPECT_FALSE(o.armed);
  EXPECT_EQ(o.state, ArmState::SwitchOff);
  expect_zero_sticks(o);
  EXPECT_FALSE(r.machine_.armed());
  // And re-arms on the next edge, the link being valid throughout.
  r.in_.arm = kOn;
  EXPECT_TRUE(r.live(Rig::kArmedAt + 2).armed);
}

TEST(Arming, ImuDeadlineMissDisarmsAndBlocksUntilInit) {
  Rig r;
  (void)r.arm();
  r.in_.imu_deadline_missed = true;
  ArmingOutput<float> o = r.live(Rig::kArmedAt + 1);
  EXPECT_FALSE(o.armed);
  EXPECT_EQ(o.state, ArmState::ImuStale);
  expect_zero_sticks(o);
  // The input going back to false changes nothing: the miss is latched. A cycled switch with a perfect link is refused.
  r.in_.imu_deadline_missed = false;
  std::uint64_t t = Rig::kArmedAt + 2;
  for (int i = 0; i < 4; ++i) {
    r.in_.arm = kOff;
    (void)r.run(t, t + kRecovery);
    t += kRecovery + Rig::kStepUs;
    r.in_.arm = kOn;
    o = r.live(t);
    EXPECT_FALSE(o.armed);
    EXPECT_EQ(o.state, ArmState::ImuStale);
    t += Rig::kStepUs;
  }
  // Only a re-initialisation clears it.
  r.machine_.init(test_config());
  r.in_.arm = kOff;
  (void)r.run(t, t + kRecovery);
  r.in_.arm = kOn;
  EXPECT_TRUE(r.live(t + kRecovery + Rig::kStepUs).armed);
}

TEST(Arming, ImuDeadlineMissBeforeArmingRefuses) {
  Rig r;
  r.in_.imu_deadline_missed = true;
  (void)r.run(kT0, kT0 + kRecovery - Rig::kStepUs);
  r.in_.arm = kOn;
  const ArmingOutput<float> o = r.live(kT0 + kRecovery);
  EXPECT_FALSE(o.armed);
  EXPECT_EQ(o.state, ArmState::ImuStale);
}

TEST(Arming, InvalidImuSamplesRefuseButDoNotDisarm) {
  Rig r;
  (void)r.run(kT0, kT0 + kRecovery - Rig::kStepUs);
  r.in_.imu_samples_valid = false;
  r.in_.arm = kOn;
  EXPECT_EQ(r.live(kT0 + kRecovery).state, ArmState::ImuInvalid);
  Rig a;
  (void)a.arm();
  a.in_.imu_samples_valid = false;
  const ArmingOutput<float> o = a.live(Rig::kArmedAt + 1);
  EXPECT_TRUE(o.armed);
  EXPECT_EQ(o.state, ArmState::Armed);
}

TEST(Arming, InvalidEstimatorRefusesInItsOrderButNeverDisarms) {
  Rig r;
  (void)r.run(kT0, kT0 + kRecovery - Rig::kStepUs);
  r.in_.estimator_valid = false;
  r.in_.arm = kOn;
  EXPECT_EQ(r.live(kT0 + kRecovery).state, ArmState::EstimatorInvalid);
  // After ImuInvalid and Saturated, before ThrottleUp (a high throttle with an invalid estimate reads EstimatorInvalid).
  r.in_.throttle = 1.0F;
  EXPECT_EQ(r.live(kT0 + kRecovery + 1).state, ArmState::EstimatorInvalid);
  r.in_.imu_saturated = true;
  EXPECT_EQ(r.live(kT0 + kRecovery + 2).state, ArmState::Saturated);
  r.in_.imu_saturated = false;
  r.in_.imu_samples_valid = false;
  EXPECT_EQ(r.live(kT0 + kRecovery + 3).state, ArmState::ImuInvalid);
  // Valid again, switch still on: the refused edge is spent.
  r.in_.imu_samples_valid = true;
  r.in_.estimator_valid = true;
  r.in_.throttle = 0.0F;
  EXPECT_EQ(r.live(kT0 + kRecovery + 4).state, ArmState::SwitchNotCycled);
  // Losing validity while armed does not disarm.
  Rig a;
  (void)a.arm();
  a.in_.estimator_valid = false;
  for (std::uint64_t t = Rig::kArmedAt + 1; t < Rig::kArmedAt + 5; ++t) {
    const ArmingOutput<float> o = a.live(t);
    EXPECT_TRUE(o.armed);
    EXPECT_EQ(o.state, ArmState::Armed);
  }
}

TEST(Arming, SaturationRefusesButNeverDisarms) {
  Rig r;
  (void)r.run(kT0, kT0 + kRecovery - Rig::kStepUs);
  r.in_.imu_saturated = true;
  r.in_.arm = kOn;
  EXPECT_EQ(r.live(kT0 + kRecovery).state, ArmState::Saturated);
  // Saturation gone, switch still on: the refused edge is spent.
  r.in_.imu_saturated = false;
  EXPECT_EQ(r.live(kT0 + kRecovery + 1).state, ArmState::SwitchNotCycled);
  r.in_.arm = kOff;
  (void)r.live(kT0 + kRecovery + 2);
  r.in_.arm = kOn;
  EXPECT_TRUE(r.live(kT0 + kRecovery + 3).armed);
  // Saturation while armed: still armed, every tick.
  r.in_.imu_saturated = true;
  for (std::uint64_t t = kT0 + kRecovery + 4; t < kT0 + kRecovery + 8; ++t) {
    const ArmingOutput<float> o = r.live(t);
    EXPECT_TRUE(o.armed);
    EXPECT_EQ(o.state, ArmState::Armed);
  }
}

TEST(Arming, MotorTestBlocksArmingAndDisarmsIfItCoexists) {
  Rig r;
  (void)r.run(kT0, kT0 + kRecovery - Rig::kStepUs);
  r.in_.motor_test_active = true;
  r.in_.arm = kOn;
  ArmingOutput<float> o = r.live(kT0 + kRecovery);
  EXPECT_FALSE(o.armed);
  EXPECT_EQ(o.state, ArmState::MotorTest);
  r.in_.motor_test_active = false;
  EXPECT_EQ(r.live(kT0 + kRecovery + 1).state, ArmState::SwitchNotCycled);

  Rig a;
  (void)a.arm();
  a.in_.motor_test_active = true;
  o = a.live(Rig::kArmedAt + 1);
  EXPECT_FALSE(o.armed);
  EXPECT_EQ(o.state, ArmState::MotorTest);
  expect_zero_sticks(o);
}

TEST(Arming, FutureStampedFrameIsNoFrame) {
  Rig r;
  (void)r.run(kT0, kT0 + kRecovery - Rig::kStepUs);
  r.in_.arm = kOn;
  r.frame(kT0 + kRecovery + 1);  // one us later than the tick
  ArmingOutput<float> o = r.at(kT0 + kRecovery);
  EXPECT_FALSE(o.armed);
  EXPECT_EQ(o.state, ArmState::NoLink);
  // Stamp equal to the tick is valid (age 0), a fresh run starting there though: still short of the recovery.
  r.frame(kT0 + kRecovery + 1);
  EXPECT_EQ(r.at(kT0 + kRecovery + 1).state, ArmState::NoLink);

  Rig a;
  (void)a.arm();
  a.frame(Rig::kArmedAt + 2 * Rig::kStepUs);
  o = a.at(Rig::kArmedAt + Rig::kStepUs);
  EXPECT_FALSE(o.armed);
  EXPECT_EQ(o.state, ArmState::Failsafe);
}

TEST(Arming, FailsafeBoundariesWhileArmed) {
  Rig r;
  (void)r.arm();
  const std::uint64_t last = Rig::kArmedAt;  // the last frame; its sticks are 0.25, -0.5, 0.125, 0
  r.sticks(0.25F, -0.5F, 0.125F, 0.5F);
  r.frame(last);
  r.in_.arm = kOn;

  ArmingOutput<float> o = r.at(last + kLoss - 1);  // link still valid
  EXPECT_TRUE(o.armed);
  EXPECT_EQ(o.state, ArmState::Armed);
  EXPECT_EQ(o.roll, 0.25F);

  o = r.at(last + kLoss);  // lost: the last sticks, held
  EXPECT_TRUE(o.armed);
  EXPECT_EQ(o.state, ArmState::ArmedHold);
  EXPECT_EQ(o.roll, 0.25F);
  EXPECT_EQ(o.pitch, -0.5F);
  EXPECT_EQ(o.yaw, 0.125F);
  EXPECT_EQ(o.throttle, 0.5F);

  o = r.at(last + kHold - 1);
  EXPECT_EQ(o.state, ArmState::ArmedHold);
  EXPECT_EQ(o.throttle, 0.5F);

  o = r.at(last + kHold);  // centred, throttle 0, still armed
  EXPECT_TRUE(o.armed);
  EXPECT_EQ(o.state, ArmState::ArmedFailsafeCentred);
  expect_zero_sticks(o);

  o = r.at(last + kDisarm - 1);
  EXPECT_TRUE(o.armed);
  EXPECT_EQ(o.state, ArmState::ArmedFailsafeCentred);
  expect_zero_sticks(o);

  o = r.at(last + kDisarm);  // disarmed
  EXPECT_FALSE(o.armed);
  EXPECT_EQ(o.state, ArmState::Failsafe);
  expect_zero_sticks(o);
  EXPECT_FALSE(r.machine_.armed());
}

TEST(Arming, NoFrameEverWhileArmedIsFailsafe) {
  // Not reachable through arm() (arming needs a frame); the guard is the machine's own: zero frames is age unbounded.
  Rig r;
  (void)r.arm();
  r.in_.frames = 0;
  const ArmingOutput<float> o = r.at(Rig::kArmedAt + 1);
  EXPECT_FALSE(o.armed);
  EXPECT_EQ(o.state, ArmState::Failsafe);
}

TEST(Arming, LinkReturnBeforeDisarmGivesLiveSticksAtOnce) {
  for (const std::uint64_t gap : {kLoss, kHold - 1, kHold, kDisarm - 1}) {
    Rig r;
    (void)r.arm();
    const std::uint64_t last = Rig::kArmedAt;
    r.in_.arm = kOn;
    EXPECT_NE(r.at(last + gap).state, ArmState::Armed);  // lost, held or centred
    r.sticks(-0.75F, 0.5F, -0.25F, 0.375F);
    const ArmingOutput<float> o = r.live(last + gap + 1);  // the very next tick, no recovery wait
    EXPECT_TRUE(o.armed) << gap;
    EXPECT_EQ(o.state, ArmState::Armed) << gap;
    EXPECT_EQ(o.roll, -0.75F);
    EXPECT_EQ(o.pitch, 0.5F);
    EXPECT_EQ(o.yaw, -0.25F);
    EXPECT_EQ(o.throttle, 0.375F);
  }
}

TEST(Arming, SwitchLeftOnThroughAFailsafeNeverRearms) {
  Rig r;
  (void)r.arm();
  const std::uint64_t last = Rig::kArmedAt;
  r.in_.arm = kOn;
  EXPECT_EQ(r.at(last + kDisarm).state, ArmState::Failsafe);
  EXPECT_EQ(r.at(last + kDisarm + 1).state, ArmState::Failsafe);  // no link yet
  // The link returns and stays valid well past the recovery, the switch still on.
  const std::uint64_t back = last + kDisarm + Rig::kStepUs;
  r.sticks(0.0F, 0.0F, 0.0F, 0.0F);
  EXPECT_EQ(r.live(back).state, ArmState::NoLink);
  for (std::uint64_t t = back + Rig::kStepUs; t <= back + 4 * kRecovery; t += Rig::kStepUs) {
    const ArmingOutput<float> o = r.live(t);
    EXPECT_FALSE(o.armed) << t;
    EXPECT_EQ(o.state, t - back < kRecovery ? ArmState::NoLink : ArmState::SwitchNotCycled) << t;
  }
  // Cycled after the recovery: arms.
  const std::uint64_t t = back + 4 * kRecovery + Rig::kStepUs;
  r.in_.arm = kOff;
  (void)r.live(t);
  r.in_.arm = kOn;
  EXPECT_TRUE(r.live(t + Rig::kStepUs).armed);
}

TEST(Arming, CycleBeforeRecoveryAfterFailsafeIsRefused) {
  Rig r;
  (void)r.arm();
  const std::uint64_t last = Rig::kArmedAt;
  r.in_.arm = kOn;
  EXPECT_EQ(r.at(last + kDisarm).state, ArmState::Failsafe);
  const std::uint64_t back = last + kDisarm + Rig::kStepUs;
  r.in_.arm = kOff;
  (void)r.live(back);
  r.in_.arm = kOn;  // edge 10 ms into the new run
  EXPECT_EQ(r.live(back + Rig::kStepUs).state, ArmState::NoLink);
  // The edge is spent; held on through the recovery, no arming.
  EXPECT_EQ(r.live(back + kRecovery).state, ArmState::SwitchNotCycled);
  // Recovery at exactly kRecovery from the return (back): the edge there arms.
  r.in_.arm = kOff;
  (void)r.live(back + kRecovery + 1);
  r.in_.arm = kOn;
  EXPECT_TRUE(r.live(back + kRecovery + 2).armed);
}

TEST(Arming, FailsafeReasonClearsWhenTheLinkIsValid) {
  Rig r;
  (void)r.arm();
  r.in_.arm = kOff;
  EXPECT_EQ(r.at(Rig::kArmedAt + kDisarm).state, ArmState::Failsafe);
  EXPECT_EQ(r.at(Rig::kArmedAt + kDisarm + 1).state, ArmState::Failsafe);
  EXPECT_EQ(r.live(Rig::kArmedAt + kDisarm + 2).state, ArmState::NoLink);
}

}  // namespace
}  // namespace marv::arming
