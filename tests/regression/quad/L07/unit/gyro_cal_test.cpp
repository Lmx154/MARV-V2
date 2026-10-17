// T1 for the boot gyro calibration (fw/estimator gyro_cal.hpp, M3 P6, decision 0042): the bias is the per-axis mean of a still
// window, a window in which the vehicle moved is discarded and started again, and a sample whose GyroValid flag is clear or
// whose axis is not finite is not accumulated.
//
// Numbers here are labelled scenario values of this test, not flight numbers:
//   - the tick, 130 us, is the whole-microsecond value of the board's 7680 Hz tick (fc3v2_scenario_values.yaml);
//   - the window, 1.25 s, is Betaflight 4.5.1's gyro_calibration_duration default (gyro.c:107), the value of the quad_fc
//     register's gyro_cal_duration_us (decision 0042);
//   - the limit, 0.05 rad/s, is a round value of the order of the register's gyro_cal_movement_max (0.0586 rad/s); the tests
//     take the margins from it (0.9 and 1.1 times), not from the register;
//   - the bias, the noise amplitude and the motion are chosen to exercise every axis.
// The noise is a deterministic zero-mean pattern of amplitude a: (k mod 11 - 5) / 5 times a, with a phase per axis (a sample
// standard deviation near 0.47 a). The expected means are computed in double from the samples the test feeds, so no closed
// form is assumed. Tolerances are stated where used; the mean's float rounding bound is n x epsilon x max|x| (Welford's
// recurrence, one rounding per update).
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "marv/estimator/gyro_cal.hpp"
#include "marv/prim/vec.hpp"

namespace {

using marv::estimator::GyroCal;
using marv::estimator::GyroCalConfig;
using marv::prim::Vec3;

constexpr std::uint64_t kTickUs = 130;
constexpr std::uint64_t kDurationUs = 1250000;
constexpr float kLimit = 0.05F;  // rad/s
constexpr double kEps = std::numeric_limits<float>::epsilon();
constexpr double kBias[3] = {0.02, -0.03, 0.011};  // rad/s

GyroCal<float> make() {
  GyroCal<float> c;
  c.init(GyroCalConfig<float>{kDurationUs, kLimit});
  return c;
}

// The noise of sample k on axis i, amplitude a: a deterministic pattern with a different phase per axis.
double noise(int k, int i, double a) { return a * (static_cast<double>((k + 3 * i) % 11) - 5.0) / 5.0; }

struct Stream {
  std::vector<Vec3<float>> gyro;
  std::vector<std::uint64_t> t_us;
};

// n samples from stamp t0 on: the bias plus noise(a) on every axis.
Stream still(std::uint64_t t0, int n, double a) {
  Stream s;
  for (int k = 0; k < n; ++k) {
    s.gyro.emplace_back(static_cast<float>(kBias[0] + noise(k, 0, a)), static_cast<float>(kBias[1] + noise(k, 1, a)),
                        static_cast<float>(kBias[2] + noise(k, 2, a)));
    s.t_us.push_back(t0 + static_cast<std::uint64_t>(k) * kTickUs);
  }
  return s;
}

int ticks_for(std::uint64_t us) { return static_cast<int>(us / kTickUs) + 1; }

// The index of the first sample of a stream at the tick whose stamp is at least the duration after the first: ceil(duration / tick).
constexpr int kDoneIndex = static_cast<int>((kDurationUs + kTickUs - 1) / kTickUs);

// Feeds the stream and returns the index of the sample on which complete() first became true, or -1.
int feed(GyroCal<float>& c, const Stream& s) {
  for (std::size_t k = 0; k < s.gyro.size(); ++k) {
    c.update(s.gyro[k], true, s.t_us[k]);
    if (c.complete()) {
      return static_cast<int>(k);
    }
  }
  return -1;
}

double mean_of(const Stream& s, std::size_t first, std::size_t last, int axis) {
  double sum = 0.0;
  for (std::size_t k = first; k <= last; ++k) {
    sum += static_cast<double>(s.gyro[k][static_cast<std::size_t>(axis)]);
  }
  return sum / static_cast<double>(last - first + 1);
}

// a. A constant bias with noise inside the limit: not complete before the window has elapsed, complete on the first sample
// whose stamp is a duration after the first, and the bias is the mean of the samples fed. The noise amplitude a = 0.02 rad/s has
// a sample standard deviation near 0.47 a, 0.0093 rad/s, 0.19 of the limit. The mean's bound is the
// float rounding of the recurrence, n x epsilon x max|x| over the window (n 9616, max|x| 0.05 rad/s: 5.7e-5 rad/s) added to
// nothing else: the expected value is the double mean of the same float samples.
TEST(GyroCal, ConstantBiasWithNoiseWithinTheLimitCompletesWithTheMean) {
  GyroCal<float> c = make();
  const Stream s = still(1000, 2 * ticks_for(kDurationUs), 0.02);
  const int done = feed(c, s);
  ASSERT_GE(done, 0);
  EXPECT_EQ(done, kDoneIndex) << "completes on the first sample a duration after the first";
  EXPECT_GE(s.t_us[static_cast<std::size_t>(done)] - s.t_us[0], kDurationUs);
  EXPECT_LT(s.t_us[static_cast<std::size_t>(done) - 1] - s.t_us[0], kDurationUs);
  const double tol = static_cast<double>(done + 1) * kEps * 0.05;
  for (int i = 0; i < 3; ++i) {
    const double mean = mean_of(s, 0, static_cast<std::size_t>(done), i);
    EXPECT_NEAR(static_cast<double>(c.bias()[static_cast<std::size_t>(i)]), mean, tol) << "axis " << i;
    EXPECT_NEAR(static_cast<double>(c.bias()[static_cast<std::size_t>(i)]), kBias[i], 0.02 / 10.0) << "axis " << i;
  }
}

// b. Until complete the bias is zero, and after it later samples change nothing.
TEST(GyroCal, BiasIsZeroUntilCompleteAndFixedAfter) {
  GyroCal<float> c = make();
  const Stream s = still(0, 2 * ticks_for(kDurationUs), 0.02);
  for (std::size_t k = 0; k + 1 < static_cast<std::size_t>(ticks_for(kDurationUs)); ++k) {
    c.update(s.gyro[k], true, s.t_us[k]);
    ASSERT_FALSE(c.complete());
    EXPECT_EQ(c.bias()[0], 0.0F);
    EXPECT_EQ(c.bias()[1], 0.0F);
    EXPECT_EQ(c.bias()[2], 0.0F);
  }
  ASSERT_GE(feed(c, still(s.t_us[static_cast<std::size_t>(ticks_for(kDurationUs)) - 1], 2, 0.02)), 0);
  const Vec3<float> done = c.bias();
  c.update(Vec3<float>(1.0F, 1.0F, 1.0F), true, 10 * kDurationUs);
  EXPECT_TRUE(c.complete());
  EXPECT_EQ(c.bias()[0], done[0]);
  EXPECT_EQ(c.bias()[1], done[1]);
  EXPECT_EQ(c.bias()[2], done[2]);
}

// c. Motion above the limit: a window in which every axis swings by +-0.2 rad/s (sample standard deviation near 0.2, four times
// the limit) is discarded at its close, the calibration is not complete, and it completes one still duration after the next
// accepted sample; the bias is the still level, with none of the motion in it (the mean of the motion window is not the bias).
TEST(GyroCal, MotionAboveTheLimitRestartsThenCompletesAfterAStillDuration) {
  GyroCal<float> c = make();
  const int n = ticks_for(kDurationUs);
  Stream moving;
  for (int k = 0; k <= kDoneIndex; ++k) {  // through the sample that closes the window
    const double swing = (k % 2 == 0 ? 0.2 : -0.2) + 0.1;  // mean +0.1 rad/s
    moving.gyro.emplace_back(static_cast<float>(swing), static_cast<float>(swing), static_cast<float>(swing));
    moving.t_us.push_back(static_cast<std::uint64_t>(k) * kTickUs);
  }
  EXPECT_EQ(feed(c, moving), -1);
  EXPECT_FALSE(c.complete());
  const std::uint64_t t0 = moving.t_us.back() + kTickUs;
  const Stream calm = still(t0, 2 * n, 0.02);
  const int done = feed(c, calm);
  ASSERT_GE(done, 0);
  EXPECT_EQ(done, kDoneIndex) << "a still duration after the first still sample";
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(static_cast<double>(c.bias()[static_cast<std::size_t>(i)]), mean_of(calm, 0, static_cast<std::size_t>(done), i),
                static_cast<double>(done + 1) * kEps * 0.05)
        << "axis " << i;
  }
}

// d. The limit decides, on one axis alone: the same stream with the standard deviation of the z axis at 0.9 of the limit
// completes, at 1.1 of the limit it does not, however many windows pass (two durations here). The alternating +-a stream has
// a sample standard deviation of a x sqrt(n / (n - 1)), n 9616: 1.00005 a, so the margins 0.9 and 1.1 are far outside it.
TEST(GyroCal, TheMovementLimitDecidesOnEveryAxis) {
  for (int axis = 0; axis < 3; ++axis) {
    for (const double factor : {0.9, 1.1}) {
      GyroCal<float> c = make();
      Stream s = still(0, 3 * ticks_for(kDurationUs), 0.0);
      for (std::size_t k = 0; k < s.gyro.size(); ++k) {
        s.gyro[k][static_cast<std::size_t>(axis)] += static_cast<float>((k % 2 == 0 ? 1.0 : -1.0) * factor * static_cast<double>(kLimit));
      }
      const int done = feed(c, s);
      if (factor < 1.0) {
        EXPECT_GE(done, 0) << "axis " << axis << " at " << factor << " of the limit";
      } else {
        EXPECT_EQ(done, -1) << "axis " << axis << " at " << factor << " of the limit";
      }
    }
  }
}

// e. Samples that are not accumulated: a GyroValid-clear sample (any value) and a valid-flag sample with a non-finite axis are
// dropped, and do not open the window, move its close or enter the mean. The stream below is the still stream of (a) with an
// invalid sample of a wild value after every valid one at the same stamp; the result is (a)'s. A non-finite sample first in
// the stream does not open the window either: its stamp is long before the first valid one's.
TEST(GyroCal, InvalidAndNonFiniteSamplesAreNotAccumulated) {
  const Stream s = still(1000, 2 * ticks_for(kDurationUs), 0.02);
  GyroCal<float> ref = make();
  const int done_ref = feed(ref, s);
  ASSERT_GE(done_ref, 0);

  GyroCal<float> c = make();
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  c.update(Vec3<float>(nan, 0.0F, 0.0F), true, 0);
  c.update(Vec3<float>(0.0F, inf, 0.0F), true, 1);
  int done = -1;
  for (std::size_t k = 0; k < s.gyro.size() && done < 0; ++k) {
    c.update(s.gyro[k], true, s.t_us[k]);
    c.update(Vec3<float>(5.0F, 5.0F, 5.0F), false, s.t_us[k]);
    c.update(Vec3<float>(5.0F, nan, 5.0F), true, s.t_us[k]);
    c.update(Vec3<float>(-inf, 5.0F, 5.0F), true, s.t_us[k]);
    if (c.complete()) {
      done = static_cast<int>(k);
    }
  }
  EXPECT_EQ(done, done_ref);
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_EQ(c.bias()[i], ref.bias()[i]) << "axis " << i;
  }
}

// f. A stamp before the window's first restarts the window on that sample, and reset() returns to the start: not complete, zero
// bias, and the next stream completes as the first did.
TEST(GyroCal, ABackwardsStampRestartsAndResetReturnsToTheStart) {
  GyroCal<float> c = make();
  const int n = ticks_for(kDurationUs);
  const Stream a = still(10 * kDurationUs, n - 1, 0.02);  // one sample short of closing
  EXPECT_EQ(feed(c, a), -1);
  const Stream b = still(0, 2 * n, 0.02);  // starts before the window did: restarts on it
  const int done = feed(c, b);
  EXPECT_EQ(done, kDoneIndex);
  c.reset();
  EXPECT_FALSE(c.complete());
  EXPECT_EQ(c.bias()[0], 0.0F);
  EXPECT_EQ(feed(c, still(0, 2 * n, 0.02)), kDoneIndex);
}

}  // namespace
