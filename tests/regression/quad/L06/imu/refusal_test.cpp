// L6 stage (a), IMU refusals: every refused call returns the right status, changes no state, draws nothing and leaves
// the output as it was. "Changes no state" is checked on a twin: two plants of one seed and config, one of which takes
// the refused calls; their later outputs (with noise on, so a lost or extra draw or a half-applied attach shows) must
// be bit-identical.
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "support.hpp"

namespace {

using namespace marv::plant::imu_test;

constexpr std::uint64_t kSeed = 7;  // labelled: any fixed seed

// Noise on in every axis so a stream that moved is visible. N, B are labelled test values (not sensor figures).
marv_plant_imu_config noisy_config(std::uint32_t latency = 2) {
  marv_plant_imu_config c = zero_noise_config(latency);
  c.gyro.noise_density = 1.0e-3;
  c.gyro.bias_instability = 2.0e-3;
  c.accel.noise_density = 1.0e-2;
  c.accel.bias_instability = 2.0e-2;
  c.gyro.lsb = 1.0e-3;    // fine enough that the noise is visible at every sample
  c.accel.lsb = 1.0e-3;
  return c;
}

marv_plant_body some_body() { return body_with_omega(0.5, -0.25, 0.125); }

// Takes `n` samples of the plant, returns the outputs.
std::vector<marv_plant_imu_out> take(marv_plant* p, int n) {
  std::vector<marv_plant_imu_out> v;
  for (int k = 0; k < n; ++k) {
    v.push_back(sample_ok(p, some_body()));
  }
  return v;
}

bool same_run(const std::vector<marv_plant_imu_out>& a, const std::vector<marv_plant_imu_out>& b) {
  if (a.size() != b.size()) {
    return false;
  }
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (!same_bits(a[i], b[i])) {
      return false;
    }
  }
  return true;
}

constexpr marv_plant_imu_out kSentinel = {{1.0F, 2.0F, 3.0F}, {4.0F, 5.0F, 6.0F}, 7.0F, 0xFFFFU};

TEST(ImuRefusal, NewStatusAppendedAfterAllocWithAMessage) {
  EXPECT_EQ(MARV_PLANT_E_STATE, MARV_PLANT_E_ALLOC + 1);
  EXPECT_STRNE(marv_plant_status_str(MARV_PLANT_E_STATE), "unknown status");
  EXPECT_EQ(MARV_PLANT_OK, 0);
  EXPECT_EQ(MARV_PLANT_E_ALLOC, 7);  // the existing codes keep their numbers
}

TEST(ImuRefusal, AttachRefusalsLeaveATwinIdentical) {
  PlantHandle a(kSeed);
  PlantHandle b(kSeed);
  const marv_plant_imu_config good = noisy_config();

  EXPECT_EQ(marv_plant_imu_attach(nullptr, &good), MARV_PLANT_E_NULL);
  EXPECT_EQ(marv_plant_imu_attach(a.get(), nullptr), MARV_PLANT_E_NULL);
  marv_plant_imu_config bad = good;
  bad.struct_size = sizeof(bad) - 1;
  EXPECT_EQ(marv_plant_imu_attach(a.get(), &bad), MARV_PLANT_E_ABI);
  bad.struct_size = sizeof(bad) + 1;
  EXPECT_EQ(marv_plant_imu_attach(a.get(), &bad), MARV_PLANT_E_ABI);

  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  // Each mutation breaks one field; each must be E_CONFIG.
  std::vector<marv_plant_imu_config> bads;
  for (marv_plant_imu_axis_config marv_plant_imu_config::*axis : {&marv_plant_imu_config::gyro, &marv_plant_imu_config::accel}) {
    auto push = [&](auto mutate) {
      marv_plant_imu_config c = good;
      mutate(c.*axis);
      bads.push_back(c);
    };
    push([&](marv_plant_imu_axis_config& x) { x.noise_density = nan; });
    push([&](marv_plant_imu_axis_config& x) { x.noise_density = -1.0e-3; });
    push([&](marv_plant_imu_axis_config& x) { x.bias_instability = inf; });
    push([&](marv_plant_imu_axis_config& x) { x.bias_instability = -1.0e-3; });
    push([&](marv_plant_imu_axis_config& x) { x.lsb = 0.0; });
    push([&](marv_plant_imu_axis_config& x) { x.lsb = -1.0e-3; });
    push([&](marv_plant_imu_axis_config& x) { x.lsb = nan; });
    push([&](marv_plant_imu_axis_config& x) { x.full_scale = 0.0; });
    push([&](marv_plant_imu_axis_config& x) { x.full_scale = inf; });
    push([&](marv_plant_imu_axis_config& x) { x.full_scale = std::numeric_limits<double>::max(); });
    push([&](marv_plant_imu_axis_config& x) { x.turn_on_bias[1] = nan; });
    push([&](marv_plant_imu_axis_config& x) { x.turn_on_bias[2] = -inf; });
    // B^2 / N overflows (N tiny, B large): a non-finite random-walk gain is refused.
    push([&](marv_plant_imu_axis_config& x) { x.noise_density = std::numeric_limits<double>::denorm_min(); x.bias_instability = 1.0e300; });
  }
  marv_plant_imu_config over = good;
  over.latency_samples = kMaxLatency + 1;
  bads.push_back(over);
  over.latency_samples = std::numeric_limits<std::uint32_t>::max();
  bads.push_back(over);
  for (std::size_t i = 0; i < bads.size(); ++i) {
    EXPECT_EQ(marv_plant_imu_attach(a.get(), &bads[i]), MARV_PLANT_E_CONFIG) << "case " << i;
  }
  // A refused attach leaves the plant un-attached: a sample still reports E_STATE.
  marv_plant_imu_out o = kSentinel;
  const marv_plant_body body = some_body();
  EXPECT_EQ(marv_plant_imu_sample(a.get(), &body, kDt, &o), MARV_PLANT_E_STATE);
  EXPECT_TRUE(same_bits(o, kSentinel));

  // The maximum latency is accepted (the boundary of the refusal above).
  PlantHandle edge(kSeed);
  marv_plant_imu_config max_cfg = good;
  max_cfg.latency_samples = kMaxLatency;
  EXPECT_EQ(marv_plant_imu_attach(edge.get(), &max_cfg), MARV_PLANT_OK);

  ASSERT_EQ(marv_plant_imu_attach(a.get(), &good), MARV_PLANT_OK);
  ASSERT_EQ(marv_plant_imu_attach(b.get(), &good), MARV_PLANT_OK);
  EXPECT_TRUE(same_run(take(a.get(), 8), take(b.get(), 8)));  // labelled: 8 samples
}

TEST(ImuRefusal, AttachTwiceIsRefusedAndChangesNothing) {
  PlantHandle a(kSeed);
  PlantHandle b(kSeed);
  const marv_plant_imu_config good = noisy_config();
  ASSERT_EQ(marv_plant_imu_attach(a.get(), &good), MARV_PLANT_OK);
  ASSERT_EQ(marv_plant_imu_attach(b.get(), &good), MARV_PLANT_OK);
  std::vector<marv_plant_imu_out> ra = take(a.get(), 3);
  const std::vector<marv_plant_imu_out> rb3 = take(b.get(), 3);
  EXPECT_TRUE(same_run(ra, rb3));
  marv_plant_imu_config other = noisy_config(0);
  other.gyro.turn_on_bias[0] = 5.0;  // a different config, which would be visible if it were applied
  EXPECT_EQ(marv_plant_imu_attach(a.get(), &other), MARV_PLANT_E_STATE);
  EXPECT_EQ(marv_plant_imu_attach(a.get(), &good), MARV_PLANT_E_STATE);  // the same config again, too
  EXPECT_TRUE(same_run(take(a.get(), 6), take(b.get(), 6)));  // the stream continues; no reset, no new config
}

TEST(ImuRefusal, SampleRefusalsLeaveATwinIdentical) {
  PlantHandle a(kSeed);
  PlantHandle b(kSeed);
  const marv_plant_imu_config good = noisy_config();
  const marv_plant_body body = some_body();
  marv_plant_imu_out o = kSentinel;

  // Before the attach.
  EXPECT_EQ(marv_plant_imu_sample(a.get(), &body, kDt, &o), MARV_PLANT_E_STATE);
  EXPECT_TRUE(same_bits(o, kSentinel));

  ASSERT_EQ(marv_plant_imu_attach(a.get(), &good), MARV_PLANT_OK);
  ASSERT_EQ(marv_plant_imu_attach(b.get(), &good), MARV_PLANT_OK);
  std::vector<marv_plant_imu_out> ra;
  std::vector<marv_plant_imu_out> rb;
  auto refuse_all = [&] {
    o = kSentinel;
    marv_plant_body bad = body;
    EXPECT_EQ(marv_plant_imu_sample(nullptr, &body, kDt, &o), MARV_PLANT_E_NULL);
    EXPECT_EQ(marv_plant_imu_sample(a.get(), nullptr, kDt, &o), MARV_PLANT_E_NULL);
    EXPECT_EQ(marv_plant_imu_sample(a.get(), &body, kDt, nullptr), MARV_PLANT_E_NULL);
    bad.struct_size = sizeof(bad) - 1;
    EXPECT_EQ(marv_plant_imu_sample(a.get(), &bad, kDt, &o), MARV_PLANT_E_ABI);
    bad = body;
    bad.q_wxyz[0] = 2.0;  // |q|^2 = 4: not normalised
    EXPECT_EQ(marv_plant_imu_sample(a.get(), &bad, kDt, &o), MARV_PLANT_E_BODY);
    bad = body;
    bad.omega_frd_rad_s[2] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(marv_plant_imu_sample(a.get(), &bad, kDt, &o), MARV_PLANT_E_BODY);
    for (double dt : {0.0, -kDt, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
      EXPECT_EQ(marv_plant_imu_sample(a.get(), &body, dt, &o), MARV_PLANT_E_DT);
    }
    EXPECT_TRUE(same_bits(o, kSentinel));
  };
  for (int k = 0; k < 5; ++k) {  // labelled: refusals before, between and after samples
    refuse_all();
    ra.push_back(sample_ok(a.get(), body));
    rb.push_back(sample_ok(b.get(), body));
  }
  refuse_all();
  EXPECT_TRUE(same_run(ra, rb));
  EXPECT_TRUE(same_run(take(a.get(), 4), take(b.get(), 4)));
}

TEST(ImuRefusal, StepRefusalsDoNotTouchTheImu) {
  // A refused marv_plant_step changes nothing, including the thrust the IMU reads: samples after it match the twin's.
  PlantHandle a(kSeed, 1500.0);
  PlantHandle b(kSeed, 1500.0);
  const marv_plant_imu_config good = noisy_config();
  ASSERT_EQ(marv_plant_imu_attach(a.get(), &good), MARV_PLANT_OK);
  ASSERT_EQ(marv_plant_imu_attach(b.get(), &good), MARV_PLANT_OK);
  const marv_plant_body body = some_body();
  const marv_plant_cmd bad_cmd = make_cmd(10, 0, 0, 0);  // neither 0 nor 48..2047
  marv_plant_out po = make_out();
  EXPECT_EQ(marv_plant_step(a.get(), &body, &bad_cmd, kDt, &po), MARV_PLANT_E_CMD);
  EXPECT_TRUE(same_run(take(a.get(), 3), take(b.get(), 3)));
}

}  // namespace
