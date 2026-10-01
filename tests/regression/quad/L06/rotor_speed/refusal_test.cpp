// L6 stage (b), rotor-speed refusals: every refused call returns the right status, changes no state and leaves the output
// as it was. "Changes no state" is checked on a twin: two plants of one config, one of which takes the refused calls;
// their later samples (the rotors moving every tick, a delay line of 2) must be bit-identical.
#include <cmath>
#include <limits>

#include "support.hpp"

namespace {

using namespace marv::plant::rotor_test;

constexpr std::uint32_t kLatency = 2;  // labelled: a nonzero delay, so a half-applied attach shows in the delay line
constexpr std::size_t kTicks = 8;      // labelled: more than the delay

marv_plant_cmd ramp_cmd(std::size_t k) {
  // Legal DShot values (48..2047) that differ per motor and per tick: 48 + (97 k + 211 i) mod 1900, the primes coprime to
  // 1900 = 2^2 5^2 19, so every motor moves every tick.
  marv_plant_cmd c{};
  c.struct_size = sizeof(c);
  for (std::size_t i = 0; i < MARV_PLANT_N_MOTORS; ++i) {
    c.dshot[i] = static_cast<std::uint16_t>(kDshotMin + ((97 * k + 211 * i) % 1900));
  }
  return c;
}

// Steps and samples kTicks times (sample, then step, as the adapter does).
std::vector<marv_plant_rotor_speed_out> run(marv_plant* p) {
  std::vector<marv_plant_rotor_speed_out> v;
  const marv_plant_body body = identity_body();
  for (std::size_t k = 0; k < kTicks; ++k) {
    v.push_back(sample_ok(p));
    const marv_plant_cmd cmd = ramp_cmd(k);
    marv_plant_out out{};
    out.struct_size = sizeof(out);
    EXPECT_EQ(marv_plant_step(p, &body, &cmd, kSubstep, &out), MARV_PLANT_OK);
  }
  return v;
}

bool same_run(const std::vector<marv_plant_rotor_speed_out>& a, const std::vector<marv_plant_rotor_speed_out>& b) {
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

constexpr marv_plant_rotor_speed_out kSentinel = {{1.0F, 2.0F, 3.0F, 4.0F}, 0xFFFFU};

bool is_sentinel(const marv_plant_rotor_speed_out& o) { return same_bits(o, kSentinel); }

TEST(RotorSpeedRefusal, EveryBadConfigIsRefusedWithTheRightStatus) {
  PlantHandle plant;
  marv_plant* p = plant.get();
  const marv_plant_rotor_speed_config good = grid_config(kLatency);
  EXPECT_EQ(marv_plant_rotor_speed_attach(nullptr, &good), MARV_PLANT_E_NULL);
  EXPECT_EQ(marv_plant_rotor_speed_attach(p, nullptr), MARV_PLANT_E_NULL);

  marv_plant_rotor_speed_config c = good;
  c.struct_size = sizeof(c) - 1;
  EXPECT_EQ(marv_plant_rotor_speed_attach(p, &c), MARV_PLANT_E_ABI);
  c.struct_size = sizeof(c) + 1;
  EXPECT_EQ(marv_plant_rotor_speed_attach(p, &c), MARV_PLANT_E_ABI);

  const auto bad = [&](auto mutate) {
    marv_plant_rotor_speed_config m = good;
    mutate(m);
    return marv_plant_rotor_speed_attach(p, &m);
  };
  EXPECT_EQ(bad([](auto& m) { m.pole_count = 0; }), MARV_PLANT_E_CONFIG);
  EXPECT_EQ(bad([](auto& m) { m.pole_count = 13; }), MARV_PLANT_E_CONFIG);
  EXPECT_EQ(bad([](auto& m) { m.latency_ticks = MARV_PLANT_ROTOR_SPEED_MAX_LATENCY_TICKS + 1; }), MARV_PLANT_E_CONFIG);
  EXPECT_EQ(bad([](auto& m) { m.exponent_bits = 0; }), MARV_PLANT_E_CONFIG);
  EXPECT_EQ(bad([](auto& m) { m.exponent_bits = MARV_PLANT_ROTOR_SPEED_MAX_EXPONENT_BITS + 1; }), MARV_PLANT_E_CONFIG);
  EXPECT_EQ(bad([](auto& m) { m.mantissa_bits = 0; }), MARV_PLANT_E_CONFIG);
  EXPECT_EQ(bad([](auto& m) { m.mantissa_bits = MARV_PLANT_ROTOR_SPEED_MAX_MANTISSA_BITS + 1; }), MARV_PLANT_E_CONFIG);
  EXPECT_EQ(bad([](auto& m) { m.period_unit_s = 0.0; }), MARV_PLANT_E_CONFIG);
  EXPECT_EQ(bad([](auto& m) { m.period_unit_s = -1.0e-6; }), MARV_PLANT_E_CONFIG);
  EXPECT_EQ(bad([](auto& m) { m.period_unit_s = std::numeric_limits<double>::quiet_NaN(); }), MARV_PLANT_E_CONFIG);
  EXPECT_EQ(bad([](auto& m) { m.period_unit_s = std::numeric_limits<double>::infinity(); }), MARV_PLANT_E_CONFIG);
  // The bounds themselves are accepted (on separate plants: one attach per plant).
  for (const auto edge : {0, 1}) {
    PlantHandle q;
    marv_plant_rotor_speed_config m = good;
    m.latency_ticks = edge != 0 ? MARV_PLANT_ROTOR_SPEED_MAX_LATENCY_TICKS : 0;
    m.exponent_bits = edge != 0 ? MARV_PLANT_ROTOR_SPEED_MAX_EXPONENT_BITS : 1;
    m.mantissa_bits = edge != 0 ? MARV_PLANT_ROTOR_SPEED_MAX_MANTISSA_BITS : 1;
    EXPECT_EQ(marv_plant_rotor_speed_attach(q.get(), &m), MARV_PLANT_OK) << "edge " << edge;
  }
}

TEST(RotorSpeedRefusal, SampleBeforeAttachAndWithNullArguments) {
  PlantHandle plant;
  marv_plant_rotor_speed_out o = kSentinel;
  EXPECT_EQ(marv_plant_rotor_speed_sample(plant.get(), &o), MARV_PLANT_E_STATE);
  EXPECT_TRUE(is_sentinel(o));
  const marv_plant_rotor_speed_config good = grid_config(kLatency);
  ASSERT_EQ(marv_plant_rotor_speed_attach(plant.get(), &good), MARV_PLANT_OK);
  EXPECT_EQ(marv_plant_rotor_speed_sample(nullptr, &o), MARV_PLANT_E_NULL);
  EXPECT_EQ(marv_plant_rotor_speed_sample(plant.get(), nullptr), MARV_PLANT_E_NULL);
  EXPECT_TRUE(is_sentinel(o));
}

TEST(RotorSpeedRefusal, RefusedCallsLeaveATwinBitIdentical) {
  const std::array<double, MARV_PLANT_N_MOTORS> start = {500.0, 800.0, 1200.0, 2000.0};
  PlantHandle a(start);
  PlantHandle b(start);
  const marv_plant_rotor_speed_config good = grid_config(kLatency);

  // Plant a takes refused calls around its attach; plant b does not.
  marv_plant_rotor_speed_out o = kSentinel;
  EXPECT_EQ(marv_plant_rotor_speed_sample(a.get(), &o), MARV_PLANT_E_STATE);
  marv_plant_rotor_speed_config bad = good;
  bad.pole_count = 13;
  EXPECT_EQ(marv_plant_rotor_speed_attach(a.get(), &bad), MARV_PLANT_E_CONFIG);
  bad = good;
  bad.struct_size = 1;
  EXPECT_EQ(marv_plant_rotor_speed_attach(a.get(), &bad), MARV_PLANT_E_ABI);
  ASSERT_EQ(marv_plant_rotor_speed_attach(a.get(), &good), MARV_PLANT_OK);
  ASSERT_EQ(marv_plant_rotor_speed_attach(b.get(), &good), MARV_PLANT_OK);
  // A second attach, with other values, is refused and changes nothing.
  marv_plant_rotor_speed_config other = grid_config(0, kPoleCount + 2);
  EXPECT_EQ(marv_plant_rotor_speed_attach(a.get(), &other), MARV_PLANT_E_STATE);
  EXPECT_EQ(marv_plant_rotor_speed_sample(a.get(), nullptr), MARV_PLANT_E_NULL);

  const std::vector<marv_plant_rotor_speed_out> run_a = run(a.get());
  const std::vector<marv_plant_rotor_speed_out> run_b = run(b.get());
  EXPECT_TRUE(same_run(run_a, run_b));

  // Control: a twin attached with the refused second config gives a different run.
  PlantHandle c(start);
  ASSERT_EQ(marv_plant_rotor_speed_attach(c.get(), &other), MARV_PLANT_OK);
  EXPECT_FALSE(same_run(run_a, run(c.get())));
}

}  // namespace
