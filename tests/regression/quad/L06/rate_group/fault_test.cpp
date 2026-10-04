// L6 stage (c), commit 3 (decision 0014, lead decision on the wiring): the gyro chain in front of the rate loop leaves
// the rate loop's fault semantics as they were at 69c62f2, before the chain. A sample the rate loop refuses (a gyro axis
// not finite, or GyroValid clear) is not filtered: the chain keeps its state and the rate loop gets the sample as given.
// On the rate tick where the rate loop reseeds (after a fault), the chain is reseeded to that tick's sample, so the two
// restart together; a refused sample on a non-rate tick is skipped and the chain continues (lead decisions).
//
// Paths, all on the same samples and the same parameters (the l4_rate_scripted set, the hover collective):
//  - the composition: l4_rate_scripted linked statically and ticked through hal_sim (the SIL boundary refuses a
//    non-finite sample, so the SIL entry cannot carry this input); its DShot per tick from the HAL latch;
//  - the shared step (fw/rate_group), the code the composition runs, whose executions expose the rate loop's flags;
//  - 69c62f2's path, reproduced here: a RateLoop fed the raw samples at the rate ticks, then allocate, record_allocation
//    and thrust_to_dshot as the composition did.
// Checks: the composition's DShot is the step's at every tick; fault_active, fault_latched and fault_count are 69c62f2's
// at every rate execution; at every fault execution the torque, the request and the thrusts are 69c62f2's bit for bit,
// and the step's and the composition's DShot is the DShot diffuser's write of those thrusts (decision 0017: the carry of
// the earlier executions, not reset by the fault), the diffuser run over the step's thrusts being the step's DShot at
// every execution (control: 69c62f2's stateless thrust_to_dshot of the same thrusts differs at some execution); a
// non-finite sample on a non-rate tick reaches no rate execution and the
// command is held over it; at the rate tick where the rate loop reseeds after a fault the chain's output equals that
// tick's sample within the seeding tolerance of d_term/chain_guard_test.cpp (seed_tolerance, the same rule on this
// chain's stages), and not before; after the non-rate-tick refusal the chain is not reseeded and its output differs
// from the chain that saw the clean sample by at most the one-step bound derived at that test.
// Controls: (1) the rate loop on the chain's held output for the non-finite sample (the chain's own guard, flags as
// given) does not fault; (2) without the reseed the chain's output after the outage blends its stale states and
// exceeds that tolerance; (3) a chain reseeded after the non-rate-tick refusal is within it (the no-reseed check breaks).
//
// Scenario test values (none a vehicle number): the run length, the ticks of the refused samples and the gyro signal
// g_a(n) = A_a sin(w_a n) + C_a; the hover collective is mass x the WGS 84 equatorial normal gravity of constants.hpp
// (NIMA TR8350.2 Table 3.4), as unit_l4_composition takes it.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <span>
#include <vector>

#include <marv/composition.hpp>
#include <marv/gyro_chain/gyro_chain.hpp>
#include <marv/hal/hal.hpp>
#include <marv/hal/tick.hpp>
#include <marv/hal_sim/hal_sim.hpp>
#include <marv/l4_script.hpp>
#include <marv/mixer/mixer.hpp>
#include <marv/params/param.hpp>
#include <marv/params/param_ids.hpp>
#include <marv/prim/constants.hpp>
#include <marv/prim/vec.hpp>
#include <marv/rate/rate_loop.hpp>
#include <marv/rate_group/rate_group.hpp>
#include <marv/types/actuator.hpp>
#include <marv/types/imu_sample.hpp>
#include <marv/types/rotor_speed_sample.hpp>

// The double stage helpers of the L6 gyro-chain tests (read-only use, as d_term/chain_guard_test.cpp).
#include "../gyro_chain/support.hpp"

namespace {

using namespace marv;
using V3 = prim::Vec3<float>;
using Raw = std::array<std::uint16_t, mixer::kMotors>;
constexpr std::size_t kAxes = 3;
constexpr std::size_t kSegments = 8;  // the register's segment capacity (l4_rate_scripted.cpp kSegmentIds)

// Scenario test values, in rate executions (tick = execution x D).
constexpr std::size_t kExecutions = 200;
constexpr std::size_t kNanExecution = 40;        // roll NaN on this rate tick
constexpr std::size_t kInfAfterExecution = 60;   // yaw +inf on the tick after this rate tick, a non-rate tick
constexpr std::size_t kInvalidFirst = 100;       // GyroValid clear from this rate tick ...
constexpr std::size_t kInvalidExecutions = 3;    // ... for this many rate periods
constexpr std::array<double, kAxes> kAmp{0.4, -0.3, 0.2};
constexpr std::array<double, kAxes> kRadPerTick{0.01, 0.017, 0.023};
constexpr std::array<double, kAxes> kOffset{0.05, -0.04, 0.03};

struct Exec {
  rate::RateOutput<float> rate;
  V3 request;
  mixer::Allocation<float> alloc;
  Raw dshot{};
};

Raw raw_of(const std::array<DshotValue, mixer::kMotors>& d) {
  Raw r{};
  for (std::size_t m = 0; m < mixer::kMotors; ++m) {
    r[m] = d[m].raw();
  }
  return r;
}

bool same_bits(const V3& a, const V3& b) { return std::memcmp(a.e, b.e, sizeof(a.e)) == 0; }

struct Setup {
  std::uint32_t divisor = 0;
  hal_sim::TickPeriod period{};
  float thrust = 0.0F;
  std::size_t ticks = 0;
};

std::array<ParamRecord, kParamCount> g_table;

// The sample of tick n (file comment), its stamp from the HAL's rule.
ImuSample sample_at(const Setup& st, std::size_t n, bool with_inf = true) {
  ImuSample s{};
  s.t_us = hal_sim::stamp_us(st.period, n);
  const std::size_t d = st.divisor;
  if (n >= kInvalidFirst * d && n < (kInvalidFirst + kInvalidExecutions) * d) {
    return s;  // GyroValid clear: the gyro is exactly 0 (imu_sample.hpp)
  }
  s.flags = imu_flag(ImuFlag::GyroValid);
  for (std::size_t a = 0; a < kAxes; ++a) {
    s.gyro_rad_s[a] =
        static_cast<float>(kAmp[a] * std::sin(kRadPerTick[a] * static_cast<double>(n)) + kOffset[a]);
  }
  if (n == kNanExecution * d) {
    s.gyro_rad_s[0] = std::numeric_limits<float>::quiet_NaN();
  }
  if (with_inf && n == kInfAfterExecution * d + 1) {
    s.gyro_rad_s[2] = std::numeric_limits<float>::infinity();
  }
  return s;
}

bool finite(const ImuSample& s) {
  bool ok = true;
  for (std::size_t a = 0; a < kAxes; ++a) {
    ok = ok && std::isfinite(s.gyro_rad_s[a]);
  }
  return ok;
}

bool usable(const ImuSample& s) { return (s.flags & imu_flag(ImuFlag::GyroValid)) != 0 && finite(s); }

// The rate loop and the mixer after the sample handed to the rate loop, as the composition does it.
struct Tail {
  rate::RateLoop<float> loop;
  mixer::MixerConfig<float> mix;
  composition::SetpointScript<float, kSegments> script{};
  composition::Chirp<float> chirp{};
  float thrust = 0.0F;
  Exec run(const ImuSample& given) {
    Exec e;
    e.rate = loop.execute(given, composition::setpoint_at(script, given.t_us));
    e.request = e.rate.torque + composition::chirp_torque(chirp, given.t_us);
    e.alloc = mixer::allocate(mix, mixer::Request<float>{thrust, e.request});
    loop.record_allocation(e.request, e.alloc);
    e.dshot = raw_of(mixer::thrust_to_dshot(mix, e.alloc.f));
    return e;
  }
};

// Head: 69c62f2's path, the raw sample to the rate loop. HeldOutput: every sample into the chain (its guard holds the
// output for a non-finite one) and the chain's output to the rate loop. NoReseed: the step's wiring without the reseed.
enum class Wiring { Head, HeldOutput, NoReseed };

struct Runs {
  Setup st;
  std::vector<Raw> composition;  // per tick
  std::vector<Exec> step;        // per rate execution
  std::vector<ImuSample> step_input;  // per tick, the step's rate_input()
  std::vector<Exec> head;
  std::vector<Exec> held_output;
  std::vector<V3> no_reseed_input;  // per tick, the gyro the NoReseed wiring hands the rate loop
  std::vector<ImuSample> unrefused_input;  // per tick, rate_input() of a step whose +inf sample is the clean one
};

std::vector<Exec> run_wiring(const Setup& st, Wiring w, std::vector<V3>* input = nullptr) {
  Tail t;
  t.mix = mixer::load_config();
  t.loop.init(rate::load_config(), t.mix);
  t.thrust = st.thrust;
  gyro_chain::GyroChain<float> chain;
  chain.init(rate_group::load_chain_config());
  std::vector<Exec> out;
  for (std::size_t n = 0; n < st.ticks; ++n) {
    const ImuSample s = sample_at(st, n);
    const bool due = n % st.divisor == 0;
    ImuSample given = s;
    if (w != Wiring::Head) {
      if (due) {
        chain.update_notches(RotorSpeedSample{});
      }
      if (w == Wiring::HeldOutput || usable(s)) {
        given.gyro_rad_s = chain.filter(s.gyro_rad_s);
      }
    }
    if (input != nullptr) {
      input->push_back(given.gyro_rad_s);
    }
    if (due) {
      out.push_back(t.run(given));
    }
  }
  return out;
}

const Runs& runs() {
  static const Runs r = [] {
    Runs x;
    const std::span<const ParamRecord, kParamCount> d = param_defaults();
    std::copy(d.begin(), d.end(), g_table.begin());
    // The hover collective as the composition's thrust request: the record's value, before the one params_init.
    const float mass = d[static_cast<std::size_t>(ParamId::mass)].value.f32;
    const float hover = mass * static_cast<float>(prim::kWgs84GammaE);
    g_table[static_cast<std::size_t>(ParamId::l4_thrust_n)].value = ParamValue{ParamType::F32, hover, 0};
    if (!params_init(std::span<const ParamRecord, kParamCount>(g_table))) {
      ADD_FAILURE() << "params_init";
      return x;
    }
    Setup st;
    st.divisor = static_cast<std::uint32_t>(param_value<ParamId::rate_loop_divisor>());
    st.period = hal_sim::TickPeriod{static_cast<std::uint32_t>(param_value<ParamId::tick_period_num_us>()),
                                    static_cast<std::uint32_t>(param_value<ParamId::tick_period_den>())};
    st.thrust = param_value<ParamId::l4_thrust_n>();
    st.ticks = kExecutions * st.divisor;

    std::array<DshotValue, composition::kMotors> latch{};
    hal_sim::setup(st.period, std::span<DshotValue>{latch}, std::span<ServoUs>{});
    composition::init();
    for (std::size_t n = 0; n < st.ticks; ++n) {
      hal_sim::begin_tick(n);
      composition::tick(sample_at(st, n));
      hal_sim::end_tick();
      x.composition.push_back(raw_of(latch));
    }

    rate_group::RateGroupStep step;
    const mixer::MixerConfig<float> mix = mixer::load_config();
    step.init(rate::load_config(), mix, rate_group::load_chain_config());
    for (std::size_t n = 0; n < st.ticks; ++n) {
      const ImuSample s = sample_at(st, n);
      const bool due = n % st.divisor == 0;
      step.filter(s, due);
      x.step_input.push_back(step.rate_input());
      if (due) {
        const V3 sp = composition::setpoint_at(composition::SetpointScript<float, kSegments>{}, s.t_us);
        const V3 added = composition::chirp_torque(composition::Chirp<float>{}, s.t_us);
        const rate_group::Execution e = step.execute(sp, added, st.thrust);
        x.step.push_back(Exec{e.rate, e.request, e.alloc, raw_of(e.dshot)});
      }
    }
    x.head = run_wiring(st, Wiring::Head);
    x.held_output = run_wiring(st, Wiring::HeldOutput);
    (void)run_wiring(st, Wiring::NoReseed, &x.no_reseed_input);
    rate_group::RateGroupStep clean;
    clean.init(rate::load_config(), mix, rate_group::load_chain_config());
    for (std::size_t n = 0; n < st.ticks; ++n) {
      const ImuSample s = sample_at(st, n, false);
      const bool due = n % st.divisor == 0;
      clean.filter(s, due);
      x.unrefused_input.push_back(clean.rate_input());
      if (due) {
        (void)clean.execute(composition::setpoint_at(composition::SetpointScript<float, kSegments>{}, s.t_us),
                            composition::chirp_torque(composition::Chirp<float>{}, s.t_us), st.thrust);
      }
    }
    x.st = st;
    return x;
  }();
  return r;
}

std::uint32_t divisor() { return static_cast<std::uint32_t>(param_value<ParamId::rate_loop_divisor>()); }

TEST(L6RateGroupFault, TheCompositionIsTheSharedStepAtEveryTick) {
  const Runs& r = runs();
  ASSERT_EQ(r.step.size(), kExecutions);
  const std::uint32_t d = divisor();
  ASSERT_GE(d, 2U);  // a non-rate tick exists
  ASSERT_EQ(r.composition.size(), kExecutions * d);
  for (std::size_t n = 0; n < r.composition.size(); ++n) {
    EXPECT_EQ(r.composition[n], r.step[n / d].dshot) << "tick " << n;
  }
}

TEST(L6RateGroupFault, FaultFlagsAreThoseOfThePathWithoutTheChainAtEveryExecution) {
  const Runs& r = runs();
  ASSERT_EQ(r.step.size(), kExecutions);
  ASSERT_EQ(r.head.size(), kExecutions);
  for (std::size_t k = 0; k < kExecutions; ++k) {
    EXPECT_EQ(r.step[k].rate.fault_active, r.head[k].rate.fault_active) << k;
    EXPECT_EQ(r.step[k].rate.fault_latched, r.head[k].rate.fault_latched) << k;
    EXPECT_EQ(r.step[k].rate.fault_count, r.head[k].rate.fault_count) << k;
  }
  // The faults are the refused rate ticks and nothing else: the NaN tick and the invalid ones; the +inf on a non-rate
  // tick reaches no execution.
  for (std::size_t k = 0; k < kExecutions; ++k) {
    const bool refused = k == kNanExecution || (k >= kInvalidFirst && k < kInvalidFirst + kInvalidExecutions);
    EXPECT_EQ(r.step[k].rate.fault_active, refused) << k;
  }
  EXPECT_EQ(r.step.back().rate.fault_count, 1 + kInvalidExecutions);
}

TEST(L6RateGroupFault, AtEveryFaultExecutionTheOutputIsThatOfThePathWithoutTheChain) {
  const Runs& r = runs();
  ASSERT_EQ(r.step.size(), kExecutions);
  const std::uint32_t d = divisor();
  std::size_t faults = 0;
  for (std::size_t k = 0; k < kExecutions; ++k) {
    if (!r.head[k].rate.fault_active) {
      continue;
    }
    ++faults;
    EXPECT_TRUE(same_bits(r.step[k].rate.torque, r.head[k].rate.torque)) << k;
    EXPECT_TRUE(same_bits(r.step[k].request, r.head[k].request)) << k;
    EXPECT_EQ(std::memcmp(r.step[k].alloc.f.data(), r.head[k].alloc.f.data(), sizeof(float) * mixer::kMotors), 0) << k;
  }
  EXPECT_EQ(faults, 1 + kInvalidExecutions);
  // The DShot (decision 0017): the diffuser from init over the step's thrusts, with 69c62f2's thrusts at its fault
  // executions, gives the step's DShot at every execution and the composition's at every fault execution.
  const mixer::MixerConfig<float> mix = mixer::load_config();
  mixer::DshotDiffuser<float> diffuser;
  diffuser.init(mix);
  bool stateless_differs = false;
  for (std::size_t k = 0; k < kExecutions; ++k) {
    const bool fault = r.head[k].rate.fault_active;
    const Raw want = raw_of(diffuser.apply(fault ? r.head[k].alloc.f : r.step[k].alloc.f));
    EXPECT_EQ(r.step[k].dshot, want) << k;
    if (fault) {
      EXPECT_EQ(r.composition[k * d], want) << k;
    }
    stateless_differs = stateless_differs || raw_of(mixer::thrust_to_dshot(mix, r.step[k].alloc.f)) != r.step[k].dshot;
  }
  EXPECT_TRUE(stateless_differs) << "control: 69c62f2's stateless rounding reproduces the step's DShot";
}

TEST(L6RateGroupFault, ControlTheChainsHeldOutputForTheNonFiniteSampleGivesNoFault) {
  const Runs& r = runs();
  ASSERT_EQ(r.held_output.size(), kExecutions);
  EXPECT_TRUE(r.head[kNanExecution].rate.fault_active);
  EXPECT_FALSE(r.held_output[kNanExecution].rate.fault_active);
}

// The seeding tolerance of d_term/chain_guard_test.cpp (seed_tolerance; derivation there), relative to |c|: per stage
// twice the DC-gain error of its float coefficients, plus gamma_5 x the sum of its absolute coefficients x (1 + the
// amplitude slack) x the l1 norm of the impulse response from that stage to the output. The stages are this chain's
// with every notch bypassed (no rotor speed staged): the identity notches are exact, the low-pass alone remains.
double seed_tolerance() {
  namespace gct = marv::gyro_chain::test;
  const gyro_chain::GyroChainConfig<float> cfg = rate_group::load_chain_config();
  const gct::Stage lp = gct::to_stage(gyro_chain::lowpass_coeffs<float>(cfg.cutoff_hz, cfg.period));
  const double gamma5 = 5.0 * gct::kU / (1.0 - 5.0 * gct::kU);
  gct::Stage rec;
  rec.a1 = lp.a1;
  rec.a2 = lp.a2;
  const std::vector<double> g = gct::run_stage(rec, gct::unit_impulse());
  const double terms = std::fabs(lp.b0) + std::fabs(lp.b1) + std::fabs(lp.b2) + std::fabs(lp.a1) + std::fabs(lp.a2);
  return 2.0 * std::fabs((lp.b0 + lp.b1 + lp.b2) / (1.0 + lp.a1 + lp.a2) - 1.0) +
         gamma5 * terms * (1.0 + gct::kAmplitudeSlack) * gct::l1(g);
}

// The largest |out - sample| / |sample| over the axes at tick n.
double relative_error(const Runs& r, const V3& out, std::size_t n) {
  const ImuSample c = sample_at(r.st, n);
  double worst = 0;
  for (std::size_t a = 0; a < kAxes; ++a) {
    worst = std::max(worst, std::fabs(static_cast<double>(out[a]) - static_cast<double>(c.gyro_rad_s[a])) /
                                std::fabs(static_cast<double>(c.gyro_rad_s[a])));
  }
  return worst;
}

// The rate ticks where the rate loop reseeds after a fault: after the NaN rate tick, and after the invalid run.
std::array<std::size_t, 2> rate_reseed_ticks(std::size_t d) {
  return {(kNanExecution + 1) * d, (kInvalidFirst + kInvalidExecutions) * d};
}

TEST(L6RateGroupFault, ANonFiniteSampleOnANonRateTickReachesNoExecutionAndTheCommandIsHeld) {
  const Runs& r = runs();
  ASSERT_EQ(r.step.size(), kExecutions);
  const std::uint32_t d = divisor();
  const std::size_t n = kInfAfterExecution * d + 1;
  ASSERT_NE(n % d, 0U);
  EXPECT_TRUE(std::isinf(r.step_input[n].gyro_rad_s[2]));  // held as given, not filtered
  EXPECT_EQ(r.composition[n], r.composition[n - 1]);       // no rate execution: the latch holds the command
  const std::size_t next = kInfAfterExecution + 1;          // the next rate execution
  EXPECT_FALSE(r.step[next].rate.fault_active);
  EXPECT_EQ(r.step[next].rate.fault_count, r.step[kInfAfterExecution].rate.fault_count);
  EXPECT_EQ(r.head[next].rate.fault_count, r.step[next].rate.fault_count);
}

TEST(L6RateGroupFault, TheChainIsReseededOnTheRateTickWhereTheRateLoopReseeds) {
  const Runs& r = runs();
  const std::uint32_t d = divisor();
  ASSERT_EQ(r.step_input.size(), kExecutions * d);
  const double tol = seed_tolerance();
  for (const std::size_t n : rate_reseed_ticks(d)) {
    EXPECT_LE(relative_error(r, r.step_input[n].gyro_rad_s, n), tol) << "tick " << n;
  }
  // Not before: the usable non-rate tick right after the NaN rate tick continues the chain.
  const std::size_t before = kNanExecution * d + 1;
  EXPECT_GT(relative_error(r, r.step_input[before].gyro_rad_s, before), tol);
}

TEST(L6RateGroupFault, ControlWithoutTheReseedTheChainBlendsItsStaleStatesAfterTheOutage) {
  const Runs& r = runs();
  ASSERT_EQ(r.no_reseed_input.size(), kExecutions * divisor());
  const std::size_t n = rate_reseed_ticks(divisor())[1];
  EXPECT_GT(relative_error(r, r.no_reseed_input[n], n), seed_tolerance());
}

// A test-local chain mirroring the step up to tick `upto`: refused samples skipped, a reseed on the rate ticks where the
// rate loop seeds (tick 0 and the first usable rate tick after a refused rate tick; every setpoint and output here is
// finite), and with `extra` one more reseed at `upto`. Its output at `upto`.
V3 mirror_chain(const Runs& r, std::size_t upto, bool extra) {
  gyro_chain::GyroChain<float> chain;
  chain.init(rate_group::load_chain_config());
  bool pending = true;
  V3 out;
  for (std::size_t m = 0; m <= upto; ++m) {
    const ImuSample s = sample_at(r.st, m);
    const bool due = m % r.st.divisor == 0;
    if (due) {
      chain.update_notches(RotorSpeedSample{});
    }
    if (!usable(s)) {
      pending = pending || due;
      continue;
    }
    if ((due && pending) || (extra && m == upto)) {
      chain.reseed();
    }
    pending = pending && !due;
    out = chain.filter(s.gyro_rad_s);
  }
  return out;
}

// After the +inf on a non-rate tick (n - 1), tick n: not reseeded, and within the one-step bound of the chain that saw the
// clean sample at n - 1. Both chains are identical up to n - 2 and only the low-pass acts (identity notches), so in exact
// arithmetic their difference at n is
//   b1 (g(n-1) - g(n-2)) + b2 (g(n-2) - g(n-3)) - a1 (y(n-1) - y(n-2)) - a2 (y(n-2) - y(n-3)),
// g the clean samples and y the clean chain's outputs: the chain's own one-tick increments. Bound: the absolute terms,
// plus the rounding of the two evaluations, 2 gamma_5 (|b0| + |b1| + |b2| + |a1| + |a2|) max|value| (support.hpp's
// local bound).
TEST(L6RateGroupFault, ARefusedSampleOnANonRateTickDoesNotReseedTheChain) {
  namespace gct = marv::gyro_chain::test;
  const Runs& r = runs();
  const std::uint32_t d = divisor();
  ASSERT_EQ(r.unrefused_input.size(), kExecutions * d);
  const std::size_t n = kInfAfterExecution * d + 2;
  const double tol = seed_tolerance();
  EXPECT_GT(relative_error(r, r.step_input[n].gyro_rad_s, n), tol);
  const V3 mirrored = mirror_chain(r, n, false);
  EXPECT_TRUE(same_bits(mirrored, r.step_input[n].gyro_rad_s));  // the mirror is the step's chain

  const gyro_chain::GyroChainConfig<float> cfg = rate_group::load_chain_config();
  const gct::Stage lp = gct::to_stage(gyro_chain::lowpass_coeffs<float>(cfg.cutoff_hz, cfg.period));
  const double gamma5 = 5.0 * gct::kU / (1.0 - 5.0 * gct::kU);
  const double coeffs = std::fabs(lp.b0) + std::fabs(lp.b1) + std::fabs(lp.b2) + std::fabs(lp.a1) + std::fabs(lp.a2);
  for (std::size_t a = 0; a < kAxes; ++a) {
    const auto g = [&](std::size_t m) { return static_cast<double>(sample_at(r.st, m, false).gyro_rad_s[a]); };
    const auto y = [&](std::size_t m) { return static_cast<double>(r.unrefused_input[m].gyro_rad_s[a]); };
    double biggest = 0;
    for (std::size_t m = n - 3; m <= n; ++m) {
      biggest = std::max({biggest, std::fabs(g(m)), std::fabs(y(m))});
    }
    const double bound = std::fabs(lp.b1) * std::fabs(g(n - 1) - g(n - 2)) + std::fabs(lp.b2) * std::fabs(g(n - 2) - g(n - 3)) +
                         std::fabs(lp.a1) * std::fabs(y(n - 1) - y(n - 2)) + std::fabs(lp.a2) * std::fabs(y(n - 2) - y(n - 3)) +
                         2.0 * gamma5 * coeffs * biggest;
    EXPECT_LE(std::fabs(static_cast<double>(r.step_input[n].gyro_rad_s[a]) - y(n)), bound) << "axis " << a;
  }
}

TEST(L6RateGroupFault, ControlAChainReseededAfterTheNonRateTickRefusalPassesAsReseeded) {
  const Runs& r = runs();
  const std::size_t n = kInfAfterExecution * divisor() + 2;
  EXPECT_LE(relative_error(r, mirror_chain(r, n, true), n), seed_tolerance());
}

}  // namespace
