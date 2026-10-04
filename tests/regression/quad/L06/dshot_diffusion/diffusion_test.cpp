// L6 stage (d), packet P1, T1 (decision 0017): DShot error diffusion, marv::mixer::DshotDiffuser. Not live: no
// composition uses it, and thrust_to_dshot, which now takes d* and d_lo from the same helpers as the diffuser, is
// unchanged bit for bit (section 5).
//
// The law, per motor and write n (mixer.hpp): d~_n = d*_n clamped to [d_lo, d_max]; u_n = fl(d~_n + e_{n-1});
// q_n = round(u_n) clamped to [d_lo, d_max]; e_n = u_n - q_n. The addition is the one rounded operation: the clamps
// select, round returns an integer, and e_n is exact (Sterbenz: q_n >= 48 and |u_n - q_n| <= 1/2). With
// rho_n = u_n - (d~_n + e_{n-1}) its rounding error, q_n - d~_n = e_{n-1} - e_n + rho_n, so over any N consecutive
// writes without a reset the sum telescopes, whatever the request sequence:
//   sum(q - d~) = e_first - e_last + sum(rho),   |mean(q) - mean(d~)| <= 1/N + delta_max   (|e| <= 1/2 at both ends).
//
// delta_max, derived: u_n lies in [d_lo - 1/2, d_max + 1/2] (d~ in [d_lo, d_max], |e| <= 1/2, both ends exact floats,
// rounding is monotone), inside [2^5, 2^11) because d_lo >= 48. The float spacing there is at most 2^(10 - 23) = 2^-13
// (the binade [2^10, 2^11)), and round to nearest errs by at most half a spacing: |rho_n| <= 2^-14 = 6.10e-5 of a step.
// It bounds each write. The window term is the mean of N of them, so the same delta_max holds for every N; it does not
// grow with N. The architect's draft, 2047.5 eps / 2 = 1.22e-4, is the standard model's u |x| bound on the same
// addition. Half the spacing is tighter by 2047.5 / 2^10, and it is attained (section 2 prints the measured maximum).
//
// The checker is exact: every d~ and u is a float in [2^5, 2^11], a multiple of 2^-18, and so is every e, so the sums
// and differences it forms in double carry no rounding.
//
// Numbers here are a cited constant (the DShot range, the float format), derived (the rule is on the line), or a
// "scenario test value" named with its reason. The product mixer comes from the parameter set.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include <marv/mixer/mixer.hpp>
#include <marv/params/param.hpp>
#include <marv/prim/constants.hpp>
#include <marv/types/actuator.hpp>

namespace {

using namespace marv;
using namespace marv::mixer;

constexpr std::size_t kN = kMotors;
using Cmd = std::array<float, kN>;
using Dshots = std::array<DshotValue, kN>;
using Rng = std::mt19937;
using Diffuser = DshotDiffuser<float>;

constexpr float kDMax = static_cast<float>(prim::kDshotThrottleMax);
constexpr double kHalf = 0.5;
constexpr float kHalfF = 0.5F;
constexpr float kInf = std::numeric_limits<float>::infinity();
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kFltMax = std::numeric_limits<float>::max();
constexpr double kNegInfD = -std::numeric_limits<double>::infinity();

// delta_max (file comment): half the float spacing at the largest |u|, d_max + 1/2. The spacing in the binade
// [2^k, 2^(k+1)) is 2^(k - (p - 1)), p the float significand bits (24), so half of it is 2^(k - p); here k = 10.
double delta_max() {
  return std::ldexp(1.0, std::ilogb(static_cast<double>(kDMax) + kHalf) - std::numeric_limits<float>::digits);
}

// The window bound's lead term |e_first - e_last|: at most 1 in general, at most 1/2 for a window that starts at a
// reset (e = 0 there).
constexpr double kLeadAny = 1.0;
constexpr double kLeadFromReset = kHalf;

// ---- scenario test values ------------------------------------------------------------------------------------
// Run lengths in writes: 2^20 for the long runs (the random sequence, the ramps), 2^18 for the near-1/2 and the
// thrust-driven random sequences, 2^16 for every other one. Long enough to visit every carry state many times, short
// enough for seconds in a debug build.
constexpr std::size_t kLongRun = std::size_t{1} << 20;
constexpr std::size_t kMidRun = std::size_t{1} << 18;
constexpr std::size_t kRun = std::size_t{1} << 16;
// Seeds of the per-motor mt19937 streams (motor i uses seed + i): committed, arbitrary, distinct per sequence.
constexpr std::uint32_t kSeedRandom = 1;
constexpr std::uint32_t kSeedNearHalf = 101;
constexpr std::uint32_t kSeedThrust = 201;
constexpr std::uint32_t kSeedSaturation = 301;
constexpr std::uint32_t kSeedReset = 401;
constexpr std::uint32_t kSeedShuffle = 501;
// Near-1/2 requests: m + 1/2 moved by up to this many float steps either way.
constexpr int kNearHalfSteps = 4;
// Fractions that are not dyadic (0.3, 0.7, 1/3); 1/2, the rounding tie, is kHalf.
constexpr double kFracA = 0.3;
constexpr double kFracB = 0.7;
constexpr double kFracThird = 1.0 / 3.0;
// Ramp steps, DShot steps per write: a slow dyadic ramp (2^-8) over the whole range, a non-dyadic one (1/3), and a fine
// one (2^-12) over the top kTopSpan steps.
constexpr double kRampSlow = 0x1p-8;
constexpr double kRampFine = 0x1p-12;
constexpr double kTopSpan = 4.0;
// Saturation schedule: blocks of this many writes.
constexpr std::size_t kSatBlock = 512;
// The second configuration: the product's, with the idle speed 0.3 of the way from omega_min to omega_max, so d_lo is
// far above kDshotThrottleMin (the product's idle is omega_min, d_lo = 48) and D(omega_idle) is not an integer.
constexpr float kRaisedIdleFraction = 0.3F;
// Section 5 sweep: kSweep thrusts evenly over [-f_max / 8, 9 f_max / 8], the allocation's range [f_min, f_max]
// widened by an eighth of f_max on each side, so negative thrust (sqrt of a negative: NaN), sub-idle and above-maximum
// thrust are in it.
constexpr std::size_t kSweep = std::size_t{1} << 20;
constexpr double kSweepMargin = 0.125;
// Section 5: the search window, in float steps of f, for a thrust whose d* is an exact integer.
constexpr int kIntegerSearchSteps = 64;
// Reset tests: writes of a random sequence before the path, enough to leave a carry on every motor (asserted).
constexpr std::size_t kPrimeWrites = 16;
// Control C2: window lengths checked one by one.
constexpr std::size_t kControlMaxWindow = 16;

float plus(float x, double y) { return static_cast<float>(static_cast<double>(x) + y); }

// ---- configurations ------------------------------------------------------------------------------------------

struct Configs {
  MixerConfig<float> product;  // the product parameter set
  MixerConfig<float> raised;   // the product set with the raised idle above
};

const Configs* configs() {
  static const Configs* built = []() -> const Configs* {
    if (!params_init(param_defaults())) {
      return nullptr;
    }
    static Configs c;
    c.product = from_params();
    c.raised = c.product;
    c.raised.omega_idle =
        c.product.omega_min + kRaisedIdleFraction * (c.product.omega_max - c.product.omega_min);
    return &c;
  }();
  return built;
}

#define REQUIRE_CONFIGS()                    \
  const Configs* const cp = configs();       \
  ASSERT_NE(cp, nullptr) << "params_init";   \
  const Configs& cs = *cp

std::array<const MixerConfig<float>*, 2> both(const Configs& cs) { return {&cs.product, &cs.raised}; }

const char* name_of(const Configs& cs, const MixerConfig<float>* cfg) {
  return cfg == &cs.product ? "product" : "raised idle";
}

// ---- the law's request and the stateless rounding, written here ----------------------------------------------

// d~: the request clamped to [d_lo, d_max] as the law defines it (NaN and -inf to d_lo, +inf to d_max).
float clamp_request(float d, float d_lo) {
  if (!(d >= d_lo)) {
    return d_lo;
  }
  return d > kDMax ? kDMax : d;
}

// Stateless rounding (thrust_to_dshot's law): round(d~). d~ is in [d_lo, d_max], both integers, so the result is too.
std::uint16_t stateless(float d_star, float d_lo) {
  return static_cast<std::uint16_t>(std::round(clamp_request(d_star, d_lo)));
}

// ---- negative-control laws: the diffuser's law with one change each, written here ----------------------------

enum class Mutation { RoundThenClamp, CarryZeroed, SignFlipped };

class Mutant {
 public:
  Mutant(Mutation m, float d_lo) : m_(m), d_lo_(d_lo) {}

  Dshots apply_unrounded(const Cmd& d_star) {
    Dshots out{};
    for (std::size_t i = 0; i < kN; ++i) {
      const float d = m_ == Mutation::RoundThenClamp ? d_star[i] : clamp_request(d_star[i], d_lo_);
      const float u = m_ == Mutation::SignFlipped ? d - carry_[i] : d + carry_[i];
      float q = std::round(u);
      if (!(q >= d_lo_)) {
        q = d_lo_;
      } else if (q > kDMax) {
        q = kDMax;
      }
      carry_[i] = m_ == Mutation::CarryZeroed ? 0.0F : u - q;
      out[i] = DshotValue::from_raw(static_cast<std::uint16_t>(q)).value_or(DshotValue::stop());
    }
    return out;
  }

  const Cmd& carry() const { return carry_; }

 private:
  Mutation m_;
  float d_lo_;
  Cmd carry_{};
};

// Control C4: stateless rounding, the product's thrust_to_dshot itself.
struct Stateless {
  MixerConfig<float> cfg;
  Cmd zero{};
  Dshots apply(const Cmd& f) const { return thrust_to_dshot(cfg, f); }
  const Cmd& carry() const { return zero; }
};

// ---- the checker ---------------------------------------------------------------------------------------------

// Online checks of one motor's writes. Write n (1-based) has request d~_n, output q_n and carry e_n; P_n is the prefix
// sum of q - d~. A window (a, b] meets the mean bound iff |P_b - P_a| <= 1 + (b - a) delta, i.e. iff A_b - A_a <= 1
// and B_a - B_b <= 1 with A_n = P_n - n delta and B_n = P_n + n delta. Tracking min A and max B over the earlier
// prefixes checks every window, every start and every N >= 1, in one pass.
struct Check {
  double delta = 0;
  double e_prev = 0;
  std::size_t n = 0;
  double p = 0;
  double min_a = 0;
  double max_b = 0;
  bool range_ok = true;
  bool carry_ok = true;
  bool mean_ok = true;
  bool from_start_ok = true;  // every window from the run's start: |P_n| <= 1/2 + n delta (a run that starts at a reset)
  double max_abs_e = 0;
  double max_abs_rho = 0;
  double max_excess = kNegInfD;  // max over windows of |P_b - P_a| - (1 + (b - a) delta): <= 0 iff mean_ok
  std::size_t first_carry_fail = 0;  // 1-based write, 0 = none
  std::size_t first_mean_fail = 0;
};

Check start_check(float e0) {
  Check c;
  c.delta = delta_max();
  c.e_prev = static_cast<double>(e0);
  return c;
}

void observe(Check& c, float d_lo, float d_tilde, DshotValue out, float carry) {
  ++c.n;
  const double q = static_cast<double>(out.raw());
  const double d = static_cast<double>(d_tilde);
  const double e = static_cast<double>(carry);
  if (!(q >= static_cast<double>(d_lo) && q <= static_cast<double>(kDMax))) {
    c.range_ok = false;
  }
  const double abs_e = std::abs(e);
  if (!(abs_e <= kHalf)) {
    c.carry_ok = false;
    if (c.first_carry_fail == 0) {
      c.first_carry_fail = c.n;
    }
  }
  c.max_abs_e = std::max(c.max_abs_e, abs_e);
  c.max_abs_rho = std::max(c.max_abs_rho, std::abs((q + e) - (d + c.e_prev)));
  c.e_prev = e;
  c.p += q - d;
  const double nd = static_cast<double>(c.n) * c.delta;
  const double a = c.p - nd;
  const double b = c.p + nd;
  const double excess = std::max(a - c.min_a, c.max_b - b) - kLeadAny;
  c.max_excess = std::max(c.max_excess, excess);
  if (!(excess <= 0)) {
    c.mean_ok = false;
    if (c.first_mean_fail == 0) {
      c.first_mean_fail = c.n;
    }
  }
  if (!(std::abs(c.p) <= kLeadFromReset + nd)) {
    c.from_start_ok = false;
  }
  c.min_a = std::min(c.min_a, a);
  c.max_b = std::max(c.max_b, b);
}

struct Summary {
  bool range_ok = true;
  bool carry_ok = true;
  bool mean_ok = true;
  bool from_start_ok = true;
  double max_abs_e = 0;
  double max_abs_rho = 0;
  double max_excess = kNegInfD;
  std::size_t writes = 0;
  std::string first_failure;  // the first sequence and motor that failed a check
};

std::string write_or_none(std::size_t n) { return n == 0 ? std::string("none") : "write " + std::to_string(n); }

void add(Summary& s, const std::string& name, const std::array<Check, kN>& checks) {
  for (std::size_t i = 0; i < kN; ++i) {
    const Check& c = checks[i];
    if (!(c.range_ok && c.carry_ok && c.mean_ok) && s.first_failure.empty()) {
      s.first_failure = name + ", motor " + std::to_string(i + 1) + ": range " + (c.range_ok ? "ok" : "FAIL") +
                        ", first carry failure " + write_or_none(c.first_carry_fail) + ", first mean failure " +
                        write_or_none(c.first_mean_fail);
    }
    s.range_ok = s.range_ok && c.range_ok;
    s.carry_ok = s.carry_ok && c.carry_ok;
    s.mean_ok = s.mean_ok && c.mean_ok;
    s.from_start_ok = s.from_start_ok && c.from_start_ok;
    s.max_abs_e = std::max(s.max_abs_e, c.max_abs_e);
    s.max_abs_rho = std::max(s.max_abs_rho, c.max_abs_rho);
    s.max_excess = std::max(s.max_excess, c.max_excess);
    s.writes += c.n;
  }
}

// Feeds `length` writes of gen(n) (per-motor pre-round commands d*) to a law with apply_unrounded and carry(); d~ is
// the checker's own clamp of d*.
template <class Law, class Gen>
std::array<Check, kN> run_unrounded(Law& law, float d_lo, std::size_t length, Gen gen) {
  std::array<Check, kN> c;
  for (std::size_t i = 0; i < kN; ++i) {
    c[i] = start_check(law.carry()[i]);
  }
  for (std::size_t n = 0; n < length; ++n) {
    const Cmd d_star = gen(n);
    const Dshots q = law.apply_unrounded(d_star);
    for (std::size_t i = 0; i < kN; ++i) {
      observe(c[i], d_lo, clamp_request(d_star[i], d_lo), q[i], law.carry()[i]);
    }
  }
  return c;
}

// The same for a law driven by per-motor thrust (apply): d~ is the clamp of d* = dshot_unrounded(cfg, f).
template <class Law, class Gen>
std::array<Check, kN> run_thrust(Law& law, const MixerConfig<float>& cfg, std::size_t length, Gen gen) {
  const float d_lo = dshot_floor(cfg);
  std::array<Check, kN> c;
  for (std::size_t i = 0; i < kN; ++i) {
    c[i] = start_check(law.carry()[i]);
  }
  for (std::size_t n = 0; n < length; ++n) {
    const Cmd f = gen(n);
    const Dshots q = law.apply(f);
    for (std::size_t i = 0; i < kN; ++i) {
      observe(c[i], d_lo, clamp_request(dshot_unrounded(cfg, f[i]), d_lo), q[i], law.carry()[i]);
    }
  }
  return c;
}

// Every window of exactly len writes: |sum(q - d~)| <= lead + len delta.
bool windows_hold(const std::vector<double>& diff, std::size_t len, double lead, double delta) {
  double sum = 0;
  for (std::size_t n = 0; n < diff.size(); ++n) {
    sum += diff[n];
    if (n >= len) {
      sum -= diff[n - len];
    }
    if (n + 1 >= len && !(std::abs(sum) <= lead + static_cast<double>(len) * delta)) {
      return false;
    }
  }
  return true;
}

// ---- sequences -----------------------------------------------------------------------------------------------

double unit(Rng& r) { return std::ldexp(static_cast<double>(r()), -static_cast<int>(Rng::word_size)); }

float uniform(Rng& r, double lo, double hi) { return static_cast<float>(lo + (hi - lo) * unit(r)); }

std::array<Rng, kN> streams(std::uint32_t seed) {
  std::array<Rng, kN> r;
  for (std::size_t i = 0; i < kN; ++i) {
    r[i].seed(seed + static_cast<std::uint32_t>(i));
  }
  return r;
}

// An integer request per motor, spread over [d_lo, d_max]: floor(d_lo + (d_max - d_lo) (i + 1) / (motors + 1)).
float spread(float d_lo, std::size_t i) {
  const double lo = static_cast<double>(d_lo);
  return static_cast<float>(
      std::floor(lo + (static_cast<double>(kDMax) - lo) * static_cast<double>(i + 1) / static_cast<double>(kN + 1)));
}

float below(float x) { return std::nextafter(x, -kInf); }
float above(float x) { return std::nextafter(x, kInf); }

// Uniform in [d_lo, d_max].
auto random_uniform(float d_lo, std::uint32_t seed) {
  return [rng = streams(seed), d_lo](std::size_t) mutable {
    Cmd d{};
    for (std::size_t i = 0; i < kN; ++i) {
      d[i] = uniform(rng[i], static_cast<double>(d_lo), static_cast<double>(kDMax));
    }
    return d;
  };
}

// m + 1/2 moved by k float steps: m uniform among the integers in [d_lo, d_max - 1], k uniform in
// [-kNearHalfSteps, kNearHalfSteps].
auto near_half(float d_lo, std::uint32_t seed) {
  return [rng = streams(seed), d_lo](std::size_t) mutable {
    const auto lo = static_cast<std::uint32_t>(d_lo);
    const auto count = static_cast<std::uint32_t>(prim::kDshotThrottleMax) - lo;
    const auto span = static_cast<std::uint32_t>(2 * kNearHalfSteps + 1);
    Cmd d{};
    for (std::size_t i = 0; i < kN; ++i) {
      const std::uint32_t m = lo + rng[i]() % count;
      const int k = static_cast<int>(rng[i]() % span) - kNearHalfSteps;
      float x = static_cast<float>(m) + kHalfF;
      for (int s = 0; s < std::abs(k); ++s) {
        x = k > 0 ? above(x) : below(x);
      }
      d[i] = x;
    }
    return d;
  };
}

auto constant(const Cmd& d) {
  return [d](std::size_t) { return d; };
}

auto alternating(const Cmd& even, const Cmd& odd) {
  return [even, odd](std::size_t n) { return n % 2 == 0 ? even : odd; };
}

// A triangle wave per motor: from start, span steps up and back down, step per write, starting phase writes in.
struct Ramp {
  double start;
  double span;
  double step;
  double phase;
};

auto ramps(const std::array<Ramp, kN>& r) {
  return [r](std::size_t n) {
    Cmd d{};
    for (std::size_t i = 0; i < kN; ++i) {
      const double period = 2 * r[i].span;
      const double x = std::fmod((static_cast<double>(n) + r[i].phase) * r[i].step, period);
      d[i] = static_cast<float>(r[i].start + (x <= r[i].span ? x : period - x));
    }
    return d;
  };
}

// Per-motor thrust uniform in [f_min, f_max], N: the allocation's range. With the raised idle, a thrust just above
// f_min maps below d_lo = ceil(D(omega_idle)) and is held at the floor, as in flight.
auto random_thrust(const MixerConfig<float>& cfg, std::uint32_t seed) {
  const double lo = static_cast<double>(f_min(cfg));
  const double hi = static_cast<double>(f_max(cfg));
  return [rng = streams(seed), lo, hi](std::size_t) mutable {
    Cmd f{};
    for (std::size_t i = 0; i < kN; ++i) {
      f[i] = uniform(rng[i], lo, hi);
    }
    return f;
  };
}

// Every in-range sequence of sections 1 and 2 (d* in [d_lo, d_max] on every write), each four per-motor sequences.
template <class Visit>
void in_range_sequences(float d_lo, Visit&& visit) {
  const float lo = d_lo;
  const float m0 = spread(lo, 0);
  const float m1 = spread(lo, 1);
  const float m2 = spread(lo, 2);
  const float m3 = spread(lo, 3);
  const double span = static_cast<double>(kDMax) - static_cast<double>(lo);
  visit("random uniform", kLongRun, random_uniform(lo, kSeedRandom));
  visit("near 1/2", kMidRun, near_half(lo, kSeedNearHalf));
  visit("constant m + 0.3, m + 1/2 and its two neighbours", kRun,
        constant({plus(m0, kFracA), plus(m1, kHalf), below(plus(m2, kHalf)), above(plus(m3, kHalf))}));
  visit("constant m + 1/3, m + 0.7, one float step above m and below m + 1", kRun,
        constant({plus(m0, kFracThird), plus(m1, kFracB), above(m2), below(m3 + 1.0F)}));
  visit("constant at d_lo and d_max and one float step inside", kRun,
        constant({lo, kDMax, above(lo), below(kDMax)}));
  visit("constant fractions at the ends", kRun,
        constant({plus(lo, kFracA), plus(kDMax, -kFracA), plus(lo, kHalf), plus(kDMax, -kHalf)}));
  visit("alternating: d_lo / d_max, ties, near ties, low / high binade", kRun,
        alternating({lo, plus(m1, kHalf), below(plus(m2, kHalf)), plus(lo, kFracThird)},
                    {kDMax, plus(m2, kHalf), above(plus(m2, kHalf)), plus(m3, kFracA)}));
  visit("alternating: wide swings, 0.3 / 0.7, top and bottom ties", kRun,
        alternating({plus(lo, kFracA), plus(m1, kFracA), kDMax, lo},
                    {plus(kDMax, -kFracA), plus(m1, kFracB), plus(kDMax, -kHalf), plus(lo, kHalf)}));
  visit("ramps: slow up, slow down, 1/3 steps, fine at the top", kLongRun,
        ramps({Ramp{static_cast<double>(lo), span, kRampSlow, 0},
               Ramp{static_cast<double>(lo), span, kRampSlow, span / kRampSlow},
               Ramp{static_cast<double>(lo), span, kFracThird, 0},
               Ramp{static_cast<double>(kDMax) - kTopSpan, kTopSpan, kRampFine, 0}}));
}

Summary run_in_range(const MixerConfig<float>& cfg) {
  Summary s;
  const float d_lo = dshot_floor(cfg);
  in_range_sequences(d_lo, [&](const char* name, std::size_t length, auto gen) {
    Diffuser law;
    law.init(cfg);
    add(s, name, run_unrounded(law, d_lo, length, gen));
  });
  Diffuser law;
  law.init(cfg);
  add(s, "random thrust through apply", run_thrust(law, cfg, kMidRun, random_thrust(cfg, kSeedThrust)));
  return s;
}

// Sections 1 and 2 share one run per configuration.
const std::array<Summary, 2>& in_range_results(const Configs& cs) {
  static const std::array<Summary, 2> r{run_in_range(cs.product), run_in_range(cs.raised)};
  return r;
}

// Saturated and non-finite requests among in-range stretches. Block b of kSatBlock writes (motor i is i blocks ahead,
// so the motors are in different phases) cycles: in range, above d_max, in range, below d_lo, NaN, in range, then the
// same kinds alternating write by write (with a request half a step below d_max, so u = d_max + 1/2 ties occur).
auto saturation(float d_lo, std::uint32_t seed) {
  const std::array<float, 6> hi{above(kDMax), plus(kDMax, kFracA), plus(kDMax, kHalf), 2 * kDMax, kFltMax, kInf};
  const std::array<float, 8> lo{below(d_lo), plus(d_lo, -kFracA), plus(d_lo, -kHalf), 0.0F, -0.0F,
                                std::numeric_limits<float>::denorm_min(), -kFltMax, -kInf};
  enum Phase : std::size_t { kIn0, kAbove, kIn1, kBelow, kNan, kIn2, kMixed, kPhases };
  return [rng = streams(seed), hi, lo, d_lo](std::size_t n) mutable {
    Cmd d{};
    for (std::size_t i = 0; i < kN; ++i) {
      const std::size_t k = n + i;
      const float in_range = uniform(rng[i], static_cast<double>(d_lo), static_cast<double>(kDMax));
      switch (static_cast<Phase>((n / kSatBlock + i) % kPhases)) {
        case kAbove:
          d[i] = hi[k % hi.size()];
          break;
        case kBelow:
          d[i] = lo[k % lo.size()];
          break;
        case kNan:
          d[i] = kNaN;
          break;
        case kMixed: {
          const std::array<float, 6> mixed{hi[k % hi.size()], in_range, lo[k % lo.size()], plus(kDMax, -kHalf), kNaN,
                                           in_range};
          d[i] = mixed[k % mixed.size()];
          break;
        }
        default:
          d[i] = in_range;
          break;
      }
    }
    return d;
  };
}

// ---- reset paths ---------------------------------------------------------------------------------------------

void prime(Diffuser& d, float d_lo) {
  auto gen = random_uniform(d_lo, kSeedReset);
  for (std::size_t n = 0; n < kPrimeWrites; ++n) {
    (void)d.apply_unrounded(gen(n));
  }
}

bool all_nonzero(const Cmd& e) {
  return std::all_of(e.begin(), e.end(), [](float x) { return x != 0.0F; });
}

// A request per motor whose next output tells a cleared carry from the stale carry e: d~ = m + 1/2 - e/2 rounds to m
// statelessly and to m + 1 with e > 0 (to m + 1 and m with e < 0).
Cmd visible(const Cmd& m, const Cmd& e) {
  Cmd d{};
  for (std::size_t i = 0; i < kN; ++i) {
    d[i] = plus(m[i], kHalf - static_cast<double>(e[i]) * kHalf);
  }
  return d;
}

// After a reset path: every carry is zero and the next output is the stateless rounding of d~, while the copy that
// skipped the path (stale) gives a different output on every motor, so the check can fail.
void expect_cleared(Diffuser& d, Diffuser& stale, const Cmd& m, float d_lo, const char* path) {
  for (std::size_t i = 0; i < kN; ++i) {
    EXPECT_EQ(d.carry()[i], 0.0F) << path << ", motor " << i + 1;
  }
  const Cmd req = visible(m, stale.carry());
  const Dshots q = d.apply_unrounded(req);
  const Dshots q_stale = stale.apply_unrounded(req);
  for (std::size_t i = 0; i < kN; ++i) {
    EXPECT_EQ(q[i].raw(), stateless(req[i], d_lo)) << path << ", motor " << i + 1;
    EXPECT_NE(q_stale[i].raw(), stateless(req[i], d_lo)) << path << ": the stale carry is not visible, motor " << i + 1;
  }
}

Cmd near_floor(float d_lo) {
  Cmd m{};
  for (std::size_t i = 0; i < kN; ++i) {
    m[i] = d_lo + static_cast<float>(i);
  }
  return m;
}

Cmd spread_all(float d_lo) {
  Cmd m{};
  for (std::size_t i = 0; i < kN; ++i) {
    m[i] = spread(d_lo, i);
  }
  return m;
}

// ---- section 5 -----------------------------------------------------------------------------------------------

// thrust_to_dshot as committed before decision 0017 (mixer.hpp at 4a76755), verbatim but for T = float.
Dshots thrust_to_dshot_before(const MixerConfig<float>& cfg, const Cmd& f) {
  using std::ceil;
  using std::round;
  using std::sqrt;
  const float d_min = static_cast<float>(prim::kDshotThrottleMin);
  const float d_max = static_cast<float>(prim::kDshotThrottleMax);
  const float omega_span = cfg.omega_max - cfg.omega_min;
  const float dshot_span = d_max - d_min;
  const float d_lo = ceil(d_min + (cfg.omega_idle - cfg.omega_min) / omega_span * dshot_span);

  Dshots out{};
  for (std::size_t i = 0; i < kN; ++i) {
    const float omega = sqrt(f[i] / cfg.thrust_coeff);
    float d = round(d_min + (omega - cfg.omega_min) / omega_span * dshot_span);
    if (!(d >= d_lo)) {
      d = d_lo;
    } else if (d > d_max) {
      d = d_max;
    }
    out[i] = DshotValue::from_raw(static_cast<std::uint16_t>(d)).value_or(DshotValue::stop());
  }
  return out;
}

// kSweep thrusts evenly over [-kSweepMargin f_max, (1 + kSweepMargin) f_max], four per write (motor i takes point
// kN k + i), then the special values.
std::vector<Cmd> thrust_sweep(const MixerConfig<float>& cfg) {
  const double top = static_cast<double>(f_max(cfg));
  const double lo = -kSweepMargin * top;
  const double hi = (1 + kSweepMargin) * top;
  std::vector<Cmd> out;
  out.reserve(kSweep / kN + kN);
  for (std::size_t k = 0; k < kSweep / kN; ++k) {
    Cmd f{};
    for (std::size_t i = 0; i < kN; ++i) {
      const double t = static_cast<double>(kN * k + i) / static_cast<double>(kSweep - 1);
      f[i] = static_cast<float>(lo + (hi - lo) * t);
    }
    out.push_back(f);
  }
  const float fl = f_min(cfg);
  const float fh = f_max(cfg);
  out.push_back({kNaN, kInf, -kInf, 0.0F});
  out.push_back({-0.0F, std::numeric_limits<float>::denorm_min(), std::numeric_limits<float>::min(), kFltMax});
  out.push_back({-kFltMax, fl, fh, above(fh)});
  out.push_back({below(fl), -std::numeric_limits<float>::min(), above(fl), below(fh)});
  return out;
}

// For each integer n in [d_lo, d_max], a float thrust f with dshot_unrounded(cfg, f) == n exactly, if one lies within
// kIntegerSearchSteps float steps of the double-precision inverse k omega_n^2. d*(f) is non-decreasing in f (every
// operation in it is monotone), so the walk goes one way.
std::vector<float> integer_thrusts(const MixerConfig<float>& cfg) {
  const double d_min = static_cast<double>(prim::kDshotThrottleMin);
  const double d_max = static_cast<double>(kDMax);
  const double w_lo = static_cast<double>(cfg.omega_min);
  const double w_span = static_cast<double>(cfg.omega_max) - w_lo;
  std::vector<float> out;
  for (auto n = static_cast<std::uint32_t>(dshot_floor(cfg)); n <= prim::kDshotThrottleMax; ++n) {
    const double omega = w_lo + (static_cast<double>(n) - d_min) / (d_max - d_min) * w_span;
    float f = static_cast<float>(static_cast<double>(cfg.thrust_coeff) * omega * omega);
    const auto target = static_cast<float>(n);
    const bool up = dshot_unrounded(cfg, f) < target;
    for (int s = 0; s <= kIntegerSearchSteps; ++s) {
      const float d = dshot_unrounded(cfg, f);
      if (d == target) {
        out.push_back(f);
        break;
      }
      if (up ? d > target : d < target) {
        break;
      }
      f = up ? above(f) : below(f);
    }
  }
  return out;
}

bool same(const Dshots& a, const Dshots& b) { return a == b; }

// ===== setup =====================================================================================================

TEST(L6DshotDiffusionSetup, BothConfigurationsAreValidAndTheRaisedFloorIsNotAnInteger) {
  REQUIRE_CONFIGS();
  EXPECT_EQ(validate(cs.product), ConfigError::None);
  EXPECT_EQ(validate(cs.raised), ConfigError::None);
  EXPECT_EQ(dshot_floor(cs.product), static_cast<float>(prim::kDshotThrottleMin));
  const float raised = dshot_of_speed(cs.raised, cs.raised.omega_idle);
  EXPECT_GT(dshot_floor(cs.raised), static_cast<float>(prim::kDshotThrottleMin));
  EXPECT_NE(dshot_floor(cs.raised), raised) << "D(omega_idle) of the raised configuration is an integer";
  std::printf("[diffusion] d_lo: product %.9g, raised idle %.9g (D(omega_idle) = %.9g); delta_max = %.9g\n",
              static_cast<double>(dshot_floor(cs.product)), static_cast<double>(dshot_floor(cs.raised)),
              static_cast<double>(raised), delta_max());
}

// ===== 1. the carry ==============================================================================================

TEST(L6DshotDiffusion, CarryStaysWithinHalfAStepOnEveryWrite) {
  REQUIRE_CONFIGS();
  const std::array<Summary, 2>& r = in_range_results(cs);
  for (std::size_t c = 0; c < r.size(); ++c) {
    const Summary& s = r[c];
    EXPECT_TRUE(s.carry_ok) << name_of(cs, both(cs)[c]) << ": " << s.first_failure;
    EXPECT_TRUE(s.range_ok) << name_of(cs, both(cs)[c]) << ": " << s.first_failure;
    EXPECT_LE(s.max_abs_e, kHalf);
    std::printf("[diffusion] %s: %zu motor-writes, max |e| = %.9g (bound 1/2)\n", name_of(cs, both(cs)[c]), s.writes,
                s.max_abs_e);
  }
}

// ===== 2. the window mean ========================================================================================

TEST(L6DshotDiffusion, WindowMeanMatchesTheRequestWithinTheBoundForEveryWindow) {
  REQUIRE_CONFIGS();
  const std::array<Summary, 2>& r = in_range_results(cs);
  for (std::size_t c = 0; c < r.size(); ++c) {
    const Summary& s = r[c];
    EXPECT_TRUE(s.mean_ok) << name_of(cs, both(cs)[c]) << ": " << s.first_failure;
    // The derived per-write bound itself: |rho| <= delta_max on every write.
    EXPECT_LE(s.max_abs_rho, delta_max()) << name_of(cs, both(cs)[c]);
    std::printf("[diffusion] %s: measured max |rho| = %.9g step against delta_max = %.9g (ratio %.4f); worst window "
                "|sum(q - d~)| - (1 + N delta_max) = %.9g step\n",
                name_of(cs, both(cs)[c]), s.max_abs_rho, delta_max(), s.max_abs_rho / delta_max(), s.max_excess);
  }
}

// ===== 3. saturation =============================================================================================

TEST(L6DshotDiffusion, SaturationClampsTheOutputAndCarriesNoWindup) {
  REQUIRE_CONFIGS();
  for (const MixerConfig<float>* cfg : both(cs)) {
    const float d_lo = dshot_floor(*cfg);
    Diffuser law;
    law.init(*cfg);
    auto gen = saturation(d_lo, kSeedSaturation);
    std::array<Check, kN> c;
    for (std::size_t i = 0; i < kN; ++i) {
      c[i] = start_check(0.0F);
    }
    std::size_t out_of_range = 0;
    std::size_t law_mismatch = 0;
    std::size_t top_mismatch = 0;
    std::size_t bottom_mismatch = 0;
    for (std::size_t n = 0; n < kRun; ++n) {
      const Cmd d_star = gen(n);
      const Cmd e_prev = law.carry();
      const Dshots q = law.apply_unrounded(d_star);
      for (std::size_t i = 0; i < kN; ++i) {
        const float dt = clamp_request(d_star[i], d_lo);
        // The carry comes from the clamped request: u = fl(d~ + e), q = round(u) clamped, e = u - q.
        const float u = dt + e_prev[i];
        float qe = std::round(u);
        if (!(qe >= d_lo)) {
          qe = d_lo;
        } else if (qe > kDMax) {
          qe = kDMax;
        }
        if (q[i].raw() != static_cast<std::uint16_t>(qe) || law.carry()[i] != u - qe) {
          ++law_mismatch;
        }
        // Held at a limit, the output is that limit: at d_max always (u in [d_max - 1/2, d_max + 1/2]); at d_lo unless
        // the carry is the top's +1/2 (d_lo + e is exact for both floors, 48 and 648, which lie in the lowest binade
        // u reaches, so u < d_lo + 1/2).
        if (dt == kDMax && q[i].raw() != prim::kDshotThrottleMax) {
          ++top_mismatch;
        }
        if (dt == d_lo && e_prev[i] < kHalfF && q[i].raw() != static_cast<std::uint16_t>(d_lo)) {
          ++bottom_mismatch;
        }
        if (!(d_star[i] >= d_lo && d_star[i] <= kDMax)) {
          ++out_of_range;
        }
        observe(c[i], d_lo, dt, q[i], law.carry()[i]);
      }
    }
    Summary s;
    add(s, "saturation", c);
    const char* name = name_of(cs, cfg);
    EXPECT_GT(out_of_range, std::size_t{0}) << name;
    EXPECT_TRUE(s.range_ok) << name << ": " << s.first_failure;
    EXPECT_TRUE(s.carry_ok) << name << ": " << s.first_failure;
    // No windup: every window, including those that start or end inside saturation or just after it, keeps the bound.
    EXPECT_TRUE(s.mean_ok) << name << ": " << s.first_failure;
    EXPECT_EQ(law_mismatch, std::size_t{0}) << name;
    EXPECT_EQ(top_mismatch, std::size_t{0}) << name;
    EXPECT_EQ(bottom_mismatch, std::size_t{0}) << name;
    std::printf("[diffusion] saturation, %s: %zu of %zu motor-writes out of range or not finite; max |e| = %.9g\n", name,
                out_of_range, s.writes, s.max_abs_e);
  }
}

// ===== 4. reset paths ============================================================================================

// Init: composition init (RateGroupStep::init) binds the configuration with every carry zero, on a new diffuser and
// on one that has run (a new SIL session re-runs init); the first write is thrust_to_dshot's.
TEST(L6DshotDiffusionReset, InitClearsTheCarry) {
  REQUIRE_CONFIGS();
  for (const MixerConfig<float>* cfg : both(cs)) {
    const float d_lo = dshot_floor(*cfg);
    Diffuser fresh;
    fresh.init(*cfg);
    for (std::size_t i = 0; i < kN; ++i) {
      EXPECT_EQ(fresh.carry()[i], 0.0F);
    }
    const Cmd f = random_thrust(*cfg, kSeedReset)(0);
    EXPECT_TRUE(same(fresh.apply(f), thrust_to_dshot(*cfg, f))) << name_of(cs, cfg);

    Diffuser d;
    d.init(*cfg);
    prime(d, d_lo);
    ASSERT_TRUE(all_nonzero(d.carry()));
    Diffuser stale = d;
    d.init(*cfg);
    expect_cleared(d, stale, spread_all(d_lo), d_lo, "init");
  }
}

// Mixer configuration reload: init with the new configuration clears every carry and binds the new map and floor; the
// next write is stateless under the new configuration.
TEST(L6DshotDiffusionReset, ConfigReloadClearsTheCarryAndBindsTheNewConfiguration) {
  REQUIRE_CONFIGS();
  const float lo_old = dshot_floor(cs.product);
  const float lo_new = dshot_floor(cs.raised);
  Diffuser d;
  d.init(cs.product);
  prime(d, lo_old);
  ASSERT_TRUE(all_nonzero(d.carry()));
  Diffuser stale = d;
  d.init(cs.raised);
  Diffuser reloaded = d;
  Diffuser stale_thrust = stale;
  expect_cleared(d, stale, spread_all(lo_new), lo_new, "config reload");

  // Through thrust: the same as thrust_to_dshot on the new configuration, including a thrust below the new floor
  // (motor 1, the old idle thrust), which the stale copy still maps to the old floor.
  Cmd f = random_thrust(cs.raised, kSeedReset)(0);
  f[0] = f_min(cs.product);
  EXPECT_TRUE(same(reloaded.apply(f), thrust_to_dshot(cs.raised, f)));
  EXPECT_EQ(reloaded.carry()[0], 0.0F) << "clamped to the new floor, an integer";
  EXPECT_NE(stale_thrust.apply(f)[0].raw(), static_cast<std::uint16_t>(lo_new));
}

// Disarm: the disarm path writes DshotValue::stop() on every motor itself, not through the diffuser, and calls
// reset(); the diffuser is not called while disarmed. Re-arm starts from idle: the first write is stateless, and every
// window of the spin-up from idle (a slow ramp from d_lo) starting at the re-arm stays within 1/(2N) + delta_max.
TEST(L6DshotDiffusionReset, DisarmClearsTheCarry) {
  REQUIRE_CONFIGS();
  for (const MixerConfig<float>* cfg : both(cs)) {
    const float d_lo = dshot_floor(*cfg);
    Diffuser d;
    d.init(*cfg);
    prime(d, d_lo);
    ASSERT_TRUE(all_nonzero(d.carry()));
    Diffuser stale = d;
    const Dshots disarm_write{};
    for (const DshotValue v : disarm_write) {
      EXPECT_EQ(v, DshotValue::stop());
    }
    d.reset();
    Diffuser armed = d;
    expect_cleared(d, stale, near_floor(d_lo), d_lo, "disarm");

    const double span = static_cast<double>(kDMax) - static_cast<double>(d_lo);
    const Ramp up{static_cast<double>(d_lo), span, kRampSlow, 0};
    Summary s;
    add(s, "spin-up after re-arm", run_unrounded(armed, d_lo, kRun, ramps({up, up, up, up})));
    EXPECT_TRUE(s.carry_ok && s.mean_ok) << name_of(cs, cfg) << ": " << s.first_failure;
    EXPECT_TRUE(s.from_start_ok) << name_of(cs, cfg);
  }
}

// Motor stop, armed and flying: the path writes DshotValue::stop() on every motor itself for some ticks, not through
// the diffuser, and calls reset(); the writes then resume. The first write after it is stateless, and every window
// from the resumption stays within 1/(2N) + delta_max: the carry from before the stop does not leak into it (the copy
// that skipped reset breaks that bound on the same requests).
TEST(L6DshotDiffusionReset, MotorStopClearsTheCarry) {
  REQUIRE_CONFIGS();
  for (const MixerConfig<float>* cfg : both(cs)) {
    const float d_lo = dshot_floor(*cfg);
    Diffuser d;
    d.init(*cfg);
    auto flight = random_uniform(d_lo, kSeedRandom);
    for (std::size_t n = 0; n < kRun; ++n) {
      (void)d.apply_unrounded(flight(n));
    }
    ASSERT_TRUE(all_nonzero(d.carry()));
    Diffuser stale = d;
    const Dshots stop_write{};
    for (const DshotValue v : stop_write) {
      EXPECT_EQ(v, DshotValue::stop());
    }
    d.reset();
    Diffuser resumed = d;
    Diffuser stale_resumed = stale;
    const Cmd hold = visible(spread_all(d_lo), stale.carry());
    expect_cleared(d, stale, spread_all(d_lo), d_lo, "motor stop");

    Summary s;
    add(s, "resumed after motor stop", run_unrounded(resumed, d_lo, kRun, constant(hold)));
    EXPECT_TRUE(s.carry_ok && s.mean_ok) << name_of(cs, cfg) << ": " << s.first_failure;
    EXPECT_TRUE(s.from_start_ok) << name_of(cs, cfg);
    Summary s_stale;
    add(s_stale, "resumed without reset", run_unrounded(stale_resumed, d_lo, kRun, constant(hold)));
    EXPECT_FALSE(s_stale.from_start_ok) << name_of(cs, cfg) << ": the stale carry did not break the bound";
  }
}

// ===== 5. bit identity ===========================================================================================

TEST(L6DshotDiffusionBitIdentity, ThrustToDshotIsUnchangedByTheRefactor) {
  REQUIRE_CONFIGS();
  for (const MixerConfig<float>* cfg : both(cs)) {
    const std::vector<Cmd> sweep = thrust_sweep(*cfg);
    std::size_t mismatches = 0;
    std::vector<bool> seen(static_cast<std::size_t>(prim::kDshotThrottleMax) + 1, false);
    for (const Cmd& f : sweep) {
      const Dshots now = thrust_to_dshot(*cfg, f);
      if (!same(now, thrust_to_dshot_before(*cfg, f))) {
        ++mismatches;
      }
      for (const DshotValue v : now) {
        seen[v.raw()] = true;
      }
    }
    EXPECT_EQ(mismatches, std::size_t{0}) << name_of(cs, cfg);
    const auto covered = static_cast<std::size_t>(std::count(seen.begin(), seen.end(), true));
    std::printf("[diffusion] bit identity, %s: %zu writes, %zu distinct DShot values of %u in [d_lo, d_max]\n",
                name_of(cs, cfg), sweep.size(), covered,
                static_cast<unsigned>(prim::kDshotThrottleMax - static_cast<std::uint16_t>(dshot_floor(*cfg)) + 1));
  }
}

TEST(L6DshotDiffusionBitIdentity, WithZeroCarryTheDiffuserEqualsThrustToDshot) {
  REQUIRE_CONFIGS();
  for (const MixerConfig<float>* cfg : both(cs)) {
    const char* name = name_of(cs, cfg);
    const float d_lo = dshot_floor(*cfg);

    // Any thrust, e = 0 (reset before each write): the output is thrust_to_dshot's.
    Diffuser d;
    d.init(*cfg);
    std::size_t mismatches = 0;
    for (const Cmd& f : thrust_sweep(*cfg)) {
      d.reset();
      if (!same(d.apply(f), thrust_to_dshot(*cfg, f))) {
        ++mismatches;
      }
    }
    EXPECT_EQ(mismatches, std::size_t{0}) << name;

    // Every integer d~ in [d_lo, d_max] on every motor (motor i starts i quarters of the way in), no reset: e stays
    // exactly 0 and q = d~.
    Diffuser z;
    z.init(*cfg);
    const auto lo = static_cast<std::uint32_t>(d_lo);
    const std::uint32_t count = static_cast<std::uint32_t>(prim::kDshotThrottleMax) - lo + 1;
    std::size_t integer_mismatches = 0;
    for (std::uint32_t k = 0; k < count; ++k) {
      Cmd req{};
      for (std::size_t i = 0; i < kN; ++i) {
        req[i] = static_cast<float>(lo + (k + static_cast<std::uint32_t>(i) * (count / kN)) % count);
      }
      const Dshots q = z.apply_unrounded(req);
      for (std::size_t i = 0; i < kN; ++i) {
        if (q[i].raw() != static_cast<std::uint16_t>(req[i]) || z.carry()[i] != 0.0F) {
          ++integer_mismatches;
        }
      }
    }
    EXPECT_EQ(integer_mismatches, std::size_t{0}) << name;

    // Through thrust: thrusts whose d* is an exact integer, ascending and then shuffled, no reset: the output equals
    // thrust_to_dshot's on every write and every carry stays exactly 0.
    std::vector<float> fs = integer_thrusts(*cfg);
    ASSERT_FALSE(fs.empty()) << name;
    std::vector<float> order = fs;
    Rng shuffle(kSeedShuffle);
    std::shuffle(order.begin(), order.end(), shuffle);
    order.insert(order.begin(), fs.begin(), fs.end());
    Diffuser t;
    t.init(*cfg);
    std::size_t thrust_mismatches = 0;
    for (std::size_t k = 0; k < order.size(); ++k) {
      Cmd f{};
      for (std::size_t i = 0; i < kN; ++i) {
        f[i] = order[(k + i * (order.size() / kN)) % order.size()];
      }
      if (!same(t.apply(f), thrust_to_dshot(*cfg, f)) || !std::all_of(t.carry().begin(), t.carry().end(),
                                                                      [](float e) { return e == 0.0F; })) {
        ++thrust_mismatches;
      }
    }
    EXPECT_EQ(thrust_mismatches, std::size_t{0}) << name;
    std::printf("[diffusion] %s: %zu of %u integer commands have a float thrust with d* exactly integral\n", name,
                fs.size(), static_cast<unsigned>(count));
  }
}

// ===== negative controls =========================================================================================

// C1: round, then clamp (the request is not clamped before the carry is added). In saturation the remainder is carried
// and grows without bound: it breaks section 3.
TEST(L6DshotDiffusionControl, RoundThenClampWindsUpAtSaturation) {
  REQUIRE_CONFIGS();
  for (const MixerConfig<float>* cfg : both(cs)) {
    const float d_lo = dshot_floor(*cfg);
    Mutant m(Mutation::RoundThenClamp, d_lo);
    Summary s;
    add(s, "round then clamp, saturation", run_unrounded(m, d_lo, kRun, saturation(d_lo, kSeedSaturation)));
    EXPECT_FALSE(s.carry_ok) << name_of(cs, cfg) << ": round then clamp kept |e| <= 1/2";
    std::printf("[diffusion] C1 round then clamp, %s: %s; max |e| = %.9g\n", name_of(cs, cfg),
                s.first_failure.c_str(), s.max_abs_e);
  }
}

// C2: the carry forced to 0, on the constant request m + 0.3 (section 2's sequence): every write errs by the fraction,
// so the window bound 1/N + delta_max fails exactly from N* = floor(1 / (frac - delta_max)) + 1 on (4 for 0.3), while
// the diffuser keeps it for every N.
TEST(L6DshotDiffusionControl, CarryForcedToZeroFailsTheMeanBoundBeyondThreeWrites) {
  REQUIRE_CONFIGS();
  const float d_lo = dshot_floor(cs.product);
  Cmd req{};
  for (std::size_t i = 0; i < kN; ++i) {
    req[i] = plus(spread(d_lo, i), kFracA);
  }
  Diffuser real;
  real.init(cs.product);
  Mutant zeroed(Mutation::CarryZeroed, d_lo);
  std::array<std::vector<double>, kN> diff_real;
  std::array<std::vector<double>, kN> diff_zeroed;
  for (std::size_t n = 0; n < kRun; ++n) {
    const Dshots qr = real.apply_unrounded(req);
    const Dshots qz = zeroed.apply_unrounded(req);
    for (std::size_t i = 0; i < kN; ++i) {
      diff_real[i].push_back(static_cast<double>(qr[i].raw()) - static_cast<double>(req[i]));
      diff_zeroed[i].push_back(static_cast<double>(qz[i].raw()) - static_cast<double>(req[i]));
    }
  }
  for (std::size_t i = 0; i < kN; ++i) {
    const double frac = static_cast<double>(req[i]) - std::floor(static_cast<double>(req[i]));
    const auto n_star = static_cast<std::size_t>(std::floor(1 / (frac - delta_max()))) + 1;
    for (std::size_t len = 1; len <= kControlMaxWindow; ++len) {
      EXPECT_EQ(windows_hold(diff_zeroed[i], len, kLeadAny, delta_max()), len < n_star)
          << "motor " << i + 1 << ", N = " << len << ", N* = " << n_star;
      EXPECT_TRUE(windows_hold(diff_real[i], len, kLeadAny, delta_max())) << "motor " << i + 1 << ", N = " << len;
    }
    std::printf("[diffusion] C2 carry forced to 0, motor %zu: fraction %.9g, the bound fails from N* = %zu on\n", i + 1,
                frac, n_star);
  }
}

// C3: the carry added with the wrong sign (u = d~ - e). |e| <= 1/2 still holds, so it breaks section 2, the mean.
TEST(L6DshotDiffusionControl, SignFlippedCarryFailsTheMeanBound) {
  REQUIRE_CONFIGS();
  for (const MixerConfig<float>* cfg : both(cs)) {
    const float d_lo = dshot_floor(*cfg);
    Mutant random_law(Mutation::SignFlipped, d_lo);
    Summary s_random;
    add(s_random, "sign flipped, random", run_unrounded(random_law, d_lo, kRun, random_uniform(d_lo, kSeedRandom)));
    Mutant constant_law(Mutation::SignFlipped, d_lo);
    Summary s_constant;
    add(s_constant, "sign flipped, constant m + 0.3", run_unrounded(constant_law, d_lo, kRun,
                                                                     constant({plus(spread(d_lo, 0), kFracA),
                                                                               plus(spread(d_lo, 1), kFracA),
                                                                               plus(spread(d_lo, 2), kFracA),
                                                                               plus(spread(d_lo, 3), kFracA)})));
    for (const Summary* s : {&s_random, &s_constant}) {
      EXPECT_FALSE(s->mean_ok) << name_of(cs, cfg) << ": the sign-flipped carry kept the mean bound";
      EXPECT_TRUE(s->carry_ok) << name_of(cs, cfg);
      std::printf("[diffusion] C3 sign flipped, %s: %s; worst window excess %.9g step\n", name_of(cs, cfg),
                  s->first_failure.c_str(), s->max_excess);
    }
  }
}

// C4: stateless rounding, thrust_to_dshot itself, on section 2's random thrust sequence: the diffuser keeps the bound
// on it (section 2), thrust_to_dshot does not.
TEST(L6DshotDiffusionControl, StatelessRoundingFailsTheMeanBound) {
  REQUIRE_CONFIGS();
  for (const MixerConfig<float>* cfg : both(cs)) {
    Stateless law{*cfg, {}};
    Summary s;
    add(s, "thrust_to_dshot, random thrust", run_thrust(law, *cfg, kMidRun, random_thrust(*cfg, kSeedThrust)));
    EXPECT_FALSE(s.mean_ok) << name_of(cs, cfg) << ": stateless rounding kept the mean bound";
    std::printf("[diffusion] C4 stateless, %s: %s; worst window excess %.9g step\n", name_of(cs, cfg),
                s.first_failure.c_str(), s.max_excess);
  }
}

}  // namespace
