// Frozen path: tests/regression/quad/L02/unit/adapter/adapter_test.cpp
// T1 test of the Gazebo adapter library (sim/gz, marv_gz_adapter, decision 0003 items 3, 5, 6 and 8). Host-free: no
// Gazebo header. Every numeric value in this file is a test value or a labelled scenario value (L1 test fixture),
// never vehicle data; the vehicle comes from the card header (MARV_PLANT_CARD_HEADER).
//
// Bounds (u = 2^-53, derivations in sim/gz/adapter/include/marv/gz/frames.hpp):
//   attitude, forward, against an independent Hamilton-product route:  <= 6 u ||q||          per component;
//   attitude, round trip:                                              <= (3 + 3 sqrt 2) u ||q|| per component;
//   body rates omega_frd = R(q)^T perm(omega), q unit:                 <= (5 sqrt 3 + 3) u ||omega||_2 per component,
//     against the same formula evaluated in long double (u_ld = 2^-64, its own bound added) and, within that plus
//     |1 - ||q||^2| |omega_i|, against the conjugated-quaternion rotation in long double.
// Everything else (vectors, plant outputs, tick ordering) is bit-exact.
//
// Every claim has an in-test negative control: a deliberately broken variant that must miss the bound by O(1) (that
// is, by more than 1e6 times the O(u) bound) or mismatch a bit-exact check.
#include <gtest/gtest.h>

#include <marv_sil.h>

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <limits>
#include <vector>

#include MARV_PLANT_CARD_HEADER
#include "marv/gz/adapter.hpp"
#include "marv/gz/constants.hpp"
#include "marv/gz/frames.hpp"
#include "marv/params/param_ids.hpp"
#include "marv/prim/quat.hpp"
#include "marv_plant.h"

namespace {

using marv::gz::Dshot;
using marv::gz::Quat;
using marv::gz::Vec3;

constexpr long double kU = static_cast<long double>(marv::gz::kUnitRoundoff);
constexpr long double kULong = std::numeric_limits<long double>::epsilon() / 2.0L;  // 2^-64
// A break is "O(1)" when it exceeds the O(u) bound by more than this factor (an O(1) error is ~1e16 u).
constexpr long double kBreakFactor = 1.0e6L;

// 1/sqrt 2 = 0.70710678118654752440..., the mathematical constant (math), independent of the adapter's sqrt 2 / 2.
constexpr double kInvSqrt2 = 0.70710678118654752440;
constexpr long double kSqrt3 = 1.73205080756887729353L;  // sqrt 3, the mathematical constant (math)

// Scenario values (L1 test fixture): the site of the plant; not vehicle data and not from the card.
constexpr double kScenarioLat = 0.82;      // rad
constexpr double kScenarioHeight = 500.0;  // m
// The SIL period the adapter is configured with: 625 / 4 us (scenario value, as the L2 open-loop test).
constexpr std::uint32_t kPeriodNumUs = 625;
constexpr std::uint32_t kPeriodDen = 4;

struct Rng {
  std::uint64_t s;
  std::uint64_t next() {  // splitmix64
    std::uint64_t z = (s += 0x9E3779B97F4A7C15ull);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
  }
  double unit() { return static_cast<double>(next() >> 11) * 0x1p-53; }  // [0, 1)
  double sym() { return (2.0 * unit()) - 1.0; }                          // [-1, 1)
};

template <class T>
bool same_bits(const T& a, const T& b) {
  return std::memcmp(&a, &b, sizeof(T)) == 0;
}

long double norm_of(const Quat& q) {
  long double n = 0.0L;
  for (double c : q) {
    n += static_cast<long double>(c) * static_cast<long double>(c);
  }
  return std::sqrt(n);
}

long double norm_of(const Vec3& v) {
  long double n = 0.0L;
  for (double c : v) {
    n += static_cast<long double>(c) * static_cast<long double>(c);
  }
  return std::sqrt(n);
}

Quat normalised(const Quat& q) {
  const double n = static_cast<double>(norm_of(q));
  return {q[0] / n, q[1] / n, q[2] / n, q[3] / n};
}

Quat conjugate(const Quat& q) { return {q[0], -q[1], -q[2], -q[3]}; }

Quat random_quat(Rng& r, bool unit) {
  for (;;) {
    const Quat q{r.sym(), r.sym(), r.sym(), r.sym()};
    if (norm_of(q) < 0.1L) {
      continue;
    }
    if (unit) {
      return normalised(q);
    }
    const double scale = std::ldexp(1.0, static_cast<int>(r.next() % 41) - 20);  // 2^-20 .. 2^20
    return {q[0] * scale, q[1] * scale, q[2] * scale, q[3] * scale};
  }
}

Vec3 random_vec(Rng& r, int exp_range) {
  const double scale = std::ldexp(1.0, static_cast<int>(r.next() % (2 * static_cast<unsigned>(exp_range) + 1)) - exp_range);
  return {r.sym() * scale, r.sym() * scale, r.sym() * scale};
}

// Deterministic edge cases of attitude: identity, w = 0, 90 and 180 degrees about each axis, both signs of q.
std::vector<Quat> edge_quats() {
  const double r = kInvSqrt2;
  std::vector<Quat> v = {
      {1, 0, 0, 0},  {-1, 0, 0, 0}, {0, 1, 0, 0},  {0, 0, 1, 0},  {0, 0, 0, 1},   {0, -1, 0, 0}, {0, 0, -1, 0},
      {0, 0, 0, -1}, {r, r, 0, 0},  {r, 0, r, 0},  {r, 0, 0, r},  {r, -r, 0, 0},  {r, 0, -r, 0},  {r, 0, 0, -r},
      {-r, r, 0, 0}, {-r, 0, r, 0}, {-r, 0, 0, r}, {0, r, r, 0},  {0, r, -r, 0},  {0, 0.6, 0.8, 0}, {0, 0, 0.6, 0.8},
  };
  return v;
}

// ---- vectors ------------------------------------------------------------------------------------------------------

struct VecRow {
  Vec3 in;
  Vec3 world;  // (y, x, -z), hand-written
  Vec3 body;   // (x, -y, -z), hand-written
};

const double kMax = DBL_MAX;
const double kSubA = DBL_TRUE_MIN;
const double kSubB = DBL_MIN / 2.0;  // subnormal
const double kInf = std::numeric_limits<double>::infinity();

const std::vector<VecRow>& vec_rows() {
  static const std::vector<VecRow> rows = {
      {{1.5, -2.5, 3.5}, {-2.5, 1.5, -3.5}, {1.5, 2.5, -3.5}},
      {{+0.0, -0.0, +0.0}, {-0.0, +0.0, -0.0}, {+0.0, +0.0, -0.0}},
      {{-0.0, +0.0, -0.0}, {+0.0, -0.0, +0.0}, {-0.0, -0.0, +0.0}},
      {{+0.0, +0.0, -0.0}, {+0.0, +0.0, +0.0}, {+0.0, -0.0, +0.0}},
      {{-0.0, -0.0, +0.0}, {-0.0, -0.0, -0.0}, {-0.0, +0.0, -0.0}},
      {{kSubA, -kSubB, kSubB}, {-kSubB, kSubA, -kSubB}, {kSubA, kSubB, -kSubB}},
      {{kMax, -kMax, kMax}, {-kMax, kMax, -kMax}, {kMax, kMax, -kMax}},
      {{DBL_MIN, kInf, -kInf}, {kInf, DBL_MIN, kInf}, {DBL_MIN, -kInf, kInf}},
      {{1.0e300, -1.0e-300, 0.1}, {-1.0e-300, 1.0e300, -0.1}, {1.0e300, 1.0e-300, -0.1}},
  };
  return rows;
}

TEST(GzAdapterVectors, MapsEqualTheHandWrittenTablesBitwise) {
  for (const VecRow& row : vec_rows()) {
    EXPECT_TRUE(same_bits(marv::gz::enu_to_ned_world(row.in), row.world));
    EXPECT_TRUE(same_bits(marv::gz::ned_to_enu_world(row.in), row.world));
    EXPECT_TRUE(same_bits(marv::gz::flu_to_frd_body(row.in), row.body));
    EXPECT_TRUE(same_bits(marv::gz::frd_to_flu_body(row.in), row.body));
  }
}

TEST(GzAdapterVectors, RoundTripsAreBitExactIncludingSignedZeros) {
  for (const VecRow& row : vec_rows()) {
    EXPECT_TRUE(same_bits(marv::gz::ned_to_enu_world(marv::gz::enu_to_ned_world(row.in)), row.in));
    EXPECT_TRUE(same_bits(marv::gz::enu_to_ned_world(marv::gz::ned_to_enu_world(row.in)), row.in));
    EXPECT_TRUE(same_bits(marv::gz::frd_to_flu_body(marv::gz::flu_to_frd_body(row.in)), row.in));
    EXPECT_TRUE(same_bits(marv::gz::flu_to_frd_body(marv::gz::frd_to_flu_body(row.in)), row.in));
  }
  // Any non-NaN bit pattern.
  Rng r{0x1234};
  std::size_t tested = 0;
  while (tested < 20000) {
    Vec3 v{};
    bool nan = false;
    for (double& c : v) {
      const std::uint64_t bits = r.next();
      std::memcpy(&c, &bits, sizeof(c));
      nan = nan || std::isnan(c);
    }
    if (nan) {
      continue;
    }
    ++tested;
    EXPECT_TRUE(same_bits(marv::gz::ned_to_enu_world(marv::gz::enu_to_ned_world(v)), v));
    EXPECT_TRUE(same_bits(marv::gz::frd_to_flu_body(marv::gz::flu_to_frd_body(v)), v));
  }
}

TEST(GzAdapterVectors, TheWrenchMapIsTheWorldPermutationOnForceAndTorque) {
  for (const VecRow& row : vec_rows()) {
    const Vec3 torque{row.in[2], row.in[0], row.in[1]};
    const marv::gz::Wrench w = marv::gz::wrench_ned_to_enu(row.in, torque);
    EXPECT_TRUE(same_bits(w.force, row.world));
    EXPECT_TRUE(same_bits(w.torque, Vec3{torque[1], torque[0], -torque[2]}));
  }
}

// Negative control: dropping the z negation must mismatch the table in every row (even a +0 gives the wrong sign).
TEST(GzAdapterVectors, NegativeControlDroppingTheZNegationMismatchesEveryRow) {
  std::size_t mismatched = 0;
  for (const VecRow& row : vec_rows()) {
    const Vec3 broken{row.in[1], row.in[0], row.in[2]};
    if (!same_bits(broken, row.world)) {
      ++mismatched;
    }
  }
  EXPECT_EQ(mismatched, vec_rows().size());
}

// ---- attitude -----------------------------------------------------------------------------------------------------

// The independent route: q_nb = q_ne (x) q_eu (x) q_fb with two Hamilton products.
Quat attitude_by_products(const Quat& q) {
  using Q = marv::prim::Quat<double>;
  const Q r = Q(0.0, kInvSqrt2, kInvSqrt2, 0.0) * Q(q[0], q[1], q[2], q[3]) * Q(0.0, 1.0, 0.0, 0.0);
  return {r.w, r.x, r.y, r.z};
}

// Largest |a_i - b_i| / (u ||q||) over the components.
long double component_ratio(const Quat& a, const Quat& b, long double norm) {
  long double worst = 0.0L;
  for (std::size_t i = 0; i < 4; ++i) {
    const long double err = std::fabs(static_cast<long double>(a[i]) - static_cast<long double>(b[i]));
    worst = std::max(worst, err / (kU * norm));
  }
  return worst;
}

std::vector<Quat> attitude_states() {
  std::vector<Quat> v = edge_quats();
  Rng r{0xA771};
  for (int i = 0; i < 8000; ++i) {
    v.push_back(random_quat(r, (i % 2) == 0));
  }
  return v;
}

constexpr long double kForwardBound = 6.0L;                            // in u ||q||
const long double kRoundTripBound = 3.0L + 3.0L * static_cast<long double>(marv::gz::kSqrt2);  // in u ||q||

TEST(GzAdapterAttitude, HandWrittenValuesAreBitExact) {
  const double s = marv::gz::kSqrt2 / 2.0;
  EXPECT_TRUE(same_bits(marv::gz::attitude_enu_flu_to_ned_frd({1, 0, 0, 0}), Quat{-s, -0.0, +0.0, -s}));
  EXPECT_TRUE(same_bits(marv::gz::attitude_enu_flu_to_ned_frd({0, 1, 0, 0}), Quat{-0.0, -s, -s, +0.0}));
  // The sign is not canonicalised: the identity does not map to w >= 0.
  EXPECT_LT(marv::gz::attitude_enu_flu_to_ned_frd({1, 0, 0, 0})[0], 0.0);
  EXPECT_EQ(marv::gz::attitude_enu_flu_to_ned_frd({-1, 0, 0, 0})[0], s);
  EXPECT_EQ(marv::gz::attitude_enu_flu_to_ned_frd({0, 0, 0, 0})[1], 0.0);
}

TEST(GzAdapterAttitude, ForwardMatchesTheHamiltonProductRouteWithinSixUnitRoundoffs) {
  long double worst = 0.0L;
  for (const Quat& q : attitude_states()) {
    const long double ratio =
        component_ratio(marv::gz::attitude_enu_flu_to_ned_frd(q), attitude_by_products(q), norm_of(q));
    worst = std::max(worst, ratio);
    EXPECT_LE(ratio, kForwardBound);
  }
  RecordProperty("worst_forward_ratio_in_u_norm_q", std::to_string(static_cast<double>(worst)));
  std::printf("worst forward error %.3f u||q|| (bound %.0f)\n", static_cast<double>(worst),
              static_cast<double>(kForwardBound));
}

TEST(GzAdapterAttitude, RoundTripIsWithinThreePlusThreeSqrtTwoUnitRoundoffs) {
  long double worst = 0.0L;
  for (const Quat& q : attitude_states()) {
    const Quat back = marv::gz::attitude_ned_frd_to_enu_flu(marv::gz::attitude_enu_flu_to_ned_frd(q));
    const long double ratio = component_ratio(back, q, norm_of(q));
    worst = std::max(worst, ratio);
    EXPECT_LE(ratio, kRoundTripBound);
  }
  RecordProperty("worst_round_trip_ratio_in_u_norm_q", std::to_string(static_cast<double>(worst)));
  std::printf("worst round-trip error %.3f u||q|| (bound %.3f)\n", static_cast<double>(worst),
              static_cast<double>(kRoundTripBound));
}

struct BreakStats {
  std::size_t cases = 0;
  std::size_t exceeding = 0;  // cases beyond the bound
  long double max_ratio = 0.0L;
};

// Negative controls of the attitude bound: each variant must miss the O(u) bound by O(1) on most states.
TEST(GzAdapterAttitude, NegativeControlsMissTheBoundByOrderOne) {
  BreakStats no_flip;
  BreakStats conj;
  for (const Quat& q : attitude_states()) {
    const long double n = norm_of(q);
    const Quat by_products = attitude_by_products(q);
    using Q = marv::prim::Quat<double>;
    // A missing body flip: q_ne (x) q only.
    const Q a = Q(0.0, kInvSqrt2, kInvSqrt2, 0.0) * Q(q[0], q[1], q[2], q[3]);
    // The conjugate of the result.
    const Quat b = conjugate(marv::gz::attitude_enu_flu_to_ned_frd(q));
    const struct {
      BreakStats* stats;
      Quat value;
    } variants[] = {{&no_flip, {a.w, a.x, a.y, a.z}}, {&conj, b}};
    for (const auto& v : variants) {
      const long double ratio = component_ratio(v.value, by_products, n);
      ++v.stats->cases;
      v.stats->exceeding += ratio > kForwardBound ? 1u : 0u;
      v.stats->max_ratio = std::max(v.stats->max_ratio, ratio);
    }
  }
  for (const BreakStats* s : {&no_flip, &conj}) {
    EXPECT_GT(s->max_ratio, kBreakFactor * kForwardBound);
    EXPECT_GT(s->exceeding * 10, s->cases * 9);  // more than 90 % of the states
  }
}

// ---- body rates ---------------------------------------------------------------------------------------------------

// omega_frd = R(q)^T v in long double, the same formula as the adapter (R entries as marv::prim::Quat).
Vec3 omega_formula_long_double(const Quat& q, const Vec3& omega_enu, long double out[3]) {
  const Vec3 vd = marv::gz::enu_to_ned_world(omega_enu);
  const long double w = q[0];
  const long double x = q[1];
  const long double y = q[2];
  const long double z = q[3];
  const long double v[3] = {vd[0], vd[1], vd[2]};
  const long double r[3][3] = {{1.0L - 2.0L * (y * y + z * z), 2.0L * (x * y - w * z), 2.0L * (x * z + w * y)},
                               {2.0L * (x * y + w * z), 1.0L - 2.0L * (x * x + z * z), 2.0L * (y * z - w * x)},
                               {2.0L * (x * z - w * y), 2.0L * (y * z + w * x), 1.0L - 2.0L * (x * x + y * y)}};
  for (std::size_t i = 0; i < 3; ++i) {
    out[i] = r[0][i] * v[0] + r[1][i] * v[1] + r[2][i] * v[2];
  }
  return vd;
}

// The conjugated-quaternion rotation, marv::prim::Quat<long double>::rotate (v + w t + u x t), an independent formula.
void omega_by_rotate(const Quat& q, const Vec3& omega_enu, long double out[3]) {
  using QL = marv::prim::Quat<long double>;
  const Vec3 vd = marv::gz::enu_to_ned_world(omega_enu);
  const QL c = QL(q[0], q[1], q[2], q[3]).conjugate();
  const auto r = c.rotate(marv::prim::Vec3<long double>(vd[0], vd[1], vd[2]));
  for (std::size_t i = 0; i < 3; ++i) {
    out[i] = r[i];
  }
}

struct OmegaCase {
  Quat q_nb;
  Vec3 omega_enu;
};

std::vector<OmegaCase> omega_cases() {
  std::vector<OmegaCase> v;
  Rng r{0x0E6A};
  for (const Quat& q : edge_quats()) {
    v.push_back({normalised(q), {0.3, -1.25, 2.0}});
    v.push_back({normalised(q), {0.0, 0.0, 0.0}});
    v.push_back({normalised(q), random_vec(r, 10)});
  }
  for (int i = 0; i < 8000; ++i) {
    v.push_back({random_quat(r, true), random_vec(r, 10)});
  }
  return v;
}

// (5 sqrt 3 + 3) u ||omega||, plus the same rule at the long double unit roundoff for the reference.
long double omega_bound(const Vec3& omega) {
  return (5.0L * kSqrt3 + 3.0L) * (kU + kULong) * norm_of(omega);
}

TEST(GzAdapterOmega, MatchesTheLongDoubleFormulaAndTheConjugateRotationWithinTheDerivedBound) {
  long double worst = 0.0L;
  for (const OmegaCase& c : omega_cases()) {
    const Vec3 got = marv::gz::omega_frd_from_world_enu(c.q_nb, c.omega_enu);
    long double ref[3];
    long double ref2[3];
    omega_formula_long_double(c.q_nb, c.omega_enu, ref);
    omega_by_rotate(c.q_nb, c.omega_enu, ref2);
    const long double n2 = static_cast<long double>(c.q_nb[0]) * c.q_nb[0] + static_cast<long double>(c.q_nb[1]) * c.q_nb[1] +
                           static_cast<long double>(c.q_nb[2]) * c.q_nb[2] + static_cast<long double>(c.q_nb[3]) * c.q_nb[3];
    const Vec3 v = marv::gz::enu_to_ned_world(c.omega_enu);
    const long double bound = omega_bound(c.omega_enu);
    for (std::size_t i = 0; i < 3; ++i) {
      const long double e1 = std::fabs(static_cast<long double>(got[i]) - ref[i]);
      // rotate() and the matrix differ by (1 - ||q||^2) v_i exactly; rotate() has its own ~64 ops of rounding.
      const long double e2 = std::fabs(static_cast<long double>(got[i]) - ref2[i]);
      const long double slack2 = bound + std::fabs(1.0L - n2) * std::fabs(static_cast<long double>(v[i])) +
                                 64.0L * kULong * norm_of(c.omega_enu);
      EXPECT_LE(e1, bound);
      EXPECT_LE(e2, slack2);
      if (norm_of(c.omega_enu) > 0.0L) {
        worst = std::max(worst, e1 / (kU * norm_of(c.omega_enu)));
      }
    }
  }
  RecordProperty("worst_omega_ratio_in_u_norm_omega", std::to_string(static_cast<double>(worst)));
  std::printf("worst omega error %.3f u||omega|| (bound %.3f)\n", static_cast<double>(worst),
              static_cast<double>(5.0L * kSqrt3 + 3.0L));
}

// Negative controls: the conjugate (R in place of R^T) and a dropped z negation in the world permutation.
TEST(GzAdapterOmega, NegativeControlsMissTheBoundByOrderOne) {
  BreakStats conj;
  BreakStats no_z;
  for (const OmegaCase& c : omega_cases()) {
    if (norm_of(c.omega_enu) == 0.0L) {
      continue;
    }
    const long double bound = omega_bound(c.omega_enu);
    long double ref[3];
    omega_formula_long_double(c.q_nb, c.omega_enu, ref);
    const Vec3 wrong_conj = marv::gz::omega_frd_from_world_enu(conjugate(c.q_nb), c.omega_enu);
    // ned = (y, x, z) instead of (y, x, -z) is the correct map applied to (ex, ey, -ez).
    const Vec3 wrong_z = marv::gz::omega_frd_from_world_enu(c.q_nb, {c.omega_enu[0], c.omega_enu[1], -c.omega_enu[2]});
    const struct {
      BreakStats* stats;
      Vec3 value;
    } variants[] = {{&conj, wrong_conj}, {&no_z, wrong_z}};
    for (const auto& v : variants) {
      long double err = 0.0L;
      for (std::size_t i = 0; i < 3; ++i) {
        err = std::max(err, std::fabs(static_cast<long double>(v.value[i]) - ref[i]));
      }
      ++v.stats->cases;
      v.stats->exceeding += err > bound ? 1u : 0u;
      v.stats->max_ratio = std::max(v.stats->max_ratio, err / bound);
    }
  }
  for (const BreakStats* s : {&conj, &no_z}) {
    EXPECT_GT(s->max_ratio, kBreakFactor);
    EXPECT_GT(s->exceeding * 10, s->cases * 9);
  }
}

TEST(GzAdapterOmega, ToPlantBodyAppliesTheMapsFieldByField) {
  Rng r{0xB0D1};
  for (int i = 0; i < 200; ++i) {
    const marv::gz::GzState s{random_vec(r, 8), random_quat(r, true), random_vec(r, 8), random_vec(r, 8)};
    const marv_plant_body b = marv::gz::to_plant_body(s);
    const Quat q = marv::gz::attitude_enu_flu_to_ned_frd(s.q_eu_wxyz);
    const Vec3 p = marv::gz::enu_to_ned_world(s.pos_enu_m);
    const Vec3 v = marv::gz::enu_to_ned_world(s.lin_vel_world_m_s);
    const Vec3 w = marv::gz::omega_frd_from_world_enu(q, s.ang_vel_world_rad_s);
    EXPECT_EQ(b.struct_size, sizeof(marv_plant_body));
    EXPECT_EQ(std::memcmp(b.pos_ned_m, p.data(), sizeof(b.pos_ned_m)), 0);
    EXPECT_EQ(std::memcmp(b.vel_ned_m_s, v.data(), sizeof(b.vel_ned_m_s)), 0);
    EXPECT_EQ(std::memcmp(b.q_wxyz, q.data(), sizeof(b.q_wxyz)), 0);
    EXPECT_EQ(std::memcmp(b.omega_frd_rad_s, w.data(), sizeof(b.omega_frd_rad_s)), 0);
  }
}

// ---- plant equality, ordering, real SIL ---------------------------------------------------------------------------

double t_tick() { return marv::gz::tick_period_s(kPeriodNumUs, kPeriodDen); }

// The card configuration with the scenario fields set. rng_seed is reserved and unused at v0 (marv_plant.h): the
// seed dimension is kept so that sensor noise at L6 changes no shape here, and at v0 it must change no output.
marv_plant_config card_config(std::uint64_t seed, double substep_s) {
  marv_plant_config c{};
  MARV_PLANT_CARD_FILL(&c);
  c.site_lat_rad = kScenarioLat;
  c.site_height_m = kScenarioHeight;
  c.motor_substep_s = substep_s;
  c.rng_seed = seed;
  return c;
}

marv_plant* make_plant(std::uint64_t seed, double substep_s) {
  const marv_plant_config c = card_config(seed, substep_s);
  marv_plant* p = nullptr;
  EXPECT_EQ(marv_plant_create(&c, &p), MARV_PLANT_OK);
  return p;
}

// The direct call, the reference: one marv_plant fed the captured body, the per-tick commands and dt.
struct Direct {
  explicit Direct(std::uint64_t seed) : plant(make_plant(seed, t_tick())) {}
  ~Direct() { marv_plant_destroy(plant); }
  Direct(const Direct&) = delete;
  Direct& operator=(const Direct&) = delete;
  marv_plant_out step(const marv_plant_body& body, const Dshot& d, double dt) {
    marv_plant_cmd cmd{};
    cmd.struct_size = sizeof(cmd);
    for (std::size_t k = 0; k < d.size(); ++k) {
      cmd.dshot[k] = d[k];
    }
    marv_plant_out out{};
    out.struct_size = sizeof(out);
    EXPECT_EQ(marv_plant_step(plant, &body, &cmd, dt, &out), MARV_PLANT_OK);
    return out;
  }
  marv_plant* plant;
};

std::uint64_t mix(std::uint64_t x) {
  Rng r{x};
  return r.next();
}

// A command that changes every tick on every motor: 0 (stop) or 48..2047, a hash of (salt, tick, motor).
Dshot script(std::uint64_t salt, std::uint64_t tick) {
  Dshot d{};
  for (std::size_t k = 0; k < d.size(); ++k) {
    const std::uint64_t v = mix((salt * 1000003ull) + (tick * 4ull) + k) % 2001ull;
    d[k] = v == 0 ? std::uint16_t{0} : static_cast<std::uint16_t>(47 + v);
  }
  return d;
}

// The exact ENU map written out here, independently of the adapter's function.
Vec3 enu_of(const double v[3]) { return {v[1], v[0], -v[2]}; }

bool outputs_bit_equal(const marv::gz::TickOutput& t, const marv_plant_out& d) {
  return t.out.erpm_valid == d.erpm_valid && same_bits(t.out.force_ned_n, d.force_ned_n) &&
         same_bits(t.out.torque_ned_nm, d.torque_ned_nm) && same_bits(t.out.rotor_speed_rad_s, d.rotor_speed_rad_s) &&
         same_bits(t.out.erpm, d.erpm) && same_bits(t.wrench_enu.force, enu_of(d.force_ned_n)) &&
         same_bits(t.wrench_enu.torque, enu_of(d.torque_ned_nm));
}

bool rotor_speeds_equal(const marv_plant_out& a, const marv_plant_out& b) {
  return same_bits(a.rotor_speed_rad_s, b.rotor_speed_rad_s);
}

marv::gz::Wrench mean_wrench(const std::vector<marv_plant_out>& outs) {
  Vec3 f = enu_of(outs[0].force_ned_n);
  Vec3 t = enu_of(outs[0].torque_ned_nm);
  for (std::size_t j = 1; j < outs.size(); ++j) {
    const Vec3 fj = enu_of(outs[j].force_ned_n);
    const Vec3 tj = enu_of(outs[j].torque_ned_nm);
    for (std::size_t c = 0; c < 3; ++c) {
      f[c] += fj[c];
      t[c] += tj[c];
    }
  }
  const double m = static_cast<double>(outs.size());
  for (std::size_t c = 0; c < 3; ++c) {
    f[c] /= m;
    t[c] /= m;
  }
  return {f, t};
}

std::vector<marv::gz::GzState> gz_states() {
  std::vector<marv::gz::GzState> v;
  Rng r{0x57A7E5};
  v.push_back({{0.0, 0.0, 0.0}, {1, 0, 0, 0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}});
  v.push_back({{1.0, 2.0, 3.0}, {0, 1, 0, 0}, {0.5, -0.5, 0.25}, {0.1, 0.2, 0.3}});
  v.push_back({{-5.0, 4.0, 250.0}, normalised({0.0, 0.3, -0.4, 0.5}), {3.0, 2.0, -1.0}, {-1.0, 2.0, -3.0}});
  for (int i = 0; i < 6; ++i) {
    const Vec3 p = random_vec(r, 6);
    v.push_back({p, random_quat(r, true), random_vec(r, 4), random_vec(r, 4)});
  }
  return v;
}

const std::array<std::uint64_t, 3> kSeeds{0, 1, 0xFEEDFACECAFEBEEFull};
const std::array<std::uint32_t, 3> kCounts{1, 3, 4};

TEST(GzAdapterTiming, TickPeriodIsTheCorrectlyRoundedRationalInSeconds) {
  EXPECT_TRUE(same_bits(t_tick(), 625.0 / 4000000.0));
  EXPECT_TRUE(same_bits(t_tick(), 1.5625e-4));
}

TEST(GzAdapterPlant, AdapterStepEqualsADirectPlantCallBitwiseOverStatesSeedsAndTickCounts) {
  std::size_t compared_ticks = 0;
  for (const marv::gz::GzState& state : gz_states()) {
    const marv_plant_body body = marv::gz::to_plant_body(state);  // the captured body
    for (std::uint64_t seed : kSeeds) {
      for (std::uint32_t m : kCounts) {
        const std::uint64_t salt = seed ^ m;
        marv::gz::ScriptedCommandSource src([salt](std::uint64_t j) { return script(salt, j); });
        marv::gz::Adapter adapter(make_plant(seed, t_tick()), src, t_tick());
        ASSERT_TRUE(same_bits(adapter.t_tick_s(), t_tick()));
        Direct direct(seed);
        std::uint64_t first = 0;
        for (int host_step = 0; host_step < 3; ++host_step) {
          const marv::gz::StepResult res = adapter.step(body, first, m);
          ASSERT_EQ(res.status, marv::gz::Status::kOk);
          ASSERT_EQ(res.ticks.size(), m);
          std::vector<marv_plant_out> outs;
          for (std::uint32_t i = 0; i < m; ++i) {
            const Dshot d = script(salt, first + i);
            EXPECT_EQ(res.ticks[i].tick, first + i);
            EXPECT_EQ(res.ticks[i].dshot, d);
            outs.push_back(direct.step(body, d, t_tick()));
            EXPECT_TRUE(outputs_bit_equal(res.ticks[i], outs.back())) << "tick " << (first + i) << " m " << m;
            ++compared_ticks;
          }
          const marv::gz::Wrench mean = mean_wrench(outs);
          EXPECT_TRUE(same_bits(res.wrench_enu.force, mean.force)) << "m " << m;
          EXPECT_TRUE(same_bits(res.wrench_enu.torque, mean.torque)) << "m " << m;
          if (m == 1) {
            EXPECT_TRUE(same_bits(res.wrench_enu.force, res.ticks[0].wrench_enu.force));
            EXPECT_TRUE(same_bits(res.wrench_enu.torque, res.ticks[0].wrench_enu.torque));
          }
          first += m;
        }
      }
    }
  }
  EXPECT_EQ(compared_ticks, gz_states().size() * kSeeds.size() * 3u * (1u + 3u + 4u));
}

TEST(GzAdapterPlant, ZeroTicksDoesNothing) {
  std::size_t calls = 0;
  marv::gz::ScriptedCommandSource src([&calls](std::uint64_t j) {
    ++calls;
    return script(0, j);
  });
  marv::gz::Adapter adapter(make_plant(0, t_tick()), src, t_tick());
  const marv_plant_body body = marv::gz::to_plant_body(gz_states()[2]);
  const marv::gz::StepResult res = adapter.step(body, 0, 0);
  EXPECT_EQ(res.status, marv::gz::Status::kZeroTicks);
  EXPECT_TRUE(res.ticks.empty());
  EXPECT_EQ(calls, 0u);
}

// ---- ordering -----------------------------------------------------------------------------------------------------

constexpr std::uint64_t kOrderSalt = 0x5EED;
constexpr std::uint32_t kOrderTicks = 8;

std::vector<marv_plant_out> direct_sequence(const marv_plant_body& body, std::uint64_t salt) {
  Direct direct(0);
  std::vector<marv_plant_out> outs;
  for (std::uint64_t j = 0; j < kOrderTicks; ++j) {
    outs.push_back(direct.step(body, script(salt, j), t_tick()));
  }
  return outs;
}

std::vector<marv::gz::TickOutput> adapter_sequence(const marv_plant_body& body, marv::gz::CommandSource& src,
                                                   std::uint32_t m) {
  marv::gz::Adapter adapter(make_plant(0, t_tick()), src, t_tick());
  std::vector<marv::gz::TickOutput> ticks;
  for (std::uint64_t first = 0; first < kOrderTicks; first += m) {
    const marv::gz::StepResult res = adapter.step(body, first, m);
    EXPECT_EQ(res.status, marv::gz::Status::kOk);
    ticks.insert(ticks.end(), res.ticks.begin(), res.ticks.end());
  }
  return ticks;
}

TEST(GzAdapterOrdering, ACommandChangingEveryTickGivesTheDirectSequenceAndAShiftedScriptDoesNot) {
  const marv_plant_body body = marv::gz::to_plant_body(gz_states()[2]);
  for (std::uint64_t j = 0; j + 1 < kOrderTicks; ++j) {
    ASSERT_NE(script(kOrderSalt, j), script(kOrderSalt, j + 1)) << "the script must change every tick";
  }
  const std::vector<marv_plant_out> direct = direct_sequence(body, kOrderSalt);

  marv::gz::ScriptedCommandSource in_order([](std::uint64_t j) { return script(kOrderSalt, j); });
  const auto good = adapter_sequence(body, in_order, 1);
  ASSERT_EQ(good.size(), direct.size());
  for (std::size_t j = 0; j < direct.size(); ++j) {
    EXPECT_TRUE(outputs_bit_equal(good[j], direct[j])) << "tick " << j;
  }

  // In-test control: the same script shifted by one tick must not match, at any tick.
  marv::gz::ScriptedCommandSource shifted([](std::uint64_t j) { return script(kOrderSalt, j + 1); });
  const auto bad = adapter_sequence(body, shifted, 1);
  ASSERT_EQ(bad.size(), direct.size());
  std::size_t matching = 0;
  for (std::size_t j = 0; j < direct.size(); ++j) {
    matching += rotor_speeds_equal(bad[j].out, direct[j]) ? 1u : 0u;
  }
  EXPECT_EQ(matching, 0u);
}

TEST(GzAdapterOrdering, PerTickOutputsAreTheSameForEveryTicksPerHostStep) {
  const marv_plant_body body = marv::gz::to_plant_body(gz_states()[2]);
  std::vector<std::vector<marv::gz::TickOutput>> runs;
  for (std::uint32_t m : {1u, 2u, 4u}) {
    marv::gz::ScriptedCommandSource src([](std::uint64_t j) { return script(kOrderSalt, j); });
    runs.push_back(adapter_sequence(body, src, m));
  }
  for (std::size_t run = 1; run < runs.size(); ++run) {
    ASSERT_EQ(runs[run].size(), runs[0].size());
    for (std::size_t j = 0; j < runs[0].size(); ++j) {
      EXPECT_TRUE(same_bits(runs[run][j].out, runs[0][j].out)) << "run " << run << " tick " << j;
      EXPECT_TRUE(same_bits(runs[run][j].wrench_enu, runs[0][j].wrench_enu)) << "run " << run << " tick " << j;
    }
  }
}

// ---- negative controls of the plant claims ------------------------------------------------------------------------

// One DShot step on one motor at tick 0 changes that motor's rotor speed and the wrench, and no other motor's speed.
TEST(GzAdapterOrdering, NegativeControlOneDshotStepOnOneMotorMismatches) {
  const marv_plant_body body = marv::gz::to_plant_body(gz_states()[2]);
  const std::vector<marv_plant_out> direct = direct_sequence(body, kOrderSalt);
  for (std::size_t motor = 0; motor < MARV_PLANT_N_MOTORS; ++motor) {
    marv::gz::ScriptedCommandSource src([motor](std::uint64_t j) {
      Dshot d = script(kOrderSalt, j);
      if (j == 0) {
        d[motor] = d[motor] == 0 ? std::uint16_t{48} : (d[motor] < 2047 ? static_cast<std::uint16_t>(d[motor] + 1)
                                                                         : std::uint16_t{2046});
      }
      return d;
    });
    const auto ticks = adapter_sequence(body, src, 1);
    EXPECT_FALSE(same_bits(ticks[0].out.rotor_speed_rad_s[motor], direct[0].rotor_speed_rad_s[motor]));
    EXPECT_FALSE(outputs_bit_equal(ticks[0], direct[0]));
    for (std::size_t other = 0; other < MARV_PLANT_N_MOTORS; ++other) {
      if (other != motor) {
        EXPECT_TRUE(same_bits(ticks[0].out.rotor_speed_rad_s[other], direct[0].rotor_speed_rad_s[other]));
      }
    }
  }
}

// A wrong dt. The plant is continuous in dt: with h = t_tick, a dt above h adds a partial sub-step whose decay
// exp(-(dt - h)/tau) is 1 to the last bit until (dt - h)/tau reaches u, and a dt below h is one partial sub-step with
// the same rounding, so a relative change of one ulp (2^-52) of dt changes no output bit (shown by the test
// AOneUlpDtIsBelowThePlantsResolution below; up to about 2^-47 was also observed while writing this test). The control is
// therefore the smallest relative change that the outputs resolve with margin, 2^-44 of dt (a few hundred ulp); the
// dt value itself is compared bitwise in the timing test and in the plant-equality test above.
constexpr double kDtControlRelative = 0x1p-44;

TEST(GzAdapterOrdering, NegativeControlAWrongDtMismatchesEveryTick) {
  const marv_plant_body body = marv::gz::to_plant_body(gz_states()[2]);
  const double wrong_dt = t_tick() * (1.0 + kDtControlRelative);
  EXPECT_FALSE(same_bits(wrong_dt, t_tick()));
  EXPECT_FALSE(same_bits(std::nextafter(t_tick(), 1.0), t_tick()));
  Direct direct(0);
  marv::gz::ScriptedCommandSource src([](std::uint64_t j) { return script(kOrderSalt, j); });
  const auto ticks = adapter_sequence(body, src, 1);
  std::size_t matching = 0;
  for (std::uint64_t j = 0; j < kOrderTicks; ++j) {
    const marv_plant_out out = direct.step(body, script(kOrderSalt, j), wrong_dt);
    matching += rotor_speeds_equal(ticks[j].out, out) ? 1u : 0u;
  }
  EXPECT_EQ(matching, 0u);
}

// The committed evidence for the comment above: a dt one ulp above t_tick leaves every rotor speed bit unchanged, which
// is why the wrong-dt control cannot be one ulp.
TEST(GzAdapterOrdering, AOneUlpDtIsBelowThePlantsResolution) {
  const marv_plant_body body = marv::gz::to_plant_body(gz_states()[2]);
  const double ulp_dt = std::nextafter(t_tick(), 1.0);
  Direct direct(0);
  marv::gz::ScriptedCommandSource src([](std::uint64_t j) { return script(kOrderSalt, j); });
  const auto ticks = adapter_sequence(body, src, 1);
  std::size_t matching = 0;
  for (std::uint64_t j = 0; j < kOrderTicks; ++j) {
    const marv_plant_out out = direct.step(body, script(kOrderSalt, j), ulp_dt);
    matching += rotor_speeds_equal(ticks[j].out, out) ? 1u : 0u;
  }
  EXPECT_EQ(matching, kOrderTicks);
}

// ---- the real SIL -------------------------------------------------------------------------------------------------

bool report_failures_on_stderr() {
  const ::testing::TestResult* r = ::testing::UnitTest::GetInstance()->current_test_info()->result();
  bool any = false;
  for (int i = 0; i < r->total_part_count(); ++i) {
    const ::testing::TestPartResult& part = r->GetTestPartResult(i);
    if (part.failed()) {
      any = true;
      std::fprintf(stderr, "%s:%d: %s\n", part.file_name(), part.line_number(), part.message());
    }
  }
  return any;
}

// marv_sil_init succeeds once per process, so the case runs in a forked child (as open_loop_test.cpp).
template <class Body>
void in_child(Body body) {
  EXPECT_EXIT(
      {
        body();
        std::exit(report_failures_on_stderr() ? 1 : 0);
      },
      ::testing::ExitedWithCode(0), "");
}

std::uint64_t sil_stamp(std::uint64_t n) {
  return static_cast<std::uint64_t>(static_cast<unsigned __int128>(n) * kPeriodNumUs / kPeriodDen);
}

// marv_sil_l2_open_loop with the DShot set by override (logical order), stepped by the adapter through
// SilCommandSource: the values reach the plant, bit-equal to a direct plant call given the same commands.
TEST(GzAdapterSil, TheOverriddenDshotReachesThePlantThroughTheSilCommandSource) {
  in_child([] {
    const Dshot want{700, 0, 1500, 2047};
    const marv::ParamId ids[4] = {marv::ParamId::ol_dshot_m1, marv::ParamId::ol_dshot_m2, marv::ParamId::ol_dshot_m3,
                                  marv::ParamId::ol_dshot_m4};
    marv_sil_param_override ov[4] = {};
    for (std::size_t k = 0; k < 4; ++k) {
      ov[k].id = static_cast<std::uint32_t>(ids[k]);
      ov[k].type = MARV_PARAM_I32;
      ov[k].i32 = want[k];
    }
    marv_sil_config c{};
    c.struct_size = sizeof(marv_sil_config);
    c.imu_meas_size = sizeof(marv_imu_meas);
    c.override_size = sizeof(marv_sil_param_override);
    c.tick_period_num_us = kPeriodNumUs;
    c.tick_period_den = kPeriodDen;
    c.n_overrides = 4;
    c.overrides = ov;
    c.param_schema_hash = marv::kParamSchemaHash;
    ASSERT_EQ(marv_sil_init(&c), MARV_SIL_OK);

    const marv_plant_body body = marv::gz::to_plant_body(gz_states()[2]);
    marv::gz::SilCommandSource sil;
    marv::gz::Adapter adapter(make_plant(0, t_tick()), sil, t_tick());
    Direct direct(0);
    std::uint64_t first = 0;
    for (std::uint32_t m : {3u, 2u}) {
      const marv::gz::StepResult res = adapter.step(body, first, m);
      ASSERT_EQ(res.status, marv::gz::Status::kOk);
      ASSERT_EQ(sil.last_status(), MARV_SIL_OK);
      EXPECT_EQ(sil.last_stamp_us(), sil_stamp(first + m - 1));
      std::vector<marv_plant_out> outs;
      for (std::uint32_t i = 0; i < m; ++i) {
        EXPECT_EQ(res.ticks[i].dshot, want);
        outs.push_back(direct.step(body, want, t_tick()));
        EXPECT_TRUE(outputs_bit_equal(res.ticks[i], outs.back()));
      }
      const marv::gz::Wrench mean = mean_wrench(outs);
      EXPECT_TRUE(same_bits(res.wrench_enu.force, mean.force));
      EXPECT_TRUE(same_bits(res.wrench_enu.torque, mean.torque));
      const marv_plant_out& last = res.ticks.back().out;
      EXPECT_EQ(last.rotor_speed_rad_s[1], 0.0);  // DShot 0 (stop) stays stopped
      EXPECT_GT(last.rotor_speed_rad_s[0], 0.0);
      EXPECT_LT(last.rotor_speed_rad_s[0], last.rotor_speed_rad_s[2]);
      EXPECT_LT(last.rotor_speed_rad_s[2], last.rotor_speed_rad_s[3]);
      EXPECT_EQ(last.erpm_valid, 1u);
      first += m;
    }
  });
}

}  // namespace
