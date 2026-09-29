// Frozen path: tests/regression/quad/L01/unit/prim/gravity_test.cpp
// T1 unit tests of marv/prim/gravity.hpp: WGS 84 normal gravity, NIMA TR8350.2 3rd ed. Amdt 1 (2000) eq. (4-1), (4-3).
//
// Tolerance rule (no tolerance is chosen by feel). Every tolerance is relative to the value checked and is
// n * epsilon(T) plus, where stated, a documented bound on a known input difference.
//  - n counts rounded floating-point operations on the path from inputs to the result. Each rounding contributes a
//    relative error <= epsilon/2 and library sin/sqrt at most 1 ulp (= epsilon), so every operation is charged
//    epsilon. The path is: sin, square, 1 + k s2, e2 s2, 1 - e2 s2, sqrt, divide, multiply by gamma_e (9 operations
//    for gamma); 1/a, 2 inv_a, f + m..., 2 f s2, the sum, the products with h and the gamma multiply (about 20 more
//    for gamma_h); plus conversion of the six constants to T (epsilon/2 each). Every factor is well conditioned
//    (1 - e2 s2 in [0.993, 1], the height bracket in [0.99, 1] for h <= 50 km), so the errors add to at most
//    ~35 epsilon. kOps = 40 covers that with a margin of a few operations; it is the same rule as kOpsChain in the
//    L00 prim tests, with the count restated for this path.
//  - The golden vectors come from the toolbox, whose e^2 is 0.00669437999013 where TR8350.2 Table 3.3 gives
//    6.69437999014e-3 (delta_e2 = 1e-14). The effect on gamma is d gamma / gamma = s2 delta_e2 / (2 (1 - e2 s2)) <=
//    delta_e2 / (2 (1 - e2)), and it is added to the double golden tolerance.
//  - The known-answer gamma_p is tabulated to 10 decimals (Table 3.4), so its tolerance is one unit of the last
//    tabulated decimal, 1e-10 m/s^2 (covers rounding or truncation of the table), plus the roundoff bound.

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "marv/prim/gravity.hpp"

namespace marv::prim::test {
namespace {

constexpr int kOps = 40;

// TR8350.2 values restated here, independent of constants.hpp, for the independent long double evaluation.
constexpr long double kRefA = 6378137.0L;
constexpr long double kRefF = 1.0L / 298.257223563L;
constexpr long double kRefE2 = 6.69437999014e-3L;
constexpr long double kRefGammaE = 9.7803253359L;
constexpr long double kRefK = 0.00193185265241L;
constexpr long double kRefM = 0.00344978650684L;
// Table 3.4 gamma_p and its last decimal.
constexpr double kTableGammaP = 9.8321849378;
constexpr double kTableUnit = 1e-10;
// Toolbox e^2 (src/lib/physics/constants.ts) against Table 3.3.
constexpr double kToolboxE2 = 0.00669437999013;
constexpr double kPi = 3.14159265358979323846;

// Independent evaluation of eq. (4-1) and (4-3) in long double.
long double ref_gamma_h(long double phi, long double h) {
  const long double s2 = std::sin(phi) * std::sin(phi);
  const long double gamma = kRefGammaE * (1 + kRefK * s2) / std::sqrt(1 - kRefE2 * s2);
  return gamma * (1 - (2 / kRefA) * (1 + kRefF + kRefM - 2 * kRefF * s2) * h + (3 / (kRefA * kRefA)) * h * h);
}

long double ref_gamma(long double phi) { return ref_gamma_h(phi, 0); }

template <class T>
T rel_tol() {
  return static_cast<T>(kOps) * std::numeric_limits<T>::epsilon();
}

struct Vector {
  double phi;
  double h;
  double gamma;
  double gamma_h;
};

std::vector<Vector> load_golden() {
  std::vector<Vector> rows;
  std::ifstream in(MARV_GRAVITY_GOLDEN_CSV);
  std::string line;
  bool header = false;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    if (!header) {
      header = true;
      continue;
    }
    std::stringstream ss(line);
    std::string cell;
    std::vector<double> v;
    while (std::getline(ss, cell, ',')) {
      v.push_back(std::strtod(cell.c_str(), nullptr));
    }
    if (v.size() == 5) {
      rows.push_back({v[1], v[2], v[3], v[4]});
    }
  }
  return rows;
}

// Grid stated in tools/golden/gravity_vectors.mjs: 13 latitudes x 6 heights.
constexpr std::size_t kGoldenRows = 13 * 6;

// Relative tolerance of the double comparison against the toolbox vectors (see the rule above).
double golden_rel_tol() {
  const double e2_effect = std::abs(6.69437999014e-3 - kToolboxE2) / (2.0 * (1.0 - 6.69437999014e-3));
  return rel_tol<double>() + e2_effect;
}

// Test-only variant of eq. (4-1)/(4-3) used only for negative controls: gamma_e scaled, optionally without the h^2
// term. Same operation order as the code under test.
double variant_gamma_h(double phi, double h, double gamma_e_scale, bool drop_quad) {
  const double s2 = std::sin(phi) * std::sin(phi);
  const double gamma = gamma_e_scale * static_cast<double>(kRefGammaE) * (1 + static_cast<double>(kRefK) * s2) /
                       std::sqrt(1 - static_cast<double>(kRefE2) * s2);
  const double a = static_cast<double>(kRefA);
  const double f = static_cast<double>(kRefF);
  const double m = static_cast<double>(kRefM);
  const double quad = drop_quad ? 0.0 : (3 / (a * a)) * h * h;
  return gamma * (1 - (2 / a) * (1 + f + m - 2 * f * s2) * h + quad);
}

bool within(double got, double want, double rel) { return std::abs(got - want) <= rel * std::abs(want); }

TEST(Gravity, KnownAnswersEquatorAndPole) {
  // sin(0) == 0 gives gamma_e exactly; sin(pi/2) == 1 in double.
  EXPECT_NEAR(normal_gravity_ellipsoid(0.0), 9.7803253359, rel_tol<double>() * 9.7803253359);
  EXPECT_NEAR(normal_gravity_ellipsoid(kPi / 2.0), kTableGammaP, kTableUnit + rel_tol<double>() * kTableGammaP);
  EXPECT_NEAR(normal_gravity(0.0, 0.0), 9.7803253359, rel_tol<double>() * 9.7803253359);
}

TEST(Gravity, MatchesIndependentLongDoubleEvaluation) {
  for (int lat = -90; lat <= 90; lat += 5) {
    const double phi = static_cast<double>(lat) * kPi / 180.0;
    for (double h : {0.0, 1.0, 100.0, 1000.0, 10000.0, 50000.0}) {
      const long double want = ref_gamma_h(static_cast<long double>(phi), static_cast<long double>(h));
      const double got = normal_gravity(phi, h);
      EXPECT_TRUE(within(got, static_cast<double>(want), rel_tol<double>())) << "lat " << lat << " h " << h;
    }
    EXPECT_TRUE(within(normal_gravity_ellipsoid(phi), static_cast<double>(ref_gamma(static_cast<long double>(phi))),
                       rel_tol<double>()))
        << "lat " << lat;
  }
}

TEST(Gravity, GoldenVectorsFromToolbox) {
  const std::vector<Vector> rows = load_golden();
  ASSERT_EQ(rows.size(), kGoldenRows) << "golden file missing or incomplete: " << MARV_GRAVITY_GOLDEN_CSV;
  const double tolerance = golden_rel_tol();
  for (const Vector& r : rows) {
    EXPECT_TRUE(within(normal_gravity_ellipsoid(r.phi), r.gamma, tolerance)) << "phi " << r.phi;
    EXPECT_TRUE(within(normal_gravity(r.phi, r.h), r.gamma_h, tolerance)) << "phi " << r.phi << " h " << r.h;
  }
}

// Negative controls: the golden comparison must have resolving power. The unperturbed variant passes the same
// comparison; a gamma_e scaled by 1 + 1e-9 (relative shift 1e-9, about 7e4 times the tolerance) fails at every
// vector; dropping the h^2 term fails at every vector with h >= 100 m (its relative size 3 (h/a)^2 = 7e-13 there).
TEST(Gravity, NegativeControlPerturbedConstantBreaksGoldenComparison) {
  const std::vector<Vector> rows = load_golden();
  ASSERT_EQ(rows.size(), kGoldenRows);
  const double tolerance = golden_rel_tol();
  for (const Vector& r : rows) {
    EXPECT_TRUE(within(variant_gamma_h(r.phi, r.h, 1.0, false), r.gamma_h, tolerance))
        << "unperturbed variant must pass: phi " << r.phi << " h " << r.h;
    EXPECT_FALSE(within(variant_gamma_h(r.phi, r.h, 1.0 + 1e-9, false), r.gamma_h, tolerance))
        << "perturbed gamma_e must fail: phi " << r.phi << " h " << r.h;
    if (r.h >= 100.0) {
      EXPECT_FALSE(within(variant_gamma_h(r.phi, r.h, 1.0, true), r.gamma_h, tolerance))
          << "dropped h^2 term must fail: phi " << r.phi << " h " << r.h;
    }
  }
}

TEST(Gravity, HeightZeroEqualsEllipsoidAndGravityDecreasesWithHeight) {
  for (int lat = -90; lat <= 90; lat += 15) {
    const double phi = static_cast<double>(lat) * kPi / 180.0;
    EXPECT_EQ(normal_gravity(phi, 0.0), normal_gravity_ellipsoid(phi)) << "lat " << lat;
    double previous = normal_gravity(phi, 0.0);
    for (double h = 1000.0; h <= 50000.0; h += 1000.0) {
      const double g = normal_gravity(phi, h);
      EXPECT_LT(g, previous) << "lat " << lat << " h " << h;
      previous = g;
    }
  }
}

// float is the flight instantiation. It is compared with the double instantiation evaluated at the float-rounded
// inputs, so the tolerance covers only float roundoff (kOps * epsilon(float), relative), not input rounding.
TEST(Gravity, FloatInstantiationMatchesDouble) {
  const std::vector<Vector> rows = load_golden();
  ASSERT_EQ(rows.size(), kGoldenRows);
  for (const Vector& r : rows) {
    const float phi = static_cast<float>(r.phi);
    const float h = static_cast<float>(r.h);
    const double want_h = normal_gravity(static_cast<double>(phi), static_cast<double>(h));
    const double want_0 = normal_gravity_ellipsoid(static_cast<double>(phi));
    EXPECT_TRUE(within(static_cast<double>(normal_gravity(phi, h)), want_h, static_cast<double>(rel_tol<float>())))
        << "phi " << r.phi << " h " << r.h;
    EXPECT_TRUE(within(static_cast<double>(normal_gravity_ellipsoid(phi)), want_0,
                       static_cast<double>(rel_tol<float>())))
        << "phi " << r.phi;
  }
}

TEST(Gravity, FloatHeightZeroEqualsEllipsoidAndDecreasesWithHeight) {
  for (int lat = -90; lat <= 90; lat += 15) {
    const float phi = static_cast<float>(lat) * static_cast<float>(kPi) / 180.0F;
    EXPECT_EQ(normal_gravity(phi, 0.0F), normal_gravity_ellipsoid(phi)) << "lat " << lat;
    // Height steps of 1 km change gamma by 3e-3 m/s^2, about 2.5e3 float epsilons of gamma, so decrease is resolved.
    float previous = normal_gravity(phi, 0.0F);
    for (float h = 1000.0F; h <= 50000.0F; h += 1000.0F) {
      const float g = normal_gravity(phi, h);
      EXPECT_LT(g, previous) << "lat " << lat << " h " << h;
      previous = g;
    }
  }
}

}  // namespace
}  // namespace marv::prim::test
