// L6 stage (a), T1 (decision 0012): the moments of the standard normals of the stream, mean, variance, skewness and
// excess kurtosis, are within bounds derived from N at the budget's confidence (normal_moments_confidence, read from
// design/budget.yaml at configure time). Negative controls: a variant with sigma x 1.1 and a uniform variant of unit
// variance must be rejected by the same check.
//
// Bound. The four statistics are one family at confidence c, so each is tested at the Bonferroni level
// alpha = (1 - c)/4 (two-sided), z = the AS241 point of alpha/2. Against the target N(0, 1) the large-N standard
// errors are (Fisher 1930; Kendall and Stuart, The Advanced Theory of Statistics, vol. 1): mean 1/sqrt(N), variance
// sqrt(2/N), skewness sqrt(6/N), excess kurtosis sqrt(24/N); the corrections are O(1/N) relative and N is large.
#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

#include "budget_register.hpp"
#include "noise/noise.hpp"
#include "support.hpp"

namespace {

namespace noise = marv::plant::noise;
using marv::l06::Moments;

// Labelled test value: N = 2^20 normals. Large enough that the bounds are tight (about 0.3 % on the variance, below
// the 10 % scale of the x 1.1 control and the 1.2 excess kurtosis of the uniform control) and the sampling
// distributions of the moments are normal to well within the bound's own resolution; cheap enough for debug builds.
constexpr std::uint64_t kN = 1U << 20U;
// Labelled test value: the one fixed seed of the check (a pass is a property of this stream, as recorded).
constexpr std::uint64_t kSeed = 1;
// The number of statistics in the family: mean, variance, skewness, excess kurtosis.
constexpr std::size_t kStatistics = 4;
// Labelled control values: the scale of the biased variant (the 10 % that the Allan controls also use).
constexpr double kControlScale = 1.1;

struct Verdict {
  bool pass = false;
  Moments m;
  double z = 0.0;
};

Verdict check_moments(const std::vector<double>& x) {
  const auto n = static_cast<double>(x.size());
  Verdict v;
  v.m = marv::l06::sample_moments(x);
  v.z = marv::l06::z_two_sided(marv::l06::bonferroni_alpha(marv::l06::kNormalMomentsConfidence, kStatistics));
  v.pass = std::fabs(v.m.mean) <= v.z / std::sqrt(n) && std::fabs(v.m.variance - 1.0) <= v.z * std::sqrt(2.0 / n) &&
           std::fabs(v.m.skewness) <= v.z * std::sqrt(6.0 / n) &&
           std::fabs(v.m.excess_kurtosis) <= v.z * std::sqrt(24.0 / n);
  std::cout << "[MOMENTS] N " << x.size() << " z " << v.z << " mean " << v.m.mean << " (bound " << v.z / std::sqrt(n)
            << ") variance " << v.m.variance << " (bound " << v.z * std::sqrt(2.0 / n) << ") skewness " << v.m.skewness
            << " (bound " << v.z * std::sqrt(6.0 / n) << ") excess kurtosis " << v.m.excess_kurtosis << " (bound "
            << v.z * std::sqrt(24.0 / n) << ")\n";
  return v;
}

std::vector<double> stream_normals() {
  std::vector<double> x(kN);
  for (std::uint64_t k = 0; k < kN; ++k) {
    x[k] = noise::normal(kSeed, noise::kStreamPrimaryImu, k);
  }
  return x;
}

TEST(L06NoiseMoments, NormalsHaveTheStandardNormalMoments) {
  const Verdict v = check_moments(stream_normals());
  EXPECT_TRUE(v.pass);
}

TEST(L06NoiseMomentsControl, ScaleBiasedVariantIsRejected) {
  std::vector<double> x = stream_normals();
  for (double& v : x) {
    v *= kControlScale;
  }
  EXPECT_FALSE(check_moments(x).pass);
}

TEST(L06NoiseMomentsControl, UniformVariantOfUnitVarianceIsRejected) {
  std::vector<double> x(kN);
  // Uniform on [-sqrt(3), sqrt(3)): mean 0 and variance 1, so only the shape (skewness 0, excess kurtosis -1.2 instead
  // of 0) distinguishes it from the standard normal.
  const double half_width = std::sqrt(3.0);
  for (std::uint64_t k = 0; k < kN; ++k) {
    x[k] = (2.0 * noise::uniform_closed_open(noise::draw(kSeed, noise::kStreamPrimaryImu, k)) - 1.0) * half_width;
  }
  const Verdict v = check_moments(x);
  EXPECT_FALSE(v.pass);
  EXPECT_LT(std::fabs(v.m.variance - 1.0), v.z * std::sqrt(2.0 / static_cast<double>(kN)));
}

// The critical value arithmetic: AS241 against the six-decimal standard normal percentage points (two-sided
// 0.05 -> 1.959964, 0.01 -> 2.575829, 0.001 -> 3.290527; upper 1e-5 -> 4.264891; tabulated e.g. in Abramowitz and
// Stegun, Handbook of Mathematical Functions, section 26.2: the figures here are the standard ones, the book was not
// consulted for this commit), and a round trip through erfc at the tail levels the independence check uses.
TEST(L06NoiseMoments, InverseNormalMatchesPublishedPercentagePoints) {
  constexpr double kTableTolerance = 5e-7;  // the tables give 6 decimals
  EXPECT_NEAR(marv::l06::z_two_sided(0.05), 1.959964, kTableTolerance);
  EXPECT_NEAR(marv::l06::z_two_sided(0.01), 2.575829, kTableTolerance);
  EXPECT_NEAR(marv::l06::z_two_sided(0.001), 3.290527, kTableTolerance);
  EXPECT_NEAR(-marv::l06::ppnd16(1e-5), 4.264891, kTableTolerance);
  // Labelled test tolerance: 1e-15, about 4.5 times the double epsilon 2^-52 = 2.2e-16. ppnd16(0.5) has q = 0 and so
  // is 0 to rounding; a wrong central branch would miss it by far more.
  EXPECT_NEAR(marv::l06::ppnd16(0.5), 0.0, 1e-15);
  // Labelled test values: two-sided levels from the middle of the range to the far tail, one per regime of the
  // algorithm: 1e-2 (central/mid branch edge), 1e-4 and 1e-8 (mid branch), 3e-6 (about the per-test level of the
  // independence check: 0.01 over its 68 pairs x 49 statistics = 3332, 3.0e-6) and 1e-12 (the far-tail branch, whose
  // split r = 5 is at p = e^-25 = 1.4e-11).
  // Labelled test tolerance: the round-trip relative error is the accuracy of AS241 (about 1e-16) times the
  // conditioning of alpha in z (about z^2, 49 at alpha = 1e-12), plus erfc's few ulp, so about 5e-15 at worst; 1e-12 is
  // about 200 times that and still far below the error of a wrong tail.
  for (const double alpha : {1e-2, 1e-4, 3e-6, 1e-8, 1e-12}) {
    const double z = marv::l06::z_two_sided(alpha);
    EXPECT_NEAR(std::erfc(z / std::sqrt(2.0)) / alpha, 1.0, 1e-12) << alpha;
  }
}

}  // namespace
