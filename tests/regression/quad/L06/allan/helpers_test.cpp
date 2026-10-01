// L6 stage (a) T1 Allan check, the unit checks of its helpers (allan.hpp): the chi-square inverse against published table
// values and closed forms, the equivalent degrees of freedom against hand-evaluated Table 5 values, the streaming
// overlapping Allan variance against hand-computed and brute-force series, the closed-form expected variance against
// the exact weights and against a simulation (a labelled cheap setting), and the record-length rule's own properties.
// Each metric check has a control that must break it.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <random>
#include <utility>
#include <vector>

#include "allan.hpp"
#include "allan_register.hpp"

namespace {

using namespace marv::l06::allan;

TEST(AllanHelpers, ChiSquareQuantilesMatchPublishedTableValues) {
  // NIST/SEMATECH e-Handbook of Statistical Methods, section 1.3.6.7.4, "Critical Values of the Chi-Square
  // Distribution" (upper-tail probability, degrees of freedom, critical value, 3 decimals). The tolerance is the table's
  // rounding, 5e-4.
  constexpr double kTableRounding = 5.0e-4;
  struct Row {
    double nu;
    double upper_tail;
    double value;
  };
  const Row rows[] = {{10.0, 0.05, 18.307}, {10.0, 0.95, 3.940},  {1.0, 0.01, 6.635},    {20.0, 0.01, 37.566},
                      {20.0, 0.99, 8.260},  {100.0, 0.005, 140.169}, {100.0, 0.995, 67.328}};
  for (const Row& r : rows) {
    EXPECT_NEAR(chi2_quantile(r.upper_tail, r.nu, true), r.value, kTableRounding) << r.nu << " " << r.upper_tail;
    EXPECT_NEAR(chi2_quantile(1.0 - r.upper_tail, r.nu, false), r.value, kTableRounding) << r.nu << " " << r.upper_tail;
  }
  // Control: a table value off by one unit in the last printed place is not within the rounding.
  EXPECT_GT(std::fabs(chi2_quantile(0.05, 10.0, true) - 18.308), kTableRounding / 2.0);
  EXPECT_GT(std::fabs(chi2_quantile(0.05, 10.0, true) - 18.306), kTableRounding / 2.0);
}

TEST(AllanHelpers, ChiSquareQuantilesMatchClosedForms) {
  constexpr double kClosedFormRelative = 1.0e-8;  // labelled: far inside the inverse's own tolerance
  // nu = 2: the survival function is exp(-x / 2), x = -2 ln(q); down to the tails used by the check.
  for (const double q : {0.5, 1.0e-3, 4.0e-4, 1.0e-9}) {
    EXPECT_NEAR(chi2_quantile(q, 2.0, true), -2.0 * std::log(q), kClosedFormRelative * -2.0 * std::log(q)) << q;
    EXPECT_NEAR(chi2_quantile(q, 2.0, false), -2.0 * std::log1p(-q), 1.0e-6 * q) << q;
  }
  // nu = 4: the CDF is 1 - exp(-x / 2)(1 + x / 2); the inverse must return an x that reproduces p.
  for (const double p : {0.01, 0.5, 0.99}) {
    const double x = chi2_quantile(p, 4.0, false);
    EXPECT_NEAR(1.0 - std::exp(-0.5 * x) * (1.0 + 0.5 * x), p, 1.0e-9) << p;
  }
  // nu = 1: the CDF is erf(sqrt(x / 2)).
  for (const double p : {0.01, 0.5, 0.99}) {
    const double x = chi2_quantile(p, 1.0, false);
    EXPECT_NEAR(std::erf(std::sqrt(0.5 * x)), p, 1.0e-9) << p;
  }
}

TEST(AllanHelpers, ChiSquareQuantileAtLargeDegreesOfFreedomMatchesCornishFisher) {
  // Independent of the incomplete gamma code: the Cornish-Fisher expansion chi2_p(nu) = nu + z sqrt(2 nu) +
  // (2 / 3)(z^2 - 1) + O(nu^(-1/2)) (Abramowitz & Stegun 26.4.16-17). At nu = 1e6 the neglected term is about 0.002; the
  // tolerance 0.05 is labelled. This is the regime of the check (nu up to about 1e7).
  constexpr double kNu = 1.0e6;
  constexpr double kTolerance = 0.05;
  for (const double q : {4.0e-4, 0.5, 0.99}) {
    const double z = normal_upper_quantile(q);
    const double cf = kNu + z * std::sqrt(2.0 * kNu) + (2.0 / 3.0) * (z * z - 1.0);
    EXPECT_NEAR(chi2_quantile(q, kNu, true), cf, kTolerance) << q;
  }
  // Control: the expansion without its second-order term is farther than the tolerance (the check discriminates).
  const double z = normal_upper_quantile(4.0e-4);
  EXPECT_GT(std::fabs(chi2_quantile(4.0e-4, kNu, true) - (kNu + z * std::sqrt(2.0 * kNu))), kTolerance);
}

TEST(AllanHelpers, EquivalentDegreesOfFreedomMatchHandEvaluatedTable5) {
  // NIST SP 1065 section 5.4.1 Table 5 evaluated by hand (arbitrary-precision arithmetic, not by allan.hpp) at four
  // (N, m). The document gives no worked edf values.
  constexpr double kRelative = 1.0e-12;
  struct Row {
    double n;
    double m;
    double w_fm;
    double rw_fm;
  };
  const Row rows[] = {{101.0, 1.0, 65.7953795379538, 100.03082049146188},
                      {101.0, 10.0, 12.878621195452878, 7.628071636817993},
                      {1001.0, 10.0, 146.17678617678618, 97.33189826546882},
                      {1001.0, 100.0, 13.002370707657546, 7.4222593483560315}};
  for (const Row& r : rows) {
    EXPECT_NEAR(edf_white_fm(r.n, r.m), r.w_fm, kRelative * r.w_fm);
    EXPECT_NEAR(edf_random_walk_fm(r.n, r.m), r.rw_fm, kRelative * r.rw_fm);
    EXPECT_NEAR(edf(static_cast<std::size_t>(r.n) - 1, static_cast<std::size_t>(r.m)), std::min(r.w_fm, r.rw_fm),
                kRelative * r.w_fm);
  }
  // Control: a wrong constant (the 4 m^2 + 5 term read as 4 m^2 + 4) is seen.
  const double wrong = (3.0 * 100.0 / 2.0 - 2.0 * 99.0 / 101.0) * 4.0 / 8.0;
  EXPECT_GT(std::fabs(wrong - rows[0].w_fm), 1.0e-3);
  // Limits: W FM at m = 1 tends to 2 N / 3; RW FM tends to N / m.
  EXPECT_NEAR(edf_white_fm(1.0e7, 1.0) / 1.0e7, 2.0 / 3.0, 1.0e-6);
  EXPECT_NEAR(edf_random_walk_fm(1.0e7, 100.0) / (1.0e7 / 100.0), 1.0, 1.0e-4);
}

TEST(AllanHelpers, StreamingAllanMatchesAHandComputedSeries) {
  // Counts 0, 1, 0, 2, 1, 3 (n = 6, N = 7 phase points). Phase sums S = 0, 0, 1, 1, 3, 4, 7.
  //  m = 1: second differences S_{i+2} - 2 S_{i+1} + S_i = 1, -1, 2, -1, 2; squares sum 11; N - 2m = 5 terms:
  //         11 / (2 * 5) = 1.1.
  //  m = 2: S_{i+4} - 2 S_{i+2} + S_i for i = 0..2 = 1, 2, 2; squares 1, 4, 4; divided by m^2 = 4: 2.25; 3 terms:
  //         2.25 / (2 * 3) = 0.375.
  const std::int64_t counts[] = {0, 1, 0, 2, 1, 3};
  OverlappingAllan a1(1);
  OverlappingAllan a2(2);
  for (const std::int64_t c : counts) {
    a1.push(c);
    a2.push(c);
  }
  EXPECT_EQ(a1.terms(), 5U);
  EXPECT_EQ(a2.terms(), 3U);
  EXPECT_DOUBLE_EQ(a1.variance(1.0), 1.1);
  EXPECT_DOUBLE_EQ(a2.variance(1.0), 0.375);
  EXPECT_DOUBLE_EQ(a1.variance(0.5), 1.1 * 0.25);  // the lsb scales the variance by lsb^2
  // Control: a record shorter than 2m + 1 phase points has no variance, and the hand value is not the m = 2 one at m = 1.
  OverlappingAllan short_record(2);
  for (int i = 0; i < 3; ++i) {
    short_record.push(i);
  }
  EXPECT_TRUE(std::isnan(short_record.variance(1.0)));
  EXPECT_NE(a1.variance(1.0), a2.variance(1.0));
}

TEST(AllanHelpers, StreamingAllanMatchesTheDirectOverlappingFormula) {
  // The window-sum form of NIST SP 1065 eq. 10 (block means of the rate, O(n m)) against the ring of phase sums, on a
  // random integer series (fixed-seed Mersenne twister; labelled length 5000 and values within +-1000).
  std::mt19937_64 rng(1);
  std::vector<std::int64_t> c(5000);
  for (std::int64_t& v : c) {
    v = static_cast<std::int64_t>(rng() % 2001) - 1000;
  }
  for (const std::size_t m : {std::size_t{1}, std::size_t{3}, std::size_t{7}, std::size_t{250}}) {
    OverlappingAllan est(m);
    for (const std::int64_t v : c) {
      est.push(v);
    }
    const std::size_t big_m = c.size();
    double sum = 0.0;
    for (std::size_t j = 0; j + 2 * m <= big_m; ++j) {
      double mean_a = 0.0;
      double mean_b = 0.0;
      for (std::size_t i = 0; i < m; ++i) {
        mean_a += static_cast<double>(c[j + i]);
        mean_b += static_cast<double>(c[j + m + i]);
      }
      const double d = (mean_b - mean_a) / static_cast<double>(m);
      sum += d * d;
    }
    const double direct = sum / (2.0 * static_cast<double>(big_m - 2 * m + 1));
    EXPECT_NEAR(est.variance(1.0), direct, 1.0e-9 * direct) << m;
  }
}

TEST(AllanHelpers, ExpectedVarianceMatchesTheExactWeightsAndTheAllanMinimum) {
  constexpr double kTau0 = 0.015625;  // labelled: 1/64 s, so that tau* = 1 s is m* = 64
  const Axis a{1.0, 1.0, 0.0};        // labelled: N = B = 1 gives tau* = (N / B)^2 = 1
  EXPECT_EQ(bias_averaging_factor(a, kTau0), 64U);
  // The random-walk term from the weights c_l = l (1..m), 2m - l (m+1..2m-1): variance of the block-mean difference
  // K^2 tau0 sum(c_l^2) / m^2, half of it.
  for (const std::size_t m : {std::size_t{1}, std::size_t{2}, std::size_t{5}, std::size_t{64}}) {
    double sum_c2 = 0.0;
    for (std::size_t l = 1; l <= m; ++l) {
      sum_c2 += static_cast<double>(l * l);
    }
    for (std::size_t l = m + 1; l < 2 * m; ++l) {
      sum_c2 += static_cast<double>((2 * m - l) * (2 * m - l));
    }
    const double k = gain_k(a);
    const double mm = static_cast<double>(m);
    const double sigma_d2 = a.noise_density * a.noise_density / (2.0 * kTau0);
    const double independent = sigma_d2 / mm + 0.5 * k * k * kTau0 * sum_c2 / (mm * mm);
    EXPECT_NEAR(expected_variance(a, kTau0, m), independent, 1.0e-12 * independent) << m;
  }
  // At tau* the two terms are equal and the deviation is B (the discrete form differs from the continuous one by
  // 1 / (2 m^2), here 1.2e-4); it is the minimum over the neighbouring factors.
  const double at_star = expected_variance(a, kTau0, 64);
  EXPECT_NEAR(at_star, 1.0, 2.0e-4);
  EXPECT_LT(at_star, expected_variance(a, kTau0, 32));
  EXPECT_LT(at_star, expected_variance(a, kTau0, 128));
  // Control: a wrong gain (K without the sqrt(6) / 2 factor) misses B by far more.
  Axis wrong = a;
  wrong.bias_instability = 1.2;
  EXPECT_GT(std::fabs(expected_variance(wrong, kTau0, 64) - 1.0), 0.1);
}

TEST(AllanHelpers, ExpectedVarianceMatchesASimulationOfTheModelAtACheapSetting) {
  // A simulation of the model alone (doubles, mt19937_64 and std::normal_distribution, no marv_plant; labelled seed 1)
  // at the labelled setting N = B = 1, tau0 = 1/64, lsb = 0.25, 2^21 samples, against the closed form at m = 1 and
  // m* = 64 and at the confidence 1 - 1e-6 per check (labelled: loose, so that the check holds on any standard library's
  // normal_distribution; the plant checks run at allan_check_confidence). Control: the perturbed model's expectation (N x 1.1
  // at m = 1, B x 1.1 at m*) is outside the interval for this record.
  constexpr double kTau0 = 0.015625;
  constexpr double kLsb = 0.25;
  constexpr std::size_t kSamples = std::size_t{1} << 21;
  constexpr double kAlpha = 1.0e-6;
  constexpr double kPerturbation = 1.1;
  const Axis a{1.0, 1.0, kLsb};
  const double sigma_d = a.noise_density / std::sqrt(2.0 * kTau0);
  const double k = gain_k(a);
  std::mt19937_64 rng(1);
  std::normal_distribution<double> normal(0.0, 1.0);
  const std::size_t m_star = bias_averaging_factor(a, kTau0);
  OverlappingAllan at_one(1);
  OverlappingAllan at_star(m_star);
  double bias = 0.0;
  for (std::size_t i = 0; i < kSamples; ++i) {
    bias += k * std::sqrt(kTau0) * normal(rng);
    const double y = bias + sigma_d * normal(rng);
    const auto count = static_cast<std::int64_t>(std::llround(y / kLsb));
    at_one.push(count);
    at_star.push(count);
  }
  for (const std::size_t m : {std::size_t{1}, m_star}) {
    const double observed = (m == 1 ? at_one : at_star).variance(kLsb);
    const Interval iv = ratio_interval(edf(kSamples, m), kAlpha);
    const double ratio = observed / expected_variance(a, kTau0, m);
    EXPECT_GE(ratio, iv.lower) << m;
    EXPECT_LE(ratio, iv.upper) << m;
    // The control perturbation that each factor resolves: N x 1.1 at m = 1, B x 1.1 at m* (at m* the white and random-walk
    // terms are equal, so N x 1.1 moves the expectation by only 1.8 %, inside this record's interval).
    Axis perturbed = a;
    (m == 1 ? perturbed.noise_density : perturbed.bias_instability) *= kPerturbation;
    const double control_ratio = observed / expected_variance(perturbed, kTau0, m);
    EXPECT_TRUE(control_ratio < iv.lower || control_ratio > iv.upper) << m;
  }
}

TEST(AllanHelpers, RecordLengthIsTheShortestThatSeparatesThePerturbedModel) {
  constexpr double kTau0 = 0.015625;
  constexpr double kAlpha = 1.0e-3;  // labelled
  constexpr double kPerturbation = 1.1;
  const Axis nominal{1.0, 1.0, 0.0};
  Axis noise_up = nominal;
  noise_up.noise_density *= kPerturbation;
  Axis bias_up = nominal;
  bias_up.bias_instability *= kPerturbation;
  const std::size_t m_star = bias_averaging_factor(nominal, kTau0);
  for (const auto& [perturbed, m] : {std::pair<Axis, std::size_t>{noise_up, 1}, std::pair<Axis, std::size_t>{bias_up, m_star}}) {
    const std::size_t n = shortest_record(nominal, perturbed, kTau0, m, kAlpha);
    ASSERT_GT(n, 0U);
    EXPECT_TRUE(separates(nominal, perturbed, kTau0, m, n, kAlpha)) << m;
    EXPECT_FALSE(separates(nominal, perturbed, kTau0, m, n - 1, kAlpha)) << m;
    // Monotone above and below on a sample of lengths (the bisection's assumption).
    for (const double f : {0.25, 0.5, 0.9, 0.999}) {
      EXPECT_FALSE(separates(nominal, perturbed, kTau0, m, static_cast<std::size_t>(f * static_cast<double>(n)), kAlpha)) << m << " " << f;
    }
    for (const double f : {1.001, 1.5, 4.0}) {
      EXPECT_TRUE(separates(nominal, perturbed, kTau0, m, static_cast<std::size_t>(f * static_cast<double>(n)), kAlpha)) << m << " " << f;
    }
    // Control: the unperturbed model never separates from itself.
    EXPECT_EQ(shortest_record(nominal, nominal, kTau0, m, kAlpha), 0U);
  }
}

TEST(AllanHelpers, BudgetRegisterValuesAreRead) {
  EXPECT_GT(marv::l06::kAllanCheckConfidence, 0.5);
  EXPECT_LT(marv::l06::kAllanCheckConfidence, 1.0);
  EXPECT_GT(marv::l06::kPerPushCheckTimeMaxS, 0.0);
  EXPECT_GT(marv::l06::kTickPeriodNumUs, 0U);
  EXPECT_GT(marv::l06::kTickPeriodDen, 0U);
}

}  // namespace
