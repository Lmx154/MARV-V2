// Shared helpers of the L6 noise tests: the inverse normal, the family-wise confidence arithmetic, sample moments, the
// cross-correlation statistic and ulp distance. Test code only: it may call the platform libm, which the
// noise stream itself never does.
#pragma once

#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace marv::l06 {

// Percentage points of the normal distribution, Wichura, "Algorithm AS241: The percentage points of the normal
// distribution", Applied Statistics 37(3), 477-484 (1988), routine PPND16 (relative accuracy about 1e-16). The
// coefficients are those of the algorithm, as published and as implemented in CPython's statistics.NormalDist.inv_cdf.
inline double horner(const double* c, std::size_t n, double x) {
  double acc = c[0];
  for (std::size_t i = 1; i < n; ++i) {
    acc = acc * x + c[i];
  }
  return acc;
}

inline double ppnd16(double p) {
  constexpr double kCentralNum[] = {2.5090809287301226727e+3, 3.3430575583588128105e+4, 6.7265770927008700853e+4,
                                    4.5921953931549871457e+4, 1.3731693765509461125e+4, 1.9715909503065514427e+3,
                                    1.3314166789178437745e+2, 3.3871328727963666080e+0};
  constexpr double kCentralDen[] = {5.2264952788528545610e+3, 2.8729085735721942674e+4, 3.9307895800092710610e+4,
                                    2.1213794301586595867e+4, 5.3941960214247511077e+3, 6.8718700749205790830e+2,
                                    4.2313330701600911252e+1, 1.0};
  constexpr double kMidNum[] = {7.74545014278341407640e-4, 2.27238449892691845833e-2, 2.41780725177450611770e-1,
                                1.27045825245236838258e+0, 3.64784832476320460504e+0, 5.76949722146069140550e+0,
                                4.63033784615654529590e+0, 1.42343711074968357734e+0};
  constexpr double kMidDen[] = {1.05075007164441684324e-9, 5.47593808499534494600e-4, 1.51986665636164571966e-2,
                                1.48103976427480074590e-1, 6.89767334985100004550e-1, 1.67638483018380384940e+0,
                                2.05319162663775882187e+0, 1.0};
  constexpr double kTailNum[] = {2.01033439929228813265e-7, 2.71155556874348757815e-5, 1.24266094738807843860e-3,
                                 2.65321895265761230930e-2, 2.96560571828504891230e-1, 1.78482653991729133580e+0,
                                 5.46378491116411436990e+0, 6.65790464350110377720e+0};
  constexpr double kTailDen[] = {2.04426310338993978564e-15, 1.42151175831644588870e-7, 1.84631831751005468180e-5,
                                 7.86869131145613259100e-4, 1.48753612908506148525e-2, 1.36929880922735805310e-1,
                                 5.99832206555887937690e-1, 1.0};
  constexpr double kCentralSplit = 0.425;
  constexpr double kCentralOffset = 0.180625;
  constexpr double kTailSplit = 5.0;
  constexpr double kMidOffset = 1.6;
  const double q = p - 0.5;
  if (std::fabs(q) <= kCentralSplit) {
    const double r = kCentralOffset - q * q;
    return q * horner(kCentralNum, 8, r) / horner(kCentralDen, 8, r);
  }
  double r = std::sqrt(-std::log(q <= 0.0 ? p : 1.0 - p));
  double x = 0.0;
  if (r <= kTailSplit) {
    r -= kMidOffset;
    x = horner(kMidNum, 8, r) / horner(kMidDen, 8, r);
  } else {
    r -= kTailSplit;
    x = horner(kTailNum, 8, r) / horner(kTailDen, 8, r);
  }
  return q < 0.0 ? -x : x;
}

// The two-sided critical value z with P(|Z| > z) = alpha, from the lower tail so a small alpha keeps its precision.
inline double z_two_sided(double alpha) { return -ppnd16(alpha / 2.0); }

// Bonferroni's per-test level for a family of m tests at family-wise confidence c: alpha = (1 - c)/m (Dunn, J. Am.
// Stat. Assoc. 56, 52-64, 1961). It holds under any dependence between the tests, which matters here: correlations of
// one pair at different lags, and moments of one sample, are not independent (Luis, 2026-10-01, decision 0012, B).
inline double bonferroni_alpha(double confidence, std::size_t m) {
  return (1.0 - confidence) / static_cast<double>(m);
}

struct Moments {
  double mean = 0.0;
  double variance = 0.0;
  double skewness = 0.0;
  double excess_kurtosis = 0.0;
};

// Mean, variance (divisor N), skewness m3 / m2^1.5 and excess kurtosis m4 / m2^2 - 3, two passes.
inline Moments sample_moments(const std::vector<double>& x) {
  const auto n = static_cast<double>(x.size());
  double sum = 0.0;
  for (const double v : x) {
    sum += v;
  }
  Moments m;
  m.mean = sum / n;
  double m2 = 0.0;
  double m3 = 0.0;
  double m4 = 0.0;
  for (const double v : x) {
    const double d = v - m.mean;
    m2 += d * d;
    m3 += d * d * d;
    m4 += d * d * d * d;
  }
  m2 /= n;
  m3 /= n;
  m4 /= n;
  m.variance = m2;
  m.skewness = m3 / (m2 * std::sqrt(m2));
  m.excess_kurtosis = m4 / (m2 * m2) - 3.0;
  return m;
}

// Sample cross-correlation of a_i with b_{i+lag}: sum over the n - lag overlapping terms of the centred products,
// divided by the root of the product of the two full-series sums of squares (the usual ccf estimator). For
// independent series it is about N(0, 1/n) (slightly narrower for lag > 0), so |rho| <= z / sqrt(n) is a bound.
inline double cross_correlation(const std::vector<double>& a, const std::vector<double>& b, std::size_t lag) {
  const std::size_t n = a.size();
  double ma = 0.0;
  double mb = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    ma += a[i];
    mb += b[i];
  }
  ma /= static_cast<double>(n);
  mb /= static_cast<double>(n);
  double saa = 0.0;
  double sbb = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    saa += (a[i] - ma) * (a[i] - ma);
    sbb += (b[i] - mb) * (b[i] - mb);
  }
  double sab = 0.0;
  for (std::size_t i = 0; i + lag < n; ++i) {
    sab += (a[i] - ma) * (b[i + lag] - mb);
  }
  return sab / std::sqrt(saa * sbb);
}

// Distance in representable doubles between a and b (0 when equal), through the monotone integer order of doubles.
inline std::int64_t ordered_bits(double x) {
  const auto i = std::bit_cast<std::int64_t>(x);
  return i < 0 ? INT64_MIN - i : i;
}

inline std::uint64_t ulp_distance(double a, double b) {
  const std::int64_t oa = ordered_bits(a);
  const std::int64_t ob = ordered_bits(b);
  return oa > ob ? static_cast<std::uint64_t>(oa) - static_cast<std::uint64_t>(ob)
                 : static_cast<std::uint64_t>(ob) - static_cast<std::uint64_t>(oa);
}

}  // namespace marv::l06
