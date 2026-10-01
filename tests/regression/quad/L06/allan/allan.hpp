// Helpers of the L6 stage (a) T1 Allan check: the chi-square inverse, the overlapping-Allan equivalent degrees of
// freedom, the streaming overlapping Allan variance, the closed-form expected Allan variance of the IMU model, and the
// record-length rule. Test code only: it may call the platform libm, which the noise stream itself never does.
#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace marv::l06::allan {

// ---- the chi-square distribution -------------------------------------------------------------------------------------
//
// The regularized incomplete gamma functions P(a, x) and Q(a, x) = 1 - P(a, x), by the series (x < a + 1) and the
// continued fraction (x >= a + 1): Press et al., Numerical Recipes in C, 2nd ed., section 6.2 (gser, gcf; Abramowitz &
// Stegun 6.5.29 and 6.5.31). The chi-square distribution with nu degrees of freedom has the CDF P(nu / 2, x / 2). The
// iteration cap and the series/fraction tolerance are labelled numerical choices (a return of NaN means the cap was hit,
// and the callers' checks then fail).
inline constexpr int kGammaIterationCap = 4000000;
inline constexpr double kGammaTolerance = 1.0e-15;
inline constexpr double kTiny = 1.0e-300;  // the tiny of the modified Lentz method (NR gcf's FPMIN)

struct GammaPQ {
  double p;
  double q;
};

inline GammaPQ gamma_pq(double a, double x) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  if (!(a > 0.0) || !(x >= 0.0)) {
    return {nan, nan};
  }
  if (x == 0.0) {
    return {0.0, 1.0};
  }
  const double log_prefactor = -x + a * std::log(x) - std::lgamma(a);
  if (x < a + 1.0) {
    double ap = a;
    double del = 1.0 / a;
    double sum = del;
    for (int i = 0; i < kGammaIterationCap; ++i) {
      ap += 1.0;
      del *= x / ap;
      sum += del;
      if (std::fabs(del) < std::fabs(sum) * kGammaTolerance) {
        const double p = sum * std::exp(log_prefactor);
        return {p, 1.0 - p};
      }
    }
    return {nan, nan};
  }
  double b = x + 1.0 - a;
  double c = 1.0 / kTiny;
  double d = 1.0 / b;
  double h = d;
  for (int i = 1; i < kGammaIterationCap; ++i) {
    const double an = -static_cast<double>(i) * (static_cast<double>(i) - a);
    b += 2.0;
    d = an * d + b;
    if (std::fabs(d) < kTiny) {
      d = kTiny;
    }
    c = b + an / c;
    if (std::fabs(c) < kTiny) {
      c = kTiny;
    }
    d = 1.0 / d;
    const double del = d * c;
    h *= del;
    if (std::fabs(del - 1.0) < kGammaTolerance) {
      const double q = std::exp(log_prefactor) * h;
      return {1.0 - q, q};
    }
  }
  return {nan, nan};
}

// The upper-tail quantile of the standard normal (z with 0.5 erfc(z / sqrt 2) = q), by bisection on std::erfc. Only the
// starting point of the chi-square inversion uses it; the answer does not depend on it.
inline double normal_upper_quantile(double q) {
  constexpr double kBound = 40.0;  // labelled: a normal quantile never leaves +-40 for a double probability
  constexpr int kBisections = 200;
  double lo = -kBound;
  double hi = kBound;
  for (int i = 0; i < kBisections; ++i) {
    const double mid = 0.5 * (lo + hi);
    (0.5 * std::erfc(mid / std::sqrt(2.0)) > q ? lo : hi) = mid;
  }
  return 0.5 * (lo + hi);
}

// The x with chi-square CDF(x; nu) = p (upper = false) or chi-square survival(x; nu) = p (upper = true). Newton's method
// on P(nu / 2, x / 2) or Q, safeguarded by a bracket (bisection when a step leaves it), started from the
// Wilson-Hilferty approximation (Wilson & Hilferty 1931; Abramowitz & Stegun 26.4.17). The convergence tolerance is a
// labelled numerical choice, far below the interval widths used.
inline double chi2_quantile(double p, double nu, bool upper) {
  constexpr double kRelativeTolerance = 1.0e-10;
  constexpr int kIterationCap = 400;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  if (!(p > 0.0 && p < 1.0 && nu > 0.0)) {
    return nan;
  }
  const double a = 0.5 * nu;
  const double log_gamma_a = std::lgamma(a);
  // g(h) = P(a, h) - p (lower) or p - Q(a, h) (upper): increasing in h, g' = the gamma density.
  auto g = [&](double h) {
    const GammaPQ v = gamma_pq(a, h);
    return upper ? p - v.q : v.p - p;
  };
  const double z = upper ? normal_upper_quantile(p) : -normal_upper_quantile(p);
  const double wh = 1.0 - 1.0 / (9.0 * a) + z / (3.0 * std::sqrt(a));
  double h = wh > 0.0 ? a * wh * wh * wh : a;
  double lo = 0.0;
  double hi = std::numeric_limits<double>::infinity();
  for (int i = 0; i < kIterationCap; ++i) {
    const double gv = g(h);
    if (std::isnan(gv)) {
      return nan;
    }
    (gv < 0.0 ? lo : hi) = h;
    const double pdf = std::exp((a - 1.0) * std::log(h) - h - log_gamma_a);
    double next = h - gv / pdf;
    if (!std::isfinite(next) || !(next > lo) || !(next < hi)) {
      next = std::isinf(hi) ? 2.0 * h : 0.5 * (lo + hi);
    }
    if (std::fabs(next - h) <= kRelativeTolerance * next) {
      return 2.0 * next;
    }
    h = next;
  }
  return nan;
}

// ---- equivalent degrees of freedom of the overlapping Allan variance ------------------------------------------------
//
// NIST SP 1065 (Riley and Howe, 2008), section 5.4.1, Table 5, "AVAR approximation formulae for each power law noise
// type", with N the number of phase data points (the number of rate samples plus one) and m = tau / tau0.
inline double edf_white_fm(double n_phase, double m) {  // W FM
  return (3.0 * (n_phase - 1.0) / (2.0 * m) - 2.0 * (n_phase - 2.0) / n_phase) * (4.0 * m * m) / (4.0 * m * m + 5.0);
}

inline double edf_random_walk_fm(double n_phase, double m) {  // RW FM
  return ((n_phase - 2.0) / m) *
         (((n_phase - 1.0) * (n_phase - 1.0) - 3.0 * m * (n_phase - 1.0) + 4.0 * m * m) /
          ((n_phase - 3.0) * (n_phase - 3.0)));
}

// The edf used by the check at tau = m tau0 for a record of n_samples rate samples: the smaller of the W FM and RW FM
// values (both noise types contribute at the checked tau; the smaller edf gives the wider, conservative interval).
inline double edf(std::size_t n_samples, std::size_t m) {
  const double n_phase = static_cast<double>(n_samples) + 1.0;
  const double mm = static_cast<double>(m);
  const double w = edf_white_fm(n_phase, mm);
  const double r = edf_random_walk_fm(n_phase, mm);
  return w < r ? w : r;
}

// ---- the streaming overlapping Allan variance ------------------------------------------------------------------------
//
// Input: the rate samples as integer counts c_0, c_1, ... (rate = count * lsb). The phase sums S_0 = 0, S_k = c_0 + ...
// + c_{k-1} are exact in int64 (they are the phase x_k = S_k * lsb * tau0, in units of lsb * tau0). The overlapping
// Allan variance at m (NIST SP 1065 section 5.2.4 eq. 11, the phase form, with N = n + 1 phase points):
//     sigma^2(m tau0) = 1 / (2 (N - 2m)) * sum_{i=0}^{N-2m-1} ((S_{i+2m} - 2 S_{i+m} + S_i) * lsb / m)^2
// (the second difference of the phase over m tau0, divided by (m tau0)^2 and with the phase's own tau0 cancelled).
// Memory: a ring of the last 2m + 1 phase sums; time O(1) per sample.
class OverlappingAllan {
 public:
  explicit OverlappingAllan(std::size_t m) : m_(m), ring_(2 * m + 1, 0) {}

  void push(std::int64_t count) {
    sum_ += count;
    ++k_;
    const std::size_t size = ring_.size();
    ring_[k_ % size] = sum_;
    if (k_ >= 2 * m_) {
      const std::int64_t d = sum_ - 2 * ring_[(k_ - m_) % size] + ring_[(k_ - 2 * m_) % size];
      acc_ += static_cast<double>(d) * static_cast<double>(d);
      ++terms_;
    }
  }

  std::size_t samples() const { return k_; }
  std::size_t terms() const { return terms_; }
  std::size_t m() const { return m_; }

  // The variance, in (lsb)^2 units times lsb2; NaN if the record is shorter than 2m + 1 phase points.
  double variance(double lsb) const {
    if (terms_ == 0) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    const double mm = static_cast<double>(m_);
    return acc_ * lsb * lsb / (2.0 * static_cast<double>(terms_) * mm * mm);
  }

 private:
  std::size_t m_;
  std::vector<std::int64_t> ring_;
  std::int64_t sum_ = 0;
  std::size_t k_ = 0;
  std::size_t terms_ = 0;
  double acc_ = 0.0;
};

// ---- the closed-form expected Allan variance of the IMU model ----------------------------------------------------
//
// One axis of the model (sim/plant/src/imu_model.hpp): the rate sample is y_k = b_k + sigma_d v_k + q_k at the interval
// tau0, with b_k = b_{k-1} + K sqrt(tau0) w_k a random walk, v and w independent unit normals, sigma_d = N / sqrt(2 tau0)
// (N the one-sided noise density, B the bias instability), K = (sqrt(6) / 2) B^2 / N, and q_k the quantisation error of a
// step lsb. The overlapping Allan variance at tau = m tau0 is the mean of half the squared difference of adjacent
// m-sample means, and is a sum over the independent parts:
//  - White FM, sample variance s^2 = sigma_d^2 + lsb^2 / 12: the mean of m samples has variance s^2 / m; the difference
//    of two adjacent means 2 s^2 / m; half of it s^2 / m. (NIST SP 1065 section 7.1 eq. 67's white FM Allan variance h_0 / (2 tau)
//    is this with h_0 / 2 = sigma_d^2 tau0 = N^2 / 2, so h_0 = N^2; the discrete form is exact.)
//  - Quantisation: q_k is rounding noise. ASSUMPTION, INFERRED: q_k is uniform on +-lsb / 2 and independent from sample to
//    sample, so its variance is lsb^2 / 12 and it adds to the white FM term. Rounding a signal whose noise is many lsb
//    wide is only approximately uniform; the term is below one part in 1e3 of sigma_d^2 for the profile's figures
//    (the test prints the ratio), and the check's width is the chi-square interval, so the assumption does not decide
//    a result.
//  - Random walk FM: with steps xi_l of variance s_b^2 = K^2 tau0, the difference of the means of the blocks [0, m) and
//    [m, 2m) of b is (1 / m) sum_j sum_{l=j+1}^{j+m} xi_l = (1 / m) sum_l c_l xi_l, c_l = l (l = 1..m), 2m - l
//    (l = m+1..2m-1). The sum of c_l^2 is m (m + 1)(2m + 1) / 6 + (m - 1) m (2m - 1) / 6 = m (2 m^2 + 1) / 3 (the test
//    checks it against the weights). So the variance of the difference is s_b^2 (2 m^2 + 1) / (3 m), half of it
//    K^2 tau0 (2 m^2 + 1) / (6 m); its continuous limit K^2 tau / 3 is the RW FM Allan variance of NIST SP 1065
//    section 7.1 eq. 67, h_{-2} (2 pi)^2 tau / 6, with h_{-2} = K^2 / (2 pi^2).
//  - The sum is sigma^2(m) = (sigma_d^2 + lsb^2 / 12) / m + K^2 tau0 (2 m^2 + 1) / (6 m). Its continuous form
//    N^2 / (2 tau) + K^2 tau / 3 has its minimum at tau* = (N / B)^2 of value B^2 by the choice of K.
struct Axis {
  double noise_density;     // N
  double bias_instability;  // B
  double lsb;
};

inline double gain_k(const Axis& a) { return std::sqrt(6.0) * 0.5 * a.bias_instability * a.bias_instability / a.noise_density; }

inline double expected_variance(const Axis& a, double tau0, std::size_t m) {
  const double mm = static_cast<double>(m);
  const double sigma_d2 = a.noise_density * a.noise_density / (2.0 * tau0);
  const double white = (sigma_d2 + a.lsb * a.lsb / 12.0) / mm;
  const double k = gain_k(a);
  const double walk = k * k * tau0 * (2.0 * mm * mm + 1.0) / (6.0 * mm);
  return white + walk;
}

// m* = round(tau* / tau0), tau* = (N / B)^2: the averaging factor of the bias-instability check.
inline std::size_t bias_averaging_factor(const Axis& a, double tau0) {
  const double ratio = a.noise_density / a.bias_instability;
  return static_cast<std::size_t>(std::llround(ratio * ratio / tau0));
}

// ---- the interval and the record-length rule -------------------------------------------------------------------------
//
// For an estimate s^2 of a variance sigma^2 with nu equivalent degrees of freedom, nu s^2 / sigma^2 is chi-square
// distributed (NIST SP 1065 eq. 44). The two-sided interval of confidence 1 - alpha for the ratio s^2 / sigma^2 is
// [chi2_{alpha/2}(nu) / nu, chi2_{1 - alpha/2}(nu) / nu], the same statement as eq. 45's interval for sigma^2 around s^2
// (sigma^2 in [nu s^2 / chi2_upper, nu s^2 / chi2_lower]).
struct Interval {
  double lower;  // of the variance ratio
  double upper;
  double nu;
};

inline Interval ratio_interval(double nu, double alpha) {
  const double lo = chi2_quantile(0.5 * alpha, nu, false);
  const double hi = chi2_quantile(0.5 * alpha, nu, true);
  return {lo / nu, hi / nu, nu};
}

// The rule of the spec: the perturbed model's own interval lies wholly outside the nominal one, which for the same nu
// and the two models' expected variances means  sigma2_perturbed / sigma2_nominal >= chi2_upper / chi2_lower.
inline bool separates(const Axis& nominal, const Axis& perturbed, double tau0, std::size_t m, std::size_t n_samples,
                      double alpha) {
  const double nu = edf(n_samples, m);
  if (!(nu > 0.0)) {
    return false;
  }
  const Interval iv = ratio_interval(nu, alpha);
  return expected_variance(perturbed, tau0, m) / expected_variance(nominal, tau0, m) >= iv.upper / iv.lower;
}

// The shortest record, in samples, at which `separates` holds. nu grows with the record, so the predicate is monotone
// above the starting record of 2m + 3 samples (where it is false, the caller checks that); the search doubles then
// bisects. Returns 0 if no record below the cap separates.
inline std::size_t shortest_record(const Axis& nominal, const Axis& perturbed, double tau0, std::size_t m, double alpha) {
  constexpr std::size_t kCapSamples = std::size_t{1} << 40;  // labelled: far beyond any record the check could run
  std::size_t lo = 2 * m + 3;
  if (separates(nominal, perturbed, tau0, m, lo, alpha)) {
    return 0;
  }
  std::size_t hi = lo;
  while (!separates(nominal, perturbed, tau0, m, hi, alpha)) {
    lo = hi;
    hi *= 2;
    if (hi > kCapSamples) {
      return 0;
    }
  }
  while (hi - lo > 1) {
    const std::size_t mid = lo + (hi - lo) / 2;
    (separates(nominal, perturbed, tau0, m, mid, alpha) ? hi : lo) = mid;
  }
  return hi;
}

}  // namespace marv::l06::allan
