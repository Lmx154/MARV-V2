#pragma once

// The T2 bench-log replay of the attitude estimator (quad spec 4.0 M3 done-when, decision 0042 item 2): parse a bench IMU
// log, replay every sample through marv::estimator::Mahony<float> with quad_fc's flag rule, and run the two checks and
// their controls. Test code, host only; the numbers come in through Params (read from the parameter set by the test).
//
// Parse (imu_log_format.hpp; tools/target/imu_log.py is the reference decoder). Bytes before the first record that checks
// are skipped (a capture may start mid-record); after it every byte must belong to a record. A log is rejected unless it is
// clean (decision 0042 item 2: no deadline miss, FIFO overflow or stamp gap): every record's CRC-32 checks, the sequence
// counter runs +1 per record, no status record has a nonzero ring_overflows, deadline_misses, fifo_overflows,
// timing_faults, stamp_clamps, order_faults or spi_errors counter (all cumulative since boot), at least one status record is
// present, and no two consecutive stamps differ by more than the tick period num / den us by more than tol_us (exact in
// integers, |dt den - num| <= tol den, the decoder's rule). An incomplete record at the end of the file (the capture
// stopped) is dropped and counted.
//
// Replay. The estimator is stepped exactly as quad_fc's tick() steps it: a sample whose GyroValid bit is clear is not
// stepped (the next step's dt spans from the last stepped stamp), one whose AccelValid bit is clear is stepped with no
// accelerometer direction (NaN: no correction), dt is the stamp difference in microseconds (0 for the first step or a
// non-increasing stamp, clamped to uint32).
//
// Still samples. Both valid bits set, every gyro axis below kStillGyroRadS and |f| in the open interval (accel_healthy_min g,
// accel_healthy_max g): the Betaflight 4.5.1 gates (src/main/flight/imu.c, quiet gyro below 15 deg/s, lines 87-88 and
// 364-377; imuIsAccelerometerHealthy, 0.9 g - 1.1 g, lines 326-338; INFERRED from a source read, as in
// fw/estimator/include/marv/estimator/mahony.hpp). A still segment is a maximal run of consecutive still samples; still 1
// and still 2 are the two longest, in time order.
//
// Checks. (a) At the last sample of each still segment the angle between the estimate's down direction in body axes
// (R^T e_z) and -mean(f)/|mean(f)| over that segment is at most share. (c') From a copy of the estimator at the last
// sample of still 1, the samples after it up to the last of still 2 are stepped with the accelerometer correction off; the
// resulting down direction is within theta_rot / 2 of still 2's -mean(f) direction, theta_rot the angle between the two
// segments' mean f directions. The log is a valid T2 log only if there are two still segments, theta_rot >= pi/2, the gap
// between them is at most 10 s, and each segment lasts at least T_s = ln(theta_rot / share) / Kp.
// Controls (each must fail its check): Control::NegateStill2Accel negates still 2's dominant mean-f axis in the estimator's
// input for every sample from the first of still 2 on; Control::NegateRotationGyro negates, in the gyro-only replay, the
// gyro axis with the largest integral over the rotation.

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "marv/estimator/mahony.hpp"
#include "marv/hal_target/imu_log_format.hpp"
#include "marv/prim/constants.hpp"
#include "marv/types/imu_sample.hpp"

namespace marv::l07 {

namespace fmt = marv::hal_target::imu_log;

// Protocol constants (decision 0042 item 2).
inline constexpr double kStillGyroDegPerS = 15.0;  // Betaflight 4.5.1 imu.c quiet-gyro gate
inline constexpr double kStillGyroRadS = kStillGyroDegPerS * prim::kPi / prim::kDegreesPerHalfTurn;
inline constexpr double kRotationMinRad = prim::kPi / 2.0;  // the protocol's smallest rotation
inline constexpr double kRotationWindowS = 10.0;            // the protocol's longest rotation

struct Params {
  estimator::MahonyConfig<float> mahony;
  double share = 0.0;           // attitude_tilt_error_max, rad
  std::uint32_t tick_num = 0;   // tick period num / den us
  std::uint32_t tick_den = 1;
  std::uint32_t tol_us = 0;     // rate_period_tolerance_us
};

// ---------------------------------------------------------------------------------------------------------- parse

struct ParsedLog {
  std::vector<ImuSample> samples;
  std::array<std::uint32_t, fmt::kStatusFields> last_status{};
  std::size_t data_records = 0;
  std::size_t status_records = 0;
  std::size_t leading_bytes = 0;
  std::size_t dropped_tail_bytes = 0;
  std::string error;  // empty: the log is accepted
  [[nodiscard]] bool ok() const { return error.empty(); }
};

namespace detail {
inline std::uint32_t rd32(const std::uint8_t* p) {
  return static_cast<std::uint32_t>(p[0]) | static_cast<std::uint32_t>(p[1]) << 8 | static_cast<std::uint32_t>(p[2]) << 16 |
         static_cast<std::uint32_t>(p[3]) << 24;
}
inline std::uint64_t rd64(const std::uint8_t* p) {
  return static_cast<std::uint64_t>(rd32(p)) | static_cast<std::uint64_t>(rd32(p + 4)) << 32;
}
inline float rdf(const std::uint8_t* p) { return std::bit_cast<float>(rd32(p)); }

struct StatusName {
  fmt::StatusField field;
  const char* name;
};
// The counters that must be zero in a clean log.
inline constexpr std::array kMustBeZero{
    StatusName{fmt::StatusField::RingOverflows, "ring_overflows"},
    StatusName{fmt::StatusField::DeadlineMisses, "deadline_misses"},
    StatusName{fmt::StatusField::FifoOverflows, "fifo_overflows"},
    StatusName{fmt::StatusField::TimingFaults, "timing_faults"},
    StatusName{fmt::StatusField::StampClamps, "stamp_clamps"},
    StatusName{fmt::StatusField::OrderFaults, "order_faults"},
    StatusName{fmt::StatusField::SpiErrors, "spi_errors"}};
}  // namespace detail

[[nodiscard]] inline ParsedLog parse_log(std::span<const std::uint8_t> buf, const Params& p) {
  ParsedLog r;
  const std::size_t n = buf.size();
  std::size_t pos = 0;
  bool seen = false;
  std::uint32_t last_seq = 0;
  while (pos + fmt::kMagicBytes <= n) {
    const std::uint8_t* b = buf.data() + pos;
    const bool is_data = std::equal(fmt::kDataMagic.begin(), fmt::kDataMagic.end(), b);
    const bool is_status = std::equal(fmt::kStatusMagic.begin(), fmt::kStatusMagic.end(), b);
    if (!is_data && !is_status) {
      if (seen) {
        r.error = "unframed bytes at offset " + std::to_string(pos);
        return r;
      }
      ++pos;
      ++r.leading_bytes;
      continue;
    }
    bool bad = n - pos < fmt::kHeaderBytes;
    if (bad && seen) {  // an incomplete header at the end of the file
      r.dropped_tail_bytes = n - pos;
      break;
    }
    std::uint32_t seq = 0;
    std::uint32_t count = 0;
    std::size_t item = 0;
    std::size_t end = 0;
    if (!bad) {
      seq = detail::rd32(b + fmt::kMagicBytes);
      count = detail::rd32(b + fmt::kMagicBytes + fmt::kWordBytes);
      item = is_data ? fmt::kSampleBytes : fmt::kWordBytes;
      bad = is_status ? count != fmt::kStatusFields : count > n / fmt::kSampleBytes;
    }
    if (!bad) {
      end = fmt::kHeaderBytes + static_cast<std::size_t>(count) * item + fmt::kCrcBytes;
      if (pos + end > n) {
        if (seen) {
          r.dropped_tail_bytes = n - pos;
          break;
        }
        bad = true;
      }
    }
    if (!bad) {
      bad = fmt::crc32(std::span<const std::uint8_t>(b, end - fmt::kCrcBytes)) != detail::rd32(b + end - fmt::kCrcBytes);
    }
    if (bad) {
      if (seen) {
        r.error = "record at offset " + std::to_string(pos) + " fails its CRC-32 or header check";
        return r;
      }
      ++pos;
      ++r.leading_bytes;
      continue;
    }
    if (seen && seq != last_seq + 1) {
      r.error = "sequence gap at offset " + std::to_string(pos) + ": expected " + std::to_string(last_seq + 1) + ", got " +
                std::to_string(seq);
      return r;
    }
    seen = true;
    last_seq = seq;
    const std::uint8_t* body = b + fmt::kHeaderBytes;
    if (is_data) {
      ++r.data_records;
      for (std::uint32_t i = 0; i < count; ++i) {
        const std::uint8_t* s = body + static_cast<std::size_t>(i) * fmt::kSampleBytes;
        ImuSample out{};
        out.t_us = detail::rd64(s);
        for (std::size_t a = 0; a < prim::kSpatialDim; ++a) {
          out.gyro_rad_s[a] = detail::rdf(s + sizeof(std::uint64_t) + a * sizeof(float));
          out.accel_m_s2[a] = detail::rdf(s + sizeof(std::uint64_t) + (prim::kSpatialDim + a) * sizeof(float));
        }
        out.flags = detail::rd32(s + fmt::kSampleBytes - sizeof(std::uint32_t));
        r.samples.push_back(out);
      }
    } else {
      ++r.status_records;
      for (std::size_t f = 0; f < fmt::kStatusFields; ++f) {
        r.last_status[f] = detail::rd32(body + f * fmt::kWordBytes);
        for (const auto& z : detail::kMustBeZero) {
          if (static_cast<std::size_t>(z.field) == f && r.last_status[f] != 0 && r.error.empty()) {
            r.error = std::string("status counter ") + z.name + " is " + std::to_string(r.last_status[f]);
          }
        }
      }
    }
    pos += end;
  }
  if (seen && pos < n) {  // fewer bytes than a magic: the start of a record the capture stopped in
    r.dropped_tail_bytes = n - pos;
  }
  if (!seen) {
    r.error = "no valid record";
    return r;
  }
  if (!r.error.empty()) {
    return r;
  }
  if (r.status_records == 0) {
    r.error = "no status record: the counters cannot show the log is clean";
    return r;
  }
  for (std::size_t k = 1; k < r.samples.size(); ++k) {
    const std::int64_t dt = static_cast<std::int64_t>(r.samples[k].t_us) - static_cast<std::int64_t>(r.samples[k - 1].t_us);
    const std::int64_t off = dt * static_cast<std::int64_t>(p.tick_den) - static_cast<std::int64_t>(p.tick_num);
    if (std::llabs(off) > static_cast<std::int64_t>(p.tol_us) * static_cast<std::int64_t>(p.tick_den)) {
      r.error = "stamp gap at sample " + std::to_string(k) + ": spacing " + std::to_string(dt) + " us";
      return r;
    }
  }
  return r;
}

// ---------------------------------------------------------------------------------------------------------- replay

// quad_fc's tick() estimator step (fw/compositions/quad_fc/src/quad_fc.cpp), on one sample. accel_on false: no correction.
class Replayer {
 public:
  explicit Replayer(const estimator::MahonyConfig<float>& cfg) { est_.init(cfg); }
  void step(const ImuSample& s, bool accel_on = true) {
    if ((s.flags & imu_flag(ImuFlag::GyroValid)) == 0) {
      return;
    }
    const std::uint64_t since = started_ && s.t_us > prev_us_ ? s.t_us - prev_us_ : 0;
    prev_us_ = s.t_us;
    started_ = true;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const prim::Vec3<float> no_accel{nan, nan, nan};
    const prim::Vec3<float> accel = accel_on && (s.flags & imu_flag(ImuFlag::AccelValid)) != 0 ? s.accel_m_s2 : no_accel;
    est_.step(s.gyro_rad_s, accel,
              since > std::numeric_limits<std::uint32_t>::max() ? std::numeric_limits<std::uint32_t>::max()
                                                                : static_cast<std::uint32_t>(since));
  }
  [[nodiscard]] const estimator::Mahony<float>& estimator() const { return est_; }

 private:
  estimator::Mahony<float> est_;
  std::uint64_t prev_us_ = 0;
  bool started_ = false;
};

using Dir = std::array<double, prim::kSpatialDim>;

[[nodiscard]] inline double dot(const Dir& a, const Dir& b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
[[nodiscard]] inline double angle_between(const Dir& a, const Dir& b) {
  const Dir c{a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
  return std::atan2(std::sqrt(dot(c, c)), dot(a, b));
}
[[nodiscard]] inline Dir unit(const Dir& a) {
  const double n = std::sqrt(dot(a, a));
  return {a[0] / n, a[1] / n, a[2] / n};
}
// The estimate's down direction in body axes, R^T e_z.
[[nodiscard]] inline Dir down_body(const estimator::Mahony<float>& e) {
  const auto& q = e.quat();
  const double w = q.w;
  const double x = q.x;
  const double y = q.y;
  const double z = q.z;
  return {2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)};
}

struct Segment {
  std::size_t first = 0;  // inclusive
  std::size_t last = 0;   // inclusive
};

[[nodiscard]] inline bool is_still(const ImuSample& s, const Params& p) {
  constexpr std::uint32_t kBoth = imu_flag(ImuFlag::GyroValid) | imu_flag(ImuFlag::AccelValid);
  if ((s.flags & kBoth) != kBoth) {
    return false;
  }
  for (std::size_t a = 0; a < prim::kSpatialDim; ++a) {
    if (!(std::fabs(static_cast<double>(s.gyro_rad_s[a])) < kStillGyroRadS)) {
      return false;
    }
  }
  const double g = prim::kStandardGravity;
  const double lo = static_cast<double>(p.mahony.accel_healthy_min) * g;
  const double hi = static_cast<double>(p.mahony.accel_healthy_max) * g;
  double n2 = 0.0;
  for (std::size_t a = 0; a < prim::kSpatialDim; ++a) {
    n2 += static_cast<double>(s.accel_m_s2[a]) * static_cast<double>(s.accel_m_s2[a]);
  }
  return lo * lo < n2 && n2 < hi * hi;
}

// The two longest still segments (by sample count), in time order; fewer if the log has fewer.
[[nodiscard]] inline std::vector<Segment> two_longest_still(std::span<const ImuSample> s, const Params& p) {
  std::vector<Segment> runs;
  for (std::size_t k = 0; k < s.size(); ++k) {
    if (!is_still(s[k], p)) {
      continue;
    }
    if (!runs.empty() && runs.back().last + 1 == k) {
      runs.back().last = k;
    } else {
      runs.push_back({k, k});
    }
  }
  std::stable_sort(runs.begin(), runs.end(), [](const Segment& a, const Segment& b) {
    return a.last - a.first > b.last - b.first;
  });
  if (runs.size() > 2) {
    runs.resize(2);
  }
  std::sort(runs.begin(), runs.end(), [](const Segment& a, const Segment& b) { return a.first < b.first; });
  return runs;
}

[[nodiscard]] inline Dir mean_accel(std::span<const ImuSample> s, const Segment& g) {
  Dir m{0.0, 0.0, 0.0};
  for (std::size_t k = g.first; k <= g.last; ++k) {
    for (std::size_t a = 0; a < prim::kSpatialDim; ++a) {
      m[a] += static_cast<double>(s[k].accel_m_s2[a]);
    }
  }
  const double c = static_cast<double>(g.last - g.first + 1);
  return {m[0] / c, m[1] / c, m[2] / c};
}

[[nodiscard]] inline double seconds(const ImuSample& a, const ImuSample& b) {
  return static_cast<double>(b.t_us - a.t_us) / prim::kMicrosecondsPerSecond;
}

enum class Control : std::uint8_t { None, NegateStill2Accel, NegateRotationGyro };

struct Report {
  std::string invalid;  // empty: the log is a valid T2 log
  Segment still1;
  Segment still2;
  double theta_rot = 0.0;      // rad
  double rotation_s = 0.0;     // end of still 1 to start of still 2
  double ts_required_s = 0.0;  // ln(theta_rot / share) / Kp
  double still1_s = 0.0;
  double still2_s = 0.0;
  double err_a1 = 0.0;  // rad
  double err_a2 = 0.0;
  double err_c = 0.0;    // rad
  double bound_c = 0.0;  // theta_rot / 2
  bool pass_a = false;
  bool pass_c = false;
  [[nodiscard]] bool valid() const { return invalid.empty(); }
};

[[nodiscard]] inline Report evaluate(std::span<const ImuSample> samples, const Params& p, Control control = Control::None) {
  Report r;
  const std::vector<Segment> segs = two_longest_still(samples, p);
  if (segs.size() < 2) {
    r.invalid = "fewer than two still segments";
    return r;
  }
  r.still1 = segs[0];
  r.still2 = segs[1];
  const Dir f1 = mean_accel(samples, r.still1);
  const Dir f2 = mean_accel(samples, r.still2);
  r.theta_rot = angle_between(f1, f2);
  r.rotation_s = seconds(samples[r.still1.last], samples[r.still2.first]);
  r.still1_s = seconds(samples[r.still1.first], samples[r.still1.last]);
  r.still2_s = seconds(samples[r.still2.first], samples[r.still2.last]);
  if (!(p.mahony.kp > 0.0F) || !(p.share > 0.0)) {
    r.invalid = "Kp or the tilt share is not positive";
    return r;
  }
  r.ts_required_s = std::log(r.theta_rot / p.share) / static_cast<double>(p.mahony.kp);
  if (r.theta_rot < kRotationMinRad) {
    r.invalid = "theta_rot is below pi/2";
    return r;
  }
  if (r.rotation_s > kRotationWindowS) {
    r.invalid = "the rotation between the still segments takes longer than the protocol window";
    return r;
  }
  if (r.still1_s < r.ts_required_s || r.still2_s < r.ts_required_s) {
    r.invalid = "a still segment is shorter than T_s = ln(theta_rot / share) / Kp";
    return r;
  }
  std::size_t axis_a = 0;
  for (std::size_t a = 1; a < prim::kSpatialDim; ++a) {
    if (std::fabs(f2[a]) > std::fabs(f2[axis_a])) {
      axis_a = a;
    }
  }
  std::array<double, prim::kSpatialDim> rotated{};
  for (std::size_t k = r.still1.last + 1; k < r.still2.first; ++k) {
    const double dt = seconds(samples[k - 1], samples[k]);
    for (std::size_t a = 0; a < prim::kSpatialDim; ++a) {
      rotated[a] += static_cast<double>(samples[k].gyro_rad_s[a]) * dt;
    }
  }
  std::size_t axis_c = 0;
  for (std::size_t a = 1; a < prim::kSpatialDim; ++a) {
    if (std::fabs(rotated[a]) > std::fabs(rotated[axis_c])) {
      axis_c = a;
    }
  }

  Replayer main(p.mahony);
  std::optional<Replayer> at_end1;
  bool ok_a = true;
  for (std::size_t k = 0; k <= r.still2.last; ++k) {
    ImuSample s = samples[k];
    if (control == Control::NegateStill2Accel && k >= r.still2.first) {
      s.accel_m_s2[axis_a] = -s.accel_m_s2[axis_a];
    }
    main.step(s);
    if (k == r.still1.last) {
      at_end1 = main;
      r.err_a1 = angle_between(down_body(main.estimator()), unit({-f1[0], -f1[1], -f1[2]}));
      ok_a = ok_a && main.estimator().valid();
    }
  }
  r.err_a2 = angle_between(down_body(main.estimator()), unit({-f2[0], -f2[1], -f2[2]}));
  ok_a = ok_a && main.estimator().valid();
  r.pass_a = ok_a && r.err_a1 <= p.share && r.err_a2 <= p.share;

  Replayer gyro_only = *at_end1;
  for (std::size_t k = r.still1.last + 1; k <= r.still2.last; ++k) {
    ImuSample s = samples[k];
    if (control == Control::NegateRotationGyro) {
      s.gyro_rad_s[axis_c] = -s.gyro_rad_s[axis_c];
    }
    gyro_only.step(s, false);
  }
  r.bound_c = r.theta_rot / 2.0;
  r.err_c = angle_between(down_body(gyro_only.estimator()), unit({-f2[0], -f2[1], -f2[2]}));
  r.pass_c = gyro_only.estimator().valid() && r.err_c <= r.bound_c;
  return r;
}

}  // namespace marv::l07
