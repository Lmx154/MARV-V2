#pragma once

// The T2 bench-log replay of the attitude estimator (quad spec 4.0 M3 done-when, decision 0042 item 2): parse a bench IMU
// log, replay every sample through marv::estimator::Mahony<float> with quad_fc's flag rule, and run the two checks and
// their controls. Test code, host only; the numbers come in through Params (read from the parameter set by the test).
//
// Boot gyro calibration. The logged stream is what the flight composition's tick() receives, before its calibration. The
// replay therefore applies it as tick() does: marv::estimator::GyroCal (gyro_cal_duration_us, gyro_cal_movement_max) takes
// every sample from the log's first on, and once it is complete its bias is subtracted from a copy of each sample, the
// completing one included, before the estimator. A T2 log starts with the board still (the protocol), which plays the role
// of the boot calibration. If the calibration has not completed by the end of the trimmed still segment 1, the log is not a
// valid T2 log. The gyro-path check (c') runs on the calibrated gyro, as the flight code would. Control::NoCalibration
// replays without it: a log with a gyro bias must then fail (a).
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
// Replay. The estimator is stepped exactly as quad_fc's tick() steps it (the calibration first, see above): a sample whose GyroValid bit is clear is not
// stepped (the next step's dt spans from the last stepped stamp), one whose AccelValid bit is clear is stepped with no
// accelerometer direction (NaN: no correction), dt is the stamp difference in microseconds (0 for the first step or a
// non-increasing stamp, clamped to uint32).
//
// Still samples. Both valid bits set, every gyro axis below kStillGyroRadS and |f| in the open interval (accel_healthy_min g,
// accel_healthy_max g): the Betaflight 4.5.1 gates (src/main/flight/imu.c, quiet gyro below 15 deg/s, lines 87-88 and
// 364-377; imuIsAccelerometerHealthy, 0.9 g - 1.1 g, lines 326-338; INFERRED from a source read, as in
// fw/estimator/include/marv/estimator/mahony.hpp). A still run is a maximal run of consecutive still samples. A run joins the
// still segment before it if the rotation over the segment's gaps, summed including the new gap, stays at most Params::share.
// The rotation over a gap is the sum of |gyro| (the vector norm) times the stamp difference over the samples after the earlier
// run's last up to the later run's first. A segment whose gaps turn the board by no more than the tilt budget in all cannot
// have moved the reference of check (a) by more than the budget through them. Still 1 and still 2 are the two longest
// segments, in time order, and each is then trimmed by Params::trim_s at both ends. The mean f, check (a), check (c') and T_s
// all use the trimmed segments (the protocol's rotation window uses the untrimmed ones).
// The trim is a test-harness scenario value (the lead's decision on the M3 P3b review, set by the caller). Why the merge: a
// hand-held log has single-sample spikes and tremor that would split a run (theta_rot ~ 0 from two pieces of one segment). A
// fixed time gap (50 ms) failed on a bench log whose rotation started gently, dipping below the gate many times: the short
// gaps were merged and still 1 ended inside the rotation. A bound on each gap alone failed the same way (bench dry run 2: every
// gap of the gentle start turned the board by less than the budget, and the chain of them ran 1 s into the rotation), so the
// bound is on the sum over the segment. Why the trim: the segment's last samples can hold the slow start of the rotation, up to
// the 15 deg/s gate, which would leave the estimate tilted against the segment's mean f for a sound estimator.
//
// Checks. (a) At the last sample of each still segment the angle between the estimate's down direction in body axes
// (R^T e_z) and -mean(f)/|mean(f)| over that segment is at most share. (c') From a copy of the estimator at the last
// sample of still 1, the samples after it up to the last of still 2 are stepped with the accelerometer correction off; the
// resulting down direction is within theta_rot / 2 of still 2's -mean(f) direction, theta_rot the angle between the two
// segments' mean f directions. The log is a valid T2 log only if there are two still segments, theta_rot >= pi/2, the gap
// between them is at most 10 s, and each segment lasts at least T_s = ln(theta_rot / share) / Kp.
// Controls (each must fail its check): Control::NegateStill2Accel negates still 2's dominant mean-f axis in the estimator's
// input for every sample from the first of still 2 on; Control::NegateRotationGyro negates, in the gyro-only replay, the
// gyro axis with the largest integral over the rotation, after the calibration (Control::NoCalibration replays without it).

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
#include <utility>
#include <vector>

#include "marv/estimator/gyro_cal.hpp"
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
  estimator::GyroCalConfig<float> gyro_cal;  // gyro_cal_duration_us and gyro_cal_movement_max
  double share = 0.0;           // attitude_tilt_error_max, rad
  std::uint32_t tick_num = 0;   // tick period num / den us
  std::uint32_t tick_den = 1;
  std::uint32_t tol_us = 0;     // rate_period_tolerance_us
  double trim_s = 0.0;          // dropped from each end of each merged still segment (test-harness scenario value)
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

// quad_fc's tick() on one sample: the boot gyro calibration, then the estimator step. accel_on false: no correction.
class Replayer {
 public:
  explicit Replayer(const Params& p, bool calibrate = true) : calibrate_(calibrate) {
    est_.init(p.mahony);
    cal_.init(p.gyro_cal);
  }
  // From now on the calibrated gyro's axis is negated (the control of check (c')).
  void negate_gyro_axis(std::size_t axis) { negate_axis_ = static_cast<int>(axis); }
  void step(const ImuSample& raw, bool accel_on = true) {
    ImuSample s = raw;
    if (calibrate_) {
      cal_.update(raw.gyro_rad_s, (raw.flags & imu_flag(ImuFlag::GyroValid)) != 0, raw.t_us);
      if (cal_.complete()) {
        if (!done_) {
          done_ = true;
          done_us_ = raw.t_us;
        }
        s.gyro_rad_s = raw.gyro_rad_s - cal_.bias();
      }
    }
    if (negate_axis_ >= 0) {
      s.gyro_rad_s[static_cast<std::size_t>(negate_axis_)] = -s.gyro_rad_s[static_cast<std::size_t>(negate_axis_)];
    }
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
  [[nodiscard]] bool calibration_complete() const { return done_; }
  [[nodiscard]] std::uint64_t calibration_done_us() const { return done_us_; }  // the completing sample's stamp
  [[nodiscard]] const prim::Vec3<float>& calibration_bias() const { return cal_.bias(); }

 private:
  estimator::Mahony<float> est_;
  estimator::GyroCal<float> cal_;
  bool calibrate_ = true;
  bool done_ = false;
  std::uint64_t done_us_ = 0;
  int negate_axis_ = -1;
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

[[nodiscard]] inline double seconds(const ImuSample& a, const ImuSample& b) {
  return static_cast<double>(b.t_us - a.t_us) / prim::kMicrosecondsPerSecond;
}

// The still runs (maximal runs of consecutive still samples), in time order.
[[nodiscard]] inline std::vector<Segment> still_runs(std::span<const ImuSample> s, const Params& p) {
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
  return runs;
}

// The rotation over the samples first .. last (inclusive): the sum of |gyro| times the stamp difference to the previous sample.
[[nodiscard]] inline double rotation_over(std::span<const ImuSample> s, std::size_t first, std::size_t last) {
  double rot = 0.0;
  for (std::size_t k = std::max<std::size_t>(first, 1); k <= last; ++k) {
    double n2 = 0.0;
    for (std::size_t a = 0; a < prim::kSpatialDim; ++a) {
      n2 += static_cast<double>(s[k].gyro_rad_s[a]) * static_cast<double>(s[k].gyro_rad_s[a]);
    }
    rot += std::sqrt(n2) * seconds(s[k - 1], s[k]);
  }
  return rot;
}

// Runs are merged while the rotation over the segment's gaps, summed, stays at most max_rotation_rad.
[[nodiscard]] inline std::vector<Segment> merge_runs(std::span<const ImuSample> s, const std::vector<Segment>& runs,
                                                     double max_rotation_rad) {
  std::vector<Segment> out;
  double used = 0.0;
  for (const Segment& r : runs) {
    const double gap = out.empty() ? 0.0 : rotation_over(s, out.back().last + 1, r.first);
    if (!out.empty() && used + gap <= max_rotation_rad) {
      out.back().last = r.last;
      used += gap;
    } else {
      out.push_back(r);
      used = 0.0;
    }
  }
  return out;
}

// The two longest segments (by sample count), in time order; fewer if there are fewer. Not trimmed.
[[nodiscard]] inline std::vector<Segment> two_longest(std::vector<Segment> runs) {
  std::stable_sort(runs.begin(), runs.end(), [](const Segment& a, const Segment& b) {
    return a.last - a.first > b.last - b.first;
  });
  if (runs.size() > 2) {
    runs.resize(2);
  }
  std::sort(runs.begin(), runs.end(), [](const Segment& a, const Segment& b) { return a.first < b.first; });
  return runs;
}

// The segment without the first and last trim_s seconds; false if nothing is left.
[[nodiscard]] inline bool trimmed(std::span<const ImuSample> s, const Segment& in, double trim_s, Segment& out) {
  const double trim_us = trim_s * prim::kMicrosecondsPerSecond;
  std::size_t a = in.first;
  std::size_t b = in.last;
  while (a <= in.last && static_cast<double>(s[a].t_us - s[in.first].t_us) < trim_us) {
    ++a;
  }
  while (b >= in.first && b > a && static_cast<double>(s[in.last].t_us - s[b].t_us) < trim_us) {
    --b;
  }
  if (a > in.last || b < a) {
    return false;
  }
  out = {a, b};
  return true;
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

enum class Control : std::uint8_t { None, NegateStill2Accel, NegateRotationGyro, NoCalibration };

struct Report {
  std::string invalid;  // empty: the log is a valid T2 log
  Segment still1;  // trimmed
  Segment still2;
  Segment raw1;    // merged, untrimmed
  Segment raw2;
  double theta_rot = 0.0;      // rad
  double rotation_s = 0.0;     // end of still 1 to start of still 2
  double ts_required_s = 0.0;  // ln(theta_rot / share) / Kp
  double still1_s = 0.0;
  double still2_s = 0.0;
  double err_a1 = 0.0;  // rad
  double err_a2 = 0.0;
  double err_c = 0.0;    // rad
  double bound_c = 0.0;  // theta_rot / 2
  bool calibrated = false;     // the replay applied the boot gyro calibration and it completed
  double cal_done_s = 0.0;     // from the log's first sample to the completing sample
  Dir cal_bias_rad_s{};        // the calibration's bias, rad/s
  bool pass_a = false;
  bool pass_c = false;
  [[nodiscard]] bool valid() const { return invalid.empty(); }
};

// segments: the two still segments to use; empty: the log's own (still runs merged by the gap rotation, the two longest).
[[nodiscard]] inline Report evaluate(std::span<const ImuSample> samples, const Params& p, Control control = Control::None,
                                     std::vector<Segment> segments = {}) {
  Report r;
  const std::vector<Segment> segs =
      segments.empty() ? two_longest(merge_runs(samples, still_runs(samples, p), p.share)) : std::move(segments);
  if (segs.size() < 2) {
    r.invalid = "fewer than two still segments";
    return r;
  }
  r.raw1 = segs[0];
  r.raw2 = segs[1];
  if (!trimmed(samples, r.raw1, p.trim_s, r.still1) || !trimmed(samples, r.raw2, p.trim_s, r.still2)) {
    r.invalid = "a still segment is empty after the trim";
    return r;
  }
  const Dir f1 = mean_accel(samples, r.still1);
  const Dir f2 = mean_accel(samples, r.still2);
  r.theta_rot = angle_between(f1, f2);
  r.rotation_s = seconds(samples[r.raw1.last], samples[r.raw2.first]);
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

  const bool calibrate = control != Control::NoCalibration;
  Replayer main(p, calibrate);
  std::optional<Replayer> at_end1;
  bool ok_a = true;
  for (std::size_t k = 0; k <= r.still2.last; ++k) {
    ImuSample s = samples[k];
    if (control == Control::NegateStill2Accel && k >= r.still2.first) {
      s.accel_m_s2[axis_a] = -s.accel_m_s2[axis_a];
    }
    main.step(s);
    if (k == r.still1.last) {
      if (calibrate) {
        if (!main.calibration_complete()) {
          r.invalid = "the boot gyro calibration did not complete within still segment 1: the log does not start with the board still";
          return r;
        }
        r.calibrated = true;
        r.cal_done_s = static_cast<double>(main.calibration_done_us() - samples[0].t_us) / prim::kMicrosecondsPerSecond;
        for (std::size_t a = 0; a < prim::kSpatialDim; ++a) {
          r.cal_bias_rad_s[a] = static_cast<double>(main.calibration_bias()[a]);
        }
      }
      at_end1 = main;
      r.err_a1 = angle_between(down_body(main.estimator()), unit({-f1[0], -f1[1], -f1[2]}));
      ok_a = ok_a && main.estimator().valid();
    }
  }
  r.err_a2 = angle_between(down_body(main.estimator()), unit({-f2[0], -f2[1], -f2[2]}));
  ok_a = ok_a && main.estimator().valid();
  r.pass_a = ok_a && r.err_a1 <= p.share && r.err_a2 <= p.share;

  Replayer gyro_only = *at_end1;
  if (control == Control::NegateRotationGyro) {
    gyro_only.negate_gyro_axis(axis_c);
  }
  for (std::size_t k = r.still1.last + 1; k <= r.still2.last; ++k) {
    gyro_only.step(samples[k], false);
  }
  r.bound_c = r.theta_rot / 2.0;
  r.err_c = angle_between(down_body(gyro_only.estimator()), unit({-f2[0], -f2[1], -f2[2]}));
  r.pass_c = gyro_only.estimator().valid() && r.err_c <= r.bound_c;
  return r;
}

}  // namespace marv::l07
