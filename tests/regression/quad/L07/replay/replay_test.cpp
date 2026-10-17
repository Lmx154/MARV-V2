// The T2 bench-log replay (M3 P3b, decision 0042 item 2) on a synthetic known-answer log and on rejected logs. The synthetic log
// is generated here from exact quantities and encoded through imu_log_format.hpp, so the parser path is exercised: still level
// for T_s, a rotation of 100 degrees (5 pi / 9, see kSynthRotationRad) about body x at a constant rate, still for T_s, at the board's tick period 1953125 / 14649 us
// (from the parameter set). The real bench log is one more entry of kRealLogs.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <span>
#include <string>
#include <vector>

#include <marv/params/param.hpp>

#include "replay.hpp"

namespace marv::l07 {
namespace {

// Committed bench logs under tests/regression/quad/L07/results/, one file name per entry. The test replays each and requires
// a valid T2 log, (a) and (c') to pass and both controls to fail.
const std::vector<std::string> kRealLogs = {};

// The synthetic log's framing, not a flight quantity: samples per data record, data records between status records.
constexpr std::size_t kSamplesPerRecord = 32;
constexpr std::size_t kRecordsPerStatus = 64;
constexpr std::uint64_t kFirstStampUs = 1000000;
// Rotation of the synthetic log (protocol: within 10 s) and the margin added to the required T_s.
constexpr double kSynthRotationS = 4.0;
// The synthetic rotation, 100 degrees about body x: beyond pi/2 on purpose. At exactly pi/2 still 2's specific force lies on
// one body axis, and the control that negates that axis hands the estimator the antipode of its own direction, where the
// filter's cross-product error is zero: it would stay there on an exact log and the control would not fail. 5 pi / 9 is the
// protocol's ">= pi/2" with the axis not aligned (a noisy real log leaves the antipode on its own).
constexpr double kSynthRotationRad = 5.0 * prim::kPi / 9.0;
constexpr double kSynthMarginS = 1.0;
// The slow start of the rotation of the ramp case: below the 15 deg/s still gate (Betaflight 4.5.1), so the segment's last
// samples before the main rotation pass the per-sample gate, and long enough (0.3 s) to tilt the estimate by 3 degrees.
constexpr double kSynthRampDegPerS = 10.0;
constexpr double kSynthRampS = 0.3;
// The gentle start of the rotation, as a bench dry run showed it (gyro 5 to 28 deg/s, dipping below the still gate several times):
// pieces that alternate 5 deg/s (inside the gate) and 25 deg/s (outside it). A piece is 49.5 ms (the lead's "50 ms pieces", a
// little shorter so the whole gap, 373 sample spacings = 49.7 ms, stays within the replaced fixed 50 ms merge rule, which the
// test keeps only as a control), and 30 pieces make 1.5 s, longer than twice the trim, so that the old rule's still 1 ends well
// inside the lead-in after the trim. A fast piece turns the board by 25 deg/s x 49.7 ms = 0.0217 rad, above the share.
constexpr double kGentleSlowDegPerS = 5.0;
constexpr double kGentleFastDegPerS = 25.0;
constexpr double kGentlePieceS = 0.0495;
constexpr std::size_t kGentlePieces = 30;
// The replaced still-segment merge rule (runs with a non-still gap of at most 50 ms were one segment), kept only as a control.
constexpr double kReplacedMergeGapS = 0.05;
// Test-harness scenario value for the still-segment trim (the lead's decision on the M3 P3b review; rationale in replay.hpp): 0.5 s
// is dropped from each end of each segment (the slow start and the settle of the hand rotation).
constexpr double kTrimS = 0.5;
// Known-answer tolerance of the synthetic log: float32 rounding accumulated over the samples; the measured errors are far
// below it and far below the share.
constexpr double kKatTolRad = 1.0e-4;
// A constant gyro bias of the board's measured size, 0.35 deg/s perpendicular to gravity: the FC3v2 at rest read
// (-0.175, 0.307, -0.10) deg/s (M3 P6 bench measurement). Test-harness scenario value.
constexpr std::array<double, prim::kSpatialDim> kBoardGyroBiasDegPerS = {-0.175, 0.307, -0.10};
// Shaking of the first still segment of the invalid-calibration case: the gyro y axis alternates +-10 deg/s sample by sample, inside
// the 15 deg/s still gate and above the calibration's movement limit (3.36 deg/s standard deviation).
constexpr double kShakeDegPerS = 10.0;

Params read_params() {
  static const bool ok = params_init(param_defaults());
  EXPECT_TRUE(ok);
  Params p;
  p.mahony = estimator::MahonyConfig<float>{param_value<ParamId::estimator_kp>(), param_value<ParamId::estimator_ki>(),
                                            param_value<ParamId::estimator_accel_healthy_min>(),
                                            param_value<ParamId::estimator_accel_healthy_max>()};
  p.gyro_cal = estimator::GyroCalConfig<float>{static_cast<std::uint64_t>(param_value<ParamId::gyro_cal_duration_us>()),
                                               param_value<ParamId::gyro_cal_movement_max>()};
  p.share = static_cast<double>(param_value<ParamId::attitude_tilt_error_max>());
  p.tick_num = static_cast<std::uint32_t>(param_value<ParamId::tick_period_num_us>());
  p.tick_den = static_cast<std::uint32_t>(param_value<ParamId::tick_period_den>());
  p.tol_us = static_cast<std::uint32_t>(param_value<ParamId::rate_period_tolerance_us>());
  p.trim_s = kTrimS;
  return p;
}

struct SynthSpec {
  double rotation_rad = kSynthRotationRad;
  double rotation_s = kSynthRotationS;
  double still_s = 0.0;  // 0: the required T_s plus the margin plus the trim at both ends
  double ramp_s = 0.0;   // a slow start of the rotation (kSynthRampDegPerS, inside the still gate) before the main rotation
  bool spikes = false;   // single-sample outliers inside each still segment
  bool gentle_start = false;  // a lead-in that dips below the still gate many times (kGentlePieces pieces)
  std::array<double, prim::kSpatialDim> gyro_bias_deg_s{};  // added to every sample's gyro (the sensor's constant bias)
  bool shaky_still1 = false;  // gyro y alternates +-kShakeDegPerS over the whole of still 1: the calibration cannot complete
};

double required_ts(double theta, const Params& p) { return std::log(theta / p.share) / static_cast<double>(p.mahony.kp); }

std::uint64_t stamp(std::size_t k, const Params& p) { return kFirstStampUs + (k * p.tick_num) / p.tick_den; }

std::size_t samples_for(double seconds, const Params& p) {
  return static_cast<std::size_t>(std::ceil(seconds * prim::kMicrosecondsPerSecond * p.tick_den / p.tick_num));
}

// Level still, [a slow lead-in,] a constant-rate rotation about body x, level-rotated still. Specific force f = R^T (0, 0, -g)
// with R = Rx(phi): (0, -g sin phi, -g cos phi); the gyro reads (rate, 0, 0) with rate = the lead-in's rate on the lead-in
// samples, omega on the rotation samples, 0 elsewhere. phi accumulates rate x (stamp difference) sample by sample, as the
// estimator integrates, with omega = (rotation_rad - lead-in rotation) / (rotation interval), so the estimator covers exactly
// rotation_rad. Lead-ins: ramp_s, a constant kSynthRampDegPerS; gentle_start, kGentlePieces pieces of kGentlePieceS that
// alternate kGentleSlowDegPerS (inside the still gate) and kGentleFastDegPerS (outside it), the slow piece first.
// spikes: in each still segment one sample with a specific-force outlier (0.7 g added on body x: |f| about 1.22 g, outside the
// healthy window) and, a thousand samples later, one sample with a gyro spike (twice the still gate, body y).
std::vector<ImuSample> synth_samples(const SynthSpec& spec, const Params& p) {
  const double still_s = spec.still_s > 0.0
                             ? spec.still_s
                             : std::ceil(required_ts(spec.rotation_rad, p)) + kSynthMarginS + 2.0 * p.trim_s;
  const std::size_t n_still = samples_for(still_s, p);
  const std::size_t piece = samples_for(kGentlePieceS, p);
  const std::size_t n_lead = spec.gentle_start ? kGentlePieces * piece : spec.ramp_s > 0.0 ? samples_for(spec.ramp_s, p) : 0;
  const std::size_t n_rot = samples_for(spec.rotation_s, p);
  const std::size_t total = 2 * n_still + n_lead + n_rot;
  const auto deg = [](double d) { return d * prim::kPi / prim::kDegreesPerHalfTurn; };
  const auto lead_rate = [&](std::size_t i) {
    if (spec.gentle_start) {
      return deg((i / piece) % 2 == 0 ? kGentleSlowDegPerS : kGentleFastDegPerS);
    }
    return deg(kSynthRampDegPerS);
  };
  const double g = prim::kStandardGravity;
  const float spike_gyro = static_cast<float>(2.0 * kStillGyroRadS);
  constexpr std::size_t kSpikeSpacing = 1000;
  constexpr std::uint32_t kValid = imu_flag(ImuFlag::GyroValid) | imu_flag(ImuFlag::AccelValid);
  const auto dt_s = [&](std::size_t k) {
    return k == 0 ? 0.0 : static_cast<double>(stamp(k, p) - stamp(k - 1, p)) / prim::kMicrosecondsPerSecond;
  };
  double lead_rad = 0.0;
  double rot_s = 0.0;
  for (std::size_t i = 0; i < n_lead; ++i) {
    lead_rad += lead_rate(i) * dt_s(n_still + i);
  }
  for (std::size_t i = 0; i < n_rot; ++i) {
    rot_s += dt_s(n_still + n_lead + i);
  }
  const double omega = (spec.rotation_rad - lead_rad) / rot_s;
  std::vector<ImuSample> out(total);
  double phi = 0.0;
  for (std::size_t k = 0; k < total; ++k) {
    double rate = 0.0;
    if (k >= n_still && k < n_still + n_lead) {
      rate = lead_rate(k - n_still);
    } else if (k >= n_still + n_lead && k < n_still + n_lead + n_rot) {
      rate = omega;
    }
    phi += rate * dt_s(k);
    ImuSample& s = out[k];
    s.t_us = stamp(k, p);
    double shake = 0.0;
    if (spec.shaky_still1 && k < n_still) {
      shake = (k % 2 == 0 ? 1.0 : -1.0) * deg(kShakeDegPerS);
    }
    s.gyro_rad_s = {static_cast<float>(rate + deg(spec.gyro_bias_deg_s[0])), static_cast<float>(shake + deg(spec.gyro_bias_deg_s[1])),
                    static_cast<float>(deg(spec.gyro_bias_deg_s[2]))};
    s.accel_m_s2 = {0.0F, static_cast<float>(-g * std::sin(phi)), static_cast<float>(-g * std::cos(phi))};
    s.temp_k = 0.0F;
    s.flags = kValid;
  }
  if (spec.spikes) {
    for (const std::size_t base : {n_still / 2, n_still + n_lead + n_rot + n_still / 2}) {
      out[base].accel_m_s2[0] += static_cast<float>(0.7 * g);
      out[base + kSpikeSpacing].gyro_rad_s[1] = spike_gyro;
    }
  }
  return out;
}

struct Fault {
  long drop_data_record = -1;   // omit this data record (sequence gap)
  long flip_in_record = -1;     // flip one bit in this data record (CRC failure)
  int counter = -1;             // StatusField index to set in every status record
  std::uint32_t counter_value = 0;
  bool no_status = false;
  std::size_t leading = 0;      // garbage bytes before the first record
  std::size_t trailing = 0;     // bytes of an incomplete record after the last
};

std::vector<std::uint8_t> encode(std::span<const ImuSample> samples, const Fault& fault = {}) {
  std::vector<std::uint8_t> out(fault.leading, std::uint8_t{0x55});
  std::vector<std::uint8_t> buf(fmt::data_record_bytes(kSamplesPerRecord));
  std::uint32_t seq = 0;
  long rec = 0;
  const auto status = [&] {
    std::array<std::uint8_t, fmt::kStatusRecordBytes> sb{};
    fmt::RecordWriter w(sb);
    w.begin(fmt::kStatusMagic, seq++, static_cast<std::uint32_t>(fmt::kStatusFields));
    for (int f = 0; f < static_cast<int>(fmt::kStatusFields); ++f) {
      w.put_u32(f == fault.counter ? fault.counter_value : 0U);
    }
    const std::size_t n = w.end();
    out.insert(out.end(), sb.begin(), sb.begin() + static_cast<std::ptrdiff_t>(n));
  };
  for (std::size_t i = 0; i < samples.size(); i += kSamplesPerRecord, ++rec) {
    const std::size_t c = std::min(kSamplesPerRecord, samples.size() - i);
    fmt::RecordWriter w(buf);
    w.begin(fmt::kDataMagic, seq, static_cast<std::uint32_t>(c));
    for (std::size_t k = 0; k < c; ++k) {
      w.put_sample(samples[i + k]);
    }
    const std::size_t n = w.end();
    ++seq;
    if (rec == fault.drop_data_record) {
      continue;
    }
    if (rec == fault.flip_in_record) {
      buf[fmt::kHeaderBytes + 3] ^= 0x01;
    }
    out.insert(out.end(), buf.begin(), buf.begin() + static_cast<std::ptrdiff_t>(n));
    if (!fault.no_status && static_cast<std::size_t>(rec + 1) % kRecordsPerStatus == 0) {
      status();
    }
  }
  if (!fault.no_status) {
    status();
  }
  for (std::size_t i = 0; i < fault.trailing; ++i) {
    out.push_back(fmt::kDataMagic[i % fmt::kMagicBytes]);
  }
  return out;
}

// The parser tests need records, not a whole log: the first kParserTestSamples samples of the synthetic log.
constexpr std::size_t kParserTestSamples = 4096;
std::vector<ImuSample> head_samples(const Params& p) {
  std::vector<ImuSample> s = synth_samples({}, p);
  s.resize(kParserTestSamples);
  return s;
}

void print(const char* name, const Report& r, const Params& p) {
  if (r.calibrated) {
    std::printf("%s: calibration completed %.3f s after the first sample, bias (%.4f, %.4f, %.4f) deg/s\n", name, r.cal_done_s,
                r.cal_bias_rad_s[0] * prim::kDegreesPerHalfTurn / prim::kPi, r.cal_bias_rad_s[1] * prim::kDegreesPerHalfTurn / prim::kPi,
                r.cal_bias_rad_s[2] * prim::kDegreesPerHalfTurn / prim::kPi);
  } else {
    std::printf("%s: no calibration applied\n", name);
  }
  std::printf(
      "%s: theta_rot %.6f rad, T_s required %.3f s (kp %.4f, share %.5f rad), still1 %.3f s, still2 %.3f s, rotation %.3f s\n"
      "  (a) err1 %.3e err2 %.3e rad (<= %.5f): %s;  (c') err %.3e rad (<= theta_rot/2 = %.5f): %s\n",
      name, r.theta_rot, r.ts_required_s, static_cast<double>(p.mahony.kp), p.share, r.still1_s, r.still2_s, r.rotation_s,
      r.err_a1, r.err_a2, p.share, r.pass_a ? "pass" : "FAIL", r.err_c, r.bound_c, r.pass_c ? "pass" : "FAIL");
}

TEST(L07Replay, SyntheticKnownAnswerPassesBothChecks) {
  const Params p = read_params();
  const std::vector<ImuSample> samples = synth_samples({}, p);
  const std::vector<std::uint8_t> bytes = encode(samples);
  const ParsedLog log = parse_log(bytes, p);
  ASSERT_TRUE(log.ok()) << log.error;
  ASSERT_EQ(log.samples.size(), samples.size());
  EXPECT_EQ(log.dropped_tail_bytes, 0U);
  const Report r = evaluate(log.samples, p);
  print("synthetic", r, p);
  ASSERT_TRUE(r.valid()) << r.invalid;
  EXPECT_NEAR(r.theta_rot, kSynthRotationRad, kKatTolRad);
  EXPECT_GE(r.still1_s, r.ts_required_s);
  EXPECT_GE(r.still2_s, r.ts_required_s);
  EXPECT_TRUE(r.pass_a);
  EXPECT_TRUE(r.pass_c);
  EXPECT_LT(r.err_a1, kKatTolRad);
  EXPECT_LT(r.err_a2, kKatTolRad);
  EXPECT_LT(r.err_c, kKatTolRad);
}

TEST(L07Replay, ABoardSizedGyroBiasIsCalibratedOutAndWithoutTheCalibrationFailsCheckA) {
  const Params p = read_params();
  SynthSpec spec;
  spec.gyro_bias_deg_s = kBoardGyroBiasDegPerS;
  const std::vector<ImuSample> samples = synth_samples(spec, p);
  const Report r = evaluate(samples, p);
  print("biased", r, p);
  ASSERT_TRUE(r.valid()) << r.invalid;
  ASSERT_TRUE(r.calibrated);
  for (std::size_t a = 0; a < prim::kSpatialDim; ++a) {
    EXPECT_NEAR(r.cal_bias_rad_s[a], spec.gyro_bias_deg_s[a] * prim::kPi / prim::kDegreesPerHalfTurn, kKatTolRad);
  }
  EXPECT_NEAR(r.theta_rot, kSynthRotationRad, kKatTolRad);
  EXPECT_TRUE(r.pass_a);
  EXPECT_TRUE(r.pass_c);
  // The control: the same log replayed without the calibration, the bias tilts the estimate by about bias / Kp.
  const Report off = evaluate(samples, p, Control::NoCalibration);
  print("biased, no calibration", off, p);
  ASSERT_TRUE(off.valid()) << off.invalid;
  EXPECT_FALSE(off.pass_a);
  EXPECT_GT(off.err_a1, p.share);
}

TEST(L07Replay, ALogWhoseCalibrationCannotCompleteInStill1IsNotAValidT2Log) {
  const Params p = read_params();
  SynthSpec spec;
  spec.shaky_still1 = true;
  const Report r = evaluate(synth_samples(spec, p), p);
  EXPECT_FALSE(r.valid());
  EXPECT_NE(r.invalid.find("calibration"), std::string::npos) << r.invalid;
  // The same log without the calibration is a valid T2 log: only the calibration rejects it.
  EXPECT_TRUE(evaluate(synth_samples(spec, p), p, Control::NoCalibration).valid());
}

TEST(L07Replay, ControlStill2AccelAxisNegatedFailsCheckA) {
  const Params p = read_params();
  const std::vector<ImuSample> samples = synth_samples({}, p);
  const Report r = evaluate(samples, p, Control::NegateStill2Accel);
  print("control (a)", r, p);
  ASSERT_TRUE(r.valid()) << r.invalid;
  EXPECT_FALSE(r.pass_a);
  EXPECT_GT(r.err_a2, p.share);
  EXPECT_TRUE(evaluate(samples, p).pass_a) << "the control differs from the clean run only by the negation";
}

TEST(L07Replay, ControlRotationGyroAxisNegatedFailsCheckC) {
  const Params p = read_params();
  const std::vector<ImuSample> samples = synth_samples({}, p);
  const Report r = evaluate(samples, p, Control::NegateRotationGyro);
  print("control (c')", r, p);
  ASSERT_TRUE(r.valid()) << r.invalid;
  EXPECT_FALSE(r.pass_c);
  EXPECT_GT(r.err_c, r.bound_c);
  EXPECT_TRUE(evaluate(samples, p).pass_c);
}

TEST(L07Replay, SingleSampleSpikesInsideTheStillSegmentsAreMerged) {
  const Params p = read_params();
  SynthSpec spec;
  spec.spikes = true;
  const std::vector<ImuSample> samples = synth_samples(spec, p);
  const Report r = evaluate(samples, p);
  print("spikes", r, p);
  ASSERT_TRUE(r.valid()) << r.invalid;
  EXPECT_NEAR(r.theta_rot, kSynthRotationRad, kKatTolRad);
  EXPECT_TRUE(r.pass_a);
  EXPECT_TRUE(r.pass_c);
  // The control: without the merge each spike splits its segment, and the two longest pieces are not the two still segments.
  const Report split = evaluate(samples, p, Control::None, two_longest(still_runs(samples, p)));
  std::printf("spikes, no merge: %s\n", split.valid() ? "valid" : split.invalid.c_str());
  EXPECT_FALSE(split.valid() && split.pass_a && split.pass_c);
}

// The replaced rule: runs with a non-still gap of at most kReplacedMergeGapS (time from the last still sample to the next).
std::vector<Segment> replaced_rule_segments(std::span<const ImuSample> s, const Params& p) {
  std::vector<Segment> out;
  for (const Segment& r : still_runs(s, p)) {
    if (!out.empty() && static_cast<double>(s[r.first].t_us - s[out.back().last].t_us) <= kReplacedMergeGapS * 1.0e6) {
      out.back().last = r.last;
    } else {
      out.push_back(r);
    }
  }
  return two_longest(out);
}

TEST(L07Replay, AGentleStartThatDipsBelowTheStillGateIsNotMergedIntoStill1) {
  const Params p = read_params();
  SynthSpec spec;
  spec.gentle_start = true;
  const std::vector<ImuSample> samples = synth_samples(spec, p);
  const Report r = evaluate(samples, p);
  print("gentle start", r, p);
  ASSERT_TRUE(r.valid()) << r.invalid;
  EXPECT_NEAR(r.theta_rot, kSynthRotationRad, kKatTolRad);
  EXPECT_TRUE(r.pass_a);
  EXPECT_TRUE(r.pass_c);
  EXPECT_LT(r.err_a1, kKatTolRad);
  EXPECT_LT(r.err_a2, kKatTolRad);
  // The control: the replaced 50 ms rule merges the dips into still 1, whose trimmed end then lies inside the rotation.
  const std::vector<Segment> old_segs = replaced_rule_segments(samples, p);
  ASSERT_EQ(old_segs.size(), 2U);
  EXPECT_GT(old_segs[0].last, r.raw1.last) << "the old rule must have merged some of the lead-in into still 1";
  const Report old = evaluate(samples, p, Control::None, old_segs);
  print("gentle start, replaced 50 ms rule", old, p);
  ASSERT_TRUE(old.valid()) << old.invalid;
  EXPECT_FALSE(old.pass_a);
  EXPECT_GT(old.err_a1, p.share);
}

TEST(L07Replay, ASlowStartOfTheRotationInsideTheStillGateIsTrimmedOff) {
  const Params p = read_params();
  SynthSpec spec;
  spec.ramp_s = kSynthRampS;
  const std::vector<ImuSample> samples = synth_samples(spec, p);
  const Report r = evaluate(samples, p);
  print("slow start", r, p);
  ASSERT_TRUE(r.valid()) << r.invalid;
  EXPECT_NEAR(r.theta_rot, kSynthRotationRad, kKatTolRad);
  EXPECT_TRUE(r.pass_a);
  EXPECT_TRUE(r.pass_c);
  // The control: untrimmed, still 1 ends 0.3 s into the ramp, where the estimate (which follows the exact gyro) is tilted
  // 3 degrees against the segment's mean specific force.
  Params no_trim = p;
  no_trim.trim_s = 0.0;
  const Report raw = evaluate(samples, no_trim);
  print("slow start, no trim", raw, no_trim);
  ASSERT_TRUE(raw.valid()) << raw.invalid;
  EXPECT_FALSE(raw.pass_a);
  EXPECT_GT(raw.err_a1, p.share);
}

TEST(L07Replay, AStillSegmentShorterThanTwiceTheTrimIsNotAValidT2Log) {
  const Params p = read_params();
  SynthSpec spec;
  spec.still_s = 2.0 * kTrimS - 0.1;
  EXPECT_FALSE(evaluate(synth_samples(spec, p), p).valid());
}

TEST(L07Replay, LeadingBytesAndAnIncompleteTailAreTolerated) {
  const Params p = read_params();
  const std::vector<ImuSample> samples = head_samples(p);
  Fault f;
  f.leading = 37;
  f.trailing = 3;
  const ParsedLog log = parse_log(encode(samples, f), p);
  ASSERT_TRUE(log.ok()) << log.error;
  EXPECT_EQ(log.leading_bytes, 37U);
  EXPECT_EQ(log.dropped_tail_bytes, 3U);
  EXPECT_EQ(log.samples.size(), samples.size());
}

TEST(L07Replay, RejectsASequenceGap) {
  const Params p = read_params();
  Fault f;
  f.drop_data_record = 10;
  const ParsedLog log = parse_log(encode(head_samples(p), f), p);
  EXPECT_FALSE(log.ok());
  EXPECT_NE(log.error.find("sequence gap"), std::string::npos) << log.error;
}

TEST(L07Replay, RejectsACrcFailure) {
  const Params p = read_params();
  Fault f;
  f.flip_in_record = 10;
  const ParsedLog log = parse_log(encode(head_samples(p), f), p);
  EXPECT_FALSE(log.ok());
  EXPECT_NE(log.error.find("CRC"), std::string::npos) << log.error;
}

TEST(L07Replay, RejectsEveryNonzeroCleanlinessCounter) {
  const Params p = read_params();
  const std::vector<ImuSample> samples = head_samples(p);
  for (const auto& z : detail::kMustBeZero) {
    Fault f;
    f.counter = static_cast<int>(z.field);
    f.counter_value = 1;
    const ParsedLog log = parse_log(encode(samples, f), p);
    EXPECT_FALSE(log.ok()) << z.name;
    EXPECT_NE(log.error.find(z.name), std::string::npos) << log.error;
  }
  // The informational counters do not reject.
  for (const fmt::StatusField field : {fmt::StatusField::RingHighWater, fmt::StatusField::Delivered, fmt::StatusField::Logged}) {
    Fault f;
    f.counter = static_cast<int>(field);
    f.counter_value = 5;
    EXPECT_TRUE(parse_log(encode(samples, f), p).ok());
  }
}

TEST(L07Replay, RejectsALogWithoutAStatusRecord) {
  const Params p = read_params();
  Fault f;
  f.no_status = true;
  EXPECT_FALSE(parse_log(encode(head_samples(p), f), p).ok());
}

TEST(L07Replay, RejectsAStampGapBeyondTheTolerance) {
  const Params p = read_params();
  std::vector<ImuSample> samples = head_samples(p);
  const std::uint64_t jump = static_cast<std::uint64_t>(p.tol_us) + 2;  // the spacing then differs by more than tol_us
  for (std::size_t k = 1000; k < samples.size(); ++k) {
    samples[k].t_us += jump;
  }
  const ParsedLog log = parse_log(encode(samples), p);
  EXPECT_FALSE(log.ok());
  EXPECT_NE(log.error.find("stamp gap"), std::string::npos) << log.error;
}

TEST(L07Replay, AStampSpacingWithinTheToleranceIsAccepted) {
  const Params p = read_params();
  std::vector<ImuSample> samples = head_samples(p);
  for (std::size_t k = 1000; k < samples.size(); ++k) {
    samples[k].t_us += 1;  // one extra us at one spacing: |dt - T| < 2 us <= tol_us
  }
  EXPECT_TRUE(parse_log(encode(samples), p).ok());
}

TEST(L07Replay, ALogWithAStillSegmentShorterThanTsIsNotAValidT2Log) {
  const Params p = read_params();
  SynthSpec spec;
  spec.still_s = std::floor(required_ts(kSynthRotationRad, p)) - 1.0;
  const Report r = evaluate(synth_samples(spec, p), p);
  EXPECT_FALSE(r.valid());
  EXPECT_NE(r.invalid.find("T_s"), std::string::npos) << r.invalid;
}

TEST(L07Replay, ARotationBelowHalfPiIsNotAValidT2Log) {
  const Params p = read_params();
  SynthSpec spec;
  spec.rotation_rad = prim::kPi / 4.0;
  spec.rotation_s = 2.0;  // 22.5 deg/s: above the still gate, so the rotation separates the two segments
  const Report r = evaluate(synth_samples(spec, p), p);
  EXPECT_FALSE(r.valid());
  EXPECT_NE(r.invalid.find("pi/2"), std::string::npos) << r.invalid;
}

TEST(L07Replay, ALogWithoutTwoStillSegmentsIsNotAValidT2Log) {
  const Params p = read_params();
  std::vector<ImuSample> samples = synth_samples({}, p);
  samples.resize(samples.size() / 4);  // still 1 only
  EXPECT_FALSE(evaluate(samples, p).valid());
}

std::vector<std::uint8_t> read_file(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

TEST(L07Replay, CommittedBenchLogs) {
  if (kRealLogs.empty()) {
    GTEST_SKIP() << "no bench log committed yet (kRealLogs is empty)";
  }
  const Params p = read_params();
  for (const std::string& name : kRealLogs) {
    SCOPED_TRACE(name);
    const std::vector<std::uint8_t> bytes = read_file(std::string(MARV_L07_RESULTS_DIR) + "/" + name);
    ASSERT_FALSE(bytes.empty());
    const ParsedLog log = parse_log(bytes, p);
    ASSERT_TRUE(log.ok()) << log.error;
    const Report r = evaluate(log.samples, p);
    print(name.c_str(), r, p);
    ASSERT_TRUE(r.valid()) << r.invalid;
    EXPECT_TRUE(r.pass_a);
    EXPECT_TRUE(r.pass_c);
    const Report ca = evaluate(log.samples, p, Control::NegateStill2Accel);
    print("control (a)", ca, p);
    EXPECT_FALSE(ca.pass_a);
    const Report cc = evaluate(log.samples, p, Control::NegateRotationGyro);
    print("control (c')", cc, p);
    EXPECT_FALSE(cc.pass_c);
  }
}

}  // namespace
}  // namespace marv::l07
