// The T2 bench-log replay (M3 P3b, decision 0042 item 2) on a synthetic known-answer log and on rejected logs. The synthetic log
// is generated here from exact quantities and encoded through imu_log_format.hpp, so the parser path is exercised: still level
// for T_s, a rotation of 100 degrees (5 pi / 9, see kSynthRotationRad) about body x at a constant rate, still for T_s, at the board's tick period 1953125 / 14649 us
// (from the parameter set). The real bench log is one more entry of kRealLogs.

#include <gtest/gtest.h>

#include <algorithm>
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
// Known-answer tolerance of the synthetic log: float32 rounding accumulated over the samples; the measured errors are far
// below it and far below the share.
constexpr double kKatTolRad = 1.0e-4;

Params read_params() {
  static const bool ok = params_init(param_defaults());
  EXPECT_TRUE(ok);
  Params p;
  p.mahony = estimator::MahonyConfig<float>{param_value<ParamId::estimator_kp>(), param_value<ParamId::estimator_ki>(),
                                            param_value<ParamId::estimator_accel_healthy_min>(),
                                            param_value<ParamId::estimator_accel_healthy_max>()};
  p.share = static_cast<double>(param_value<ParamId::attitude_tilt_error_max>());
  p.tick_num = static_cast<std::uint32_t>(param_value<ParamId::tick_period_num_us>());
  p.tick_den = static_cast<std::uint32_t>(param_value<ParamId::tick_period_den>());
  p.tol_us = static_cast<std::uint32_t>(param_value<ParamId::rate_period_tolerance_us>());
  return p;
}

struct SynthSpec {
  double rotation_rad = kSynthRotationRad;
  double rotation_s = kSynthRotationS;
  double still_s = 0.0;  // 0: the required T_s plus the margin
};

double required_ts(double theta, const Params& p) { return std::log(theta / p.share) / static_cast<double>(p.mahony.kp); }

std::uint64_t stamp(std::size_t k, const Params& p) { return kFirstStampUs + (k * p.tick_num) / p.tick_den; }

std::size_t samples_for(double seconds, const Params& p) {
  return static_cast<std::size_t>(std::ceil(seconds * prim::kMicrosecondsPerSecond * p.tick_den / p.tick_num));
}

// Level still, a constant-rate rotation about body x, level-rotated still. Specific force f = R^T (0, 0, -g) with
// R = Rx(phi): (0, -g sin phi, -g cos phi); the gyro reads (omega, 0, 0) on the rotation samples; phi(t) = omega (t - t_start)
// clamped to [0, rotation_rad], with omega = rotation_rad / (t_end - t_start) over the rotation samples' interval, so the
// estimator, integrating omega over the stamp differences, covers exactly rotation_rad.
std::vector<ImuSample> synth_samples(const SynthSpec& spec, const Params& p) {
  const double still_s = spec.still_s > 0.0 ? spec.still_s : std::ceil(required_ts(spec.rotation_rad, p)) + kSynthMarginS;
  const std::size_t n_still = samples_for(still_s, p);
  const std::size_t n_rot = samples_for(spec.rotation_s, p);
  const std::size_t total = 2 * n_still + n_rot;
  const double t_start = static_cast<double>(stamp(n_still - 1, p));
  const double t_end = static_cast<double>(stamp(n_still + n_rot - 1, p));
  const double omega = spec.rotation_rad / ((t_end - t_start) / prim::kMicrosecondsPerSecond);
  const double g = prim::kStandardGravity;
  constexpr std::uint32_t kValid = imu_flag(ImuFlag::GyroValid) | imu_flag(ImuFlag::AccelValid);
  std::vector<ImuSample> out(total);
  for (std::size_t k = 0; k < total; ++k) {
    const double t = static_cast<double>(stamp(k, p));
    const double phi = omega * (std::clamp(t, t_start, t_end) - t_start) / prim::kMicrosecondsPerSecond;
    const bool rotating = k >= n_still && k < n_still + n_rot;
    ImuSample& s = out[k];
    s.t_us = stamp(k, p);
    s.gyro_rad_s = {rotating ? static_cast<float>(omega) : 0.0F, 0.0F, 0.0F};
    s.accel_m_s2 = {0.0F, static_cast<float>(-g * std::sin(phi)), static_cast<float>(-g * std::cos(phi))};
    s.temp_k = 0.0F;
    s.flags = kValid;
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
