// sensor_replay: the SIL-2 replay of the Gazebo lockstep plugin's sensor path (decision 0019, W1 C2, test b; quad spec
// section 4 L6 pass bar (a), T1: "SIL-2 with noise ... The adapter's sensor bytes equal a direct marv_plant call").
// Host only, harness only. It links marv_plant and the generated IMU header (marv_imu_profile_config): no SIL library, no
// adapter, no Gazebo. The log layout is the plugin's (sim/gz/plugin/src/lockstep_log.hpp, whose record enum it shares);
// the sample layouts are those of marv_sil.h and marv_truth.h (headers only).
//
//   sensor_replay <plant> <log>
//
// <log>    a complete binary log of the plugin (a trailer whose counts are the records read) of a run with
//          <gyro_source>model</gyro_source>: record 6 SENSORS first, gyro_source 2.
// <plant>  text, one line per plant element of the run's world, with the element's text as the world gives it (the plugin
//          reads the same text with std::from_chars, so the values are the plugin's, bitwise):
//            <name> <value>                   esc_map, pole_count, mass_kg, thrust_coeff, torque_ratio_m, omega_min_rad_s,
//                                             omega_max_rad_s, motor_tau_s, site_lat_rad, site_height_m, motor_substep_s
//            rotor <motor> <x> <y> <z> <yaw>  one per logical motor 1..4: position_frd_m, then yaw_sign
//            initial_rotor_speed_rad_s <w1> <w2> <w3> <w4>   optional; absent, all 0 (as in the plugin)
//            rotor_speed_model <latency_ticks> <exponent_bits> <mantissa_bits> <period_unit_s>
//                                             the sensor profile's rotor-speed values (the profile's latency in rate periods
//                                             x rate_loop_divisor, and its telemetry grid), not the world's; required iff
//                                             record 6 has rotor_speed 1, otherwise optional and unused
//          Each line once; every name but the two optional ones is required. rng_seed is record 6's seed.
//
// The replay is a sequence of direct marv_plant calls. One plant from <plant> with rng_seed = record 6's seed; record 6's
// IMU configuration attached, then, with rotor_speed 1, its rotor-speed configuration (the adapter's order). For each host
// step, its STEP record's plant body (the body the plugin fed, held over the step) and, for each tick j of the step, in
// the adapter's order (sim/gz/adapter/include/marv/gz/adapter.hpp): marv_plant_imu_sample(body, t_true),
// marv_plant_rotor_speed_sample (with rotor_speed 1), then marv_plant_step(body, dshot_j, t_true) with dshot_j the TICK
// record's. t_true is derived from the header and record 6, not read from the log: host_step_ns / (m 1e9) with a clock
// element (the plugin's rule, lockstep.cpp start_sensors), num_us / (den 1e6) without one (tick_period_s, adapter.cpp).
//
// Compared bitwise, at every tick:
//   imu    the call's marv_plant_imu_out against the TICK record's 32 IMU bytes (the sample passed to the SIL);
//   rotor  the call's marv_plant_rotor_speed_out against the ROTOR record's 20 bytes (with rotor_speed 1);
//   plant  marv_plant_out's erpm_valid, force, torque, rotor speed and erpm against the TICK record's.
// And once, config: record 6's IMU configuration and turn-on bias bounds against the generated header's
// imu_corner_config(signs) and bounds; its odr_error against kImuOdrError with a clock element (0, and corner 0, without
// one); its tick_s against t_true; its stream id and counter base against the plugin's (constants.hpp); with rotor_speed 1
// its rotor-speed pole count against the plant's and its latency_ticks, exponent_bits, mantissa_bits and period_unit_s
// against the plant file's rotor_speed_model line.
//
// Output (stdout): one line per mismatch, "mismatch <kind> <tick> <what>" (kind config, imu, rotor or plant; the tick is
// "-" for config), then "ticks <n> config <c> imu <i> rotor <r> plant <p>". Exit 0 if every count is 0, 1 if one is not,
// 2 on a usage, input or log error or a refused plant call (one line on stderr).
#include <array>
#include <bit>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iterator>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <type_traits>
#include <vector>

#include "constants.hpp"
#include "lockstep_log.hpp"
#include "marv/gz/constants.hpp"
#include "marv/sim/imu_profile_config.hpp"
#include "marv_plant.h"
#include "marv_sil.h"
#include "marv_truth.h"

static_assert(std::endian::native == std::endian::little, "the lockstep log is little-endian and read natively");
static_assert(sizeof(marv_plant_imu_out) == sizeof(marv_imu_meas), "the plant's IMU output is the logged sample");
static_assert(sizeof(marv_plant_rotor_speed_out) == sizeof(marv_rotor_speed_meas),
              "the plant's rotor-speed output is the logged sample");

namespace {

using marv::gz::LogGyroSource;
using marv::gz::LogRecord;

constexpr std::size_t kMotors = MARV_PLANT_N_MOTORS;
constexpr std::size_t kAxes = std::extent_v<decltype(marv_plant_imu_axis_config::turn_on_bias)>;
constexpr std::size_t kQuaternion = std::extent_v<decltype(marv_plant_body::q_wxyz)>;
constexpr std::size_t kBiasSigns = 2 * kAxes;  // gyro x y z, then accel x y z (record 6 and imu_corner_config)

// The header's magic and version. Citation: sim/gz/plugin/src/lockstep_log.hpp (header: magic "MARVLOCK", version 1) and
// lockstep_log.cpp (kMagic, kVersion).
// Kind: interface.
constexpr std::string_view kMagic = "MARVLOCK";
constexpr std::uint32_t kVersion = 1;

// Doubles of a STEP record before the plant body, and of a TICK record after the plant outputs, and of an APPLIED record.
// Rule: lockstep_log.hpp (STEP: the raw gz reads, position 3, quaternion 4, linear velocity 3, angular velocity 3; TICK:
// the per-tick wrench in ENU, force 3 and torque 3; APPLIED: W_bar, force 3 and torque 3).
// Kind: derived.
constexpr std::size_t kStepGzDoubles = kAxes + kQuaternion + kAxes + kAxes;
constexpr std::size_t kWrenchDoubles = 2 * kAxes;

enum Exit : int { kExitEqual = 0, kExitMismatch = 1, kExitError = 2 };

using ImuBytes = std::array<std::uint8_t, sizeof(marv_imu_meas)>;
using RotorBytes = std::array<std::uint8_t, sizeof(marv_rotor_speed_meas)>;

struct Sensors {
  std::uint8_t gyro_source = 0;
  std::uint8_t rotor_speed = 0;
  std::uint8_t clock = 0;
  std::int8_t clock_corner = 0;
  double odr_error = 0.0;
  std::uint64_t host_step_ns = 0;
  double tick_s = 0.0;
  std::uint64_t seed = 0;
  std::uint32_t imu_stream_id = 0;
  std::uint64_t imu_counter_base = 0;
  marv_plant_imu_config imu{};
  double gyro_bound = 0.0;
  double accel_bound = 0.0;
  std::array<std::int8_t, kBiasSigns> signs{};
  marv_plant_rotor_speed_config rotor{};
};

struct Tick {
  std::uint64_t tick = 0;
  ImuBytes imu{};
  std::array<std::uint16_t, kMotors> dshot{};
  std::uint32_t erpm_valid = 0;
  std::array<double, kAxes> force{};
  std::array<double, kAxes> torque{};
  std::array<double, kMotors> rotor_speed{};
  std::array<double, kMotors> erpm{};
  std::optional<RotorBytes> rotor;
};

struct Step {
  marv_plant_body body{};
  std::vector<Tick> ticks;
};

struct Log {
  std::uint32_t m = 0;
  std::uint32_t num_us = 0;
  std::uint32_t den = 0;
  Sensors sensors;
  std::vector<Step> steps;
};

// Sequential little-endian reads of a byte buffer; a read past the end clears ok() and yields zeros.
class Reader {
 public:
  explicit Reader(const std::vector<std::uint8_t>& d) : d_(d) {}
  bool ok() const { return ok_; }
  std::size_t pos() const { return pos_; }
  bool at_end() const { return pos_ == d_.size(); }
  void bytes(void* dst, std::size_t n) {
    if (!ok_ || d_.size() - pos_ < n) {
      ok_ = false;
      std::memset(dst, 0, n);
      return;
    }
    std::memcpy(dst, d_.data() + pos_, n);
    pos_ += n;
  }
  template <typename T>
  T get() {
    T v{};
    bytes(&v, sizeof(v));
    return v;
  }
  void skip(std::size_t n) {
    if (!ok_ || d_.size() - pos_ < n) {
      ok_ = false;
      return;
    }
    pos_ += n;
  }

 private:
  const std::vector<std::uint8_t>& d_;
  std::size_t pos_ = 0;
  bool ok_ = true;
};

template <std::size_t N>
void get_doubles(Reader& r, std::array<double, N>& out) {
  for (double& v : out) {
    v = r.get<double>();
  }
}

void get_doubles(Reader& r, double* out, std::size_t n) {
  for (std::size_t i = 0; i < n; ++i) {
    out[i] = r.get<double>();
  }
}

void get_imu_axis(Reader& r, marv_plant_imu_axis_config& a) {
  a.noise_density = r.get<double>();
  a.bias_instability = r.get<double>();
  a.lsb = r.get<double>();
  a.full_scale = r.get<double>();
  get_doubles(r, a.turn_on_bias, kAxes);
}

// Record 6, in lockstep_log.hpp's field order.
Sensors get_sensors(Reader& r) {
  Sensors s;
  s.gyro_source = r.get<std::uint8_t>();
  s.rotor_speed = r.get<std::uint8_t>();
  s.clock = r.get<std::uint8_t>();
  s.clock_corner = r.get<std::int8_t>();
  s.odr_error = r.get<double>();
  s.host_step_ns = r.get<std::uint64_t>();
  s.tick_s = r.get<double>();
  s.seed = r.get<std::uint64_t>();
  s.imu_stream_id = r.get<std::uint32_t>();
  s.imu_counter_base = r.get<std::uint64_t>();
  s.imu.struct_size = sizeof(s.imu);
  s.imu.latency_samples = r.get<std::uint32_t>();
  get_imu_axis(r, s.imu.gyro);
  get_imu_axis(r, s.imu.accel);
  s.gyro_bound = r.get<double>();
  s.accel_bound = r.get<double>();
  for (std::int8_t& v : s.signs) {
    v = r.get<std::int8_t>();
  }
  r.skip(marv::gz::kSha256DigestBytes);
  s.rotor.struct_size = sizeof(s.rotor);
  s.rotor.pole_count = r.get<std::uint32_t>();
  s.rotor.latency_ticks = r.get<std::uint32_t>();
  s.rotor.exponent_bits = r.get<std::uint32_t>();
  s.rotor.mantissa_bits = r.get<std::uint32_t>();
  s.rotor.period_unit_s = r.get<double>();
  return s;
}

bool fail(std::string& error, const std::string& what, std::size_t pos) {
  error = what + " (byte " + std::to_string(pos) + ")";
  return false;
}

bool read_file(const std::string& path, std::vector<std::uint8_t>& out) {
  std::ifstream f(path, std::ios::binary);
  if (!f) {
    return false;
  }
  out.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
  return !f.bad();
}

// The log, with the structure the plugin writes (lockstep_log.hpp) checked: record 6 first; per host step one STEP, m
// TICKs numbered step * m + i, each followed by its TRUTH (skipped) if any and its ROTOR iff rotor_speed is 1, then one
// APPLIED; a trailer last whose counts are the records read.
bool read_log(const std::vector<std::uint8_t>& data, Log& log, std::string& error) {
  Reader r(data);
  std::array<char, kMagic.size()> magic{};
  r.bytes(magic.data(), magic.size());
  const auto version = r.get<std::uint32_t>();
  const auto header_size = r.get<std::uint32_t>();
  log.m = r.get<std::uint32_t>();
  log.num_us = r.get<std::uint32_t>();
  log.den = r.get<std::uint32_t>();
  r.get<std::uint64_t>();  // the header's seed: the replay takes record 6's
  r.skip(r.get<std::uint32_t>());
  if (!r.ok() || std::string_view(magic.data(), magic.size()) != kMagic || version != kVersion ||
      header_size != r.pos() || log.m == 0 || log.den == 0) {
    return fail(error, "not a lockstep log header of version " + std::to_string(kVersion), 0);
  }
  if (r.get<std::uint8_t>() != static_cast<std::uint8_t>(LogRecord::kSensors)) {
    return fail(error, "the first record is not record 6 (SENSORS): not a run with a sensor element", r.pos());
  }
  log.sensors = get_sensors(r);
  if (!r.ok()) {
    return fail(error, "record 6 is cut", r.pos());
  }
  if (log.sensors.gyro_source != static_cast<std::uint8_t>(LogGyroSource::kModel)) {
    return fail(error, "record 6's gyro_source is not 2 (model): not a model-gyro run", r.pos());
  }
  const bool rotor = log.sensors.rotor_speed != 0;
  bool step_open = false;
  bool trailer = false;
  std::uint64_t tick_count = 0;
  LogRecord last = LogRecord::kSensors;
  while (!r.at_end()) {
    const std::size_t at = r.pos();
    if (trailer) {
      return fail(error, "a record after the trailer", at);
    }
    const auto kind = static_cast<LogRecord>(r.get<std::uint8_t>());
    Tick* const prev = step_open && !log.steps.back().ticks.empty() ? &log.steps.back().ticks.back() : nullptr;
    switch (kind) {
      case LogRecord::kStep: {
        if (step_open) {
          return fail(error, "a STEP record before the previous step's APPLIED record", at);
        }
        Step s;
        r.skip(sizeof(std::uint64_t) + sizeof(std::uint64_t) + kStepGzDoubles * sizeof(double));
        s.body.struct_size = sizeof(s.body);
        get_doubles(r, s.body.pos_ned_m, std::size(s.body.pos_ned_m));
        get_doubles(r, s.body.vel_ned_m_s, std::size(s.body.vel_ned_m_s));
        get_doubles(r, s.body.q_wxyz, std::size(s.body.q_wxyz));
        get_doubles(r, s.body.omega_frd_rad_s, std::size(s.body.omega_frd_rad_s));
        log.steps.push_back(s);
        step_open = true;
        break;
      }
      case LogRecord::kTick: {
        if (!step_open || log.steps.back().ticks.size() == log.m || (prev != nullptr && rotor && !prev->rotor)) {
          return fail(error, "a TICK record out of place", at);
        }
        Tick t;
        t.tick = r.get<std::uint64_t>();
        r.get<std::uint64_t>();  // sil_t_us
        r.bytes(t.imu.data(), t.imu.size());
        for (std::uint16_t& d : t.dshot) {
          d = r.get<std::uint16_t>();
        }
        t.erpm_valid = r.get<std::uint32_t>();
        r.get<std::uint32_t>();  // 0
        get_doubles(r, t.force);
        get_doubles(r, t.torque);
        get_doubles(r, t.rotor_speed);
        get_doubles(r, t.erpm);
        r.skip(kWrenchDoubles * sizeof(double));
        const std::uint64_t want = (log.steps.size() - 1) * log.m + log.steps.back().ticks.size();
        if (r.ok() && t.tick != want) {
          return fail(error, "TICK record of tick " + std::to_string(t.tick) + " where tick " + std::to_string(want) +
                                 " is due",
                      at);
        }
        log.steps.back().ticks.push_back(t);
        ++tick_count;
        break;
      }
      case LogRecord::kTruth:
        if (last != LogRecord::kTick || prev == nullptr || r.get<std::uint64_t>() != prev->tick) {
          return fail(error, "a TRUTH record that does not follow its tick's TICK record", at);
        }
        r.skip(sizeof(marv_truth_state));
        break;
      case LogRecord::kRotor:
        if (!rotor || (last != LogRecord::kTick && last != LogRecord::kTruth) || prev == nullptr || prev->rotor ||
            r.get<std::uint64_t>() != prev->tick) {
          return fail(error, "a ROTOR record that does not follow its tick's TICK record, or one without rotor_speed 1",
                      at);
        }
        prev->rotor.emplace();
        r.bytes(prev->rotor->data(), prev->rotor->size());
        break;
      case LogRecord::kApplied:
        if (!step_open || log.steps.back().ticks.size() != log.m || (rotor && !prev->rotor)) {
          return fail(error, "an APPLIED record before the step's m ticks (and their ROTOR records)", at);
        }
        r.skip(kWrenchDoubles * sizeof(double));
        step_open = false;
        break;
      case LogRecord::kTrailer: {
        const auto steps = r.get<std::uint64_t>();
        const auto ticks = r.get<std::uint64_t>();
        const auto applied = r.get<std::uint64_t>();
        if (step_open || steps != log.steps.size() || ticks != tick_count || applied != log.steps.size()) {
          return fail(error, "the trailer's counts are not the records read", at);
        }
        trailer = true;
        break;
      }
      default:
        return fail(error, "an unknown record type or a second record 6", at);
    }
    if (!r.ok()) {
      return fail(error, "a record is cut", at);
    }
    last = kind;
  }
  if (!trailer) {
    return fail(error, "no trailer: the run did not end cleanly", r.pos());
  }
  return true;
}

std::vector<std::string> tokens(const std::string& line) {
  std::istringstream in(line);
  std::vector<std::string> out;
  for (std::string t; in >> t;) {
    out.push_back(t);
  }
  return out;
}

template <typename T>
bool parse(const std::string& text, T& out) {
  const char* last = text.data() + text.size();
  const auto r = std::from_chars(text.data(), last, out);
  return !text.empty() && r.ec == std::errc{} && r.ptr == last;
}

// Tokens of the rotor_speed_model line: the name, then latency_ticks, exponent_bits, mantissa_bits and period_unit_s.
// Rule: the usage above (one name and the four values of RotorGrid).
// Kind: derived.
constexpr std::size_t kGridFields = 1 + 4;

// The profile's rotor-speed values (the plant file's rotor_speed_model line).
struct RotorGrid {
  std::uint32_t latency_ticks = 0;
  std::uint32_t exponent_bits = 0;
  std::uint32_t mantissa_bits = 0;
  double period_unit_s = 0.0;
};

// The <plant> file (usage above).
bool read_plant(const std::string& path, marv_plant_config& pc, std::optional<RotorGrid>& grid, std::string& error) {
  std::ifstream f(path);
  if (!f) {
    error = "cannot read the plant file " + path;
    return false;
  }
  pc = marv_plant_config{};
  pc.struct_size = sizeof(pc);
  struct Scalar {
    std::string_view name;
    double* dst;
    bool seen;
  };
  std::array scalars{
      Scalar{"mass_kg", &pc.mass_kg, false},
      Scalar{"thrust_coeff", &pc.thrust_coeff, false},
      Scalar{"torque_ratio_m", &pc.torque_ratio_m, false},
      Scalar{"omega_min_rad_s", &pc.omega_min_rad_s, false},
      Scalar{"omega_max_rad_s", &pc.omega_max_rad_s, false},
      Scalar{"motor_tau_s", &pc.motor_tau_s, false},
      Scalar{"site_lat_rad", &pc.site_lat_rad, false},
      Scalar{"site_height_m", &pc.site_height_m, false},
      Scalar{"motor_substep_s", &pc.motor_substep_s, false},
  };
  bool esc = false;
  bool poles = false;
  bool initial = false;
  std::array<bool, kMotors> rotor_seen{};
  std::size_t line_no = 0;
  for (std::string line; std::getline(f, line);) {
    ++line_no;
    const std::vector<std::string> t = tokens(line);
    if (t.empty()) {
      continue;
    }
    const auto bad = [&](const std::string& why) {
      error = path + ":" + std::to_string(line_no) + ": " + why;
      return false;
    };
    const std::string& name = t[0];
    Scalar* s = nullptr;
    for (Scalar& c : scalars) {
      s = c.name == name ? &c : s;
    }
    if (s != nullptr) {
      if (s->seen || t.size() != 2 || !parse(t[1], *s->dst)) {
        return bad("'" + name + "' is repeated or is not one number");
      }
      s->seen = true;
    } else if (name == "esc_map" || name == "pole_count") {
      bool& seen = name == "esc_map" ? esc : poles;
      if (seen || t.size() != 2 || !parse(t[1], name == "esc_map" ? pc.esc_map : pc.pole_count)) {
        return bad("'" + name + "' is repeated or is not one unsigned integer");
      }
      seen = true;
    } else if (name == "rotor") {
      std::size_t motor = 0;
      constexpr std::size_t kFields = 2 + kAxes + 1;  // rotor, motor, position x y z, yaw_sign
      if (t.size() != kFields || !parse(t[1], motor) || motor < 1 || motor > kMotors || rotor_seen[motor - 1]) {
        return bad("'rotor' is not 'rotor <motor 1.." + std::to_string(kMotors) + "> <x> <y> <z> <yaw_sign>' once");
      }
      rotor_seen[motor - 1] = true;
      for (std::size_t i = 0; i < kAxes; ++i) {
        if (!parse(t[2 + i], pc.rotor_position_frd_m[motor - 1][i])) {
          return bad("a rotor position is not a number");
        }
      }
      if (!parse(t[2 + kAxes], pc.yaw_sign[motor - 1])) {
        return bad("a rotor yaw_sign is not an integer");
      }
    } else if (name == "rotor_speed_model") {
      RotorGrid g;
      if (grid || t.size() != kGridFields || !parse(t[1], g.latency_ticks) || !parse(t[2], g.exponent_bits) ||
          !parse(t[3], g.mantissa_bits) || !parse(t[4], g.period_unit_s)) {
        return bad("'rotor_speed_model' is repeated or is not three unsigned integers and a number");
      }
      grid = g;
    } else if (name == "initial_rotor_speed_rad_s") {
      if (initial || t.size() != 1 + kMotors) {
        return bad("'initial_rotor_speed_rad_s' is repeated or does not have " + std::to_string(kMotors) + " values");
      }
      for (std::size_t i = 0; i < kMotors; ++i) {
        if (!parse(t[1 + i], pc.initial_omega_rad_s[i])) {
          return bad("an initial rotor speed is not a number");
        }
      }
      initial = true;
    } else {
      return bad("unknown name '" + name + "'");
    }
  }
  for (const Scalar& c : scalars) {
    if (!c.seen) {
      error = path + ": '" + std::string(c.name) + "' is missing";
      return false;
    }
  }
  for (const bool seen : rotor_seen) {
    if (!seen) {
      error = path + ": a rotor line is missing";
      return false;
    }
  }
  if (!esc || !poles) {
    error = path + ": 'esc_map' or 'pole_count' is missing";
    return false;
  }
  return true;
}

bool same(double a, double b) { return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b); }

class Counts {
 public:
  void mismatch(const char* kind, std::optional<std::uint64_t> tick, const std::string& what) {
    std::printf("mismatch %s %s %s\n", kind, tick ? std::to_string(*tick).c_str() : "-", what.c_str());
    const std::string_view k = kind;
    std::uint64_t& n = k == "config" ? config_ : k == "imu" ? imu_ : k == "rotor" ? rotor_ : plant_;
    ++n;
  }
  bool any() const { return config_ + imu_ + rotor_ + plant_ != 0; }
  void summary(std::uint64_t ticks) const {
    std::printf("ticks %llu config %llu imu %llu rotor %llu plant %llu\n", static_cast<unsigned long long>(ticks),
                static_cast<unsigned long long>(config_), static_cast<unsigned long long>(imu_),
                static_cast<unsigned long long>(rotor_), static_cast<unsigned long long>(plant_));
  }

 private:
  std::uint64_t config_ = 0;
  std::uint64_t imu_ = 0;
  std::uint64_t rotor_ = 0;
  std::uint64_t plant_ = 0;
};

void check_axis(Counts& c, const char* name, const marv_plant_imu_axis_config& got,
                const marv_plant_imu_axis_config& want) {
  const std::string n = name;
  if (!same(got.noise_density, want.noise_density)) {
    c.mismatch("config", std::nullopt, n + ".noise_density");
  }
  if (!same(got.bias_instability, want.bias_instability)) {
    c.mismatch("config", std::nullopt, n + ".bias_instability");
  }
  if (!same(got.lsb, want.lsb)) {
    c.mismatch("config", std::nullopt, n + ".lsb");
  }
  if (!same(got.full_scale, want.full_scale)) {
    c.mismatch("config", std::nullopt, n + ".full_scale");
  }
  for (std::size_t i = 0; i < kAxes; ++i) {
    if (!same(got.turn_on_bias[i], want.turn_on_bias[i])) {
      c.mismatch("config", std::nullopt, n + ".turn_on_bias[" + std::to_string(i) + "]");
    }
  }
}

// The config checks (usage above).
void check_config(Counts& c, const Log& log, const marv_plant_config& pc, const std::optional<RotorGrid>& grid,
                  double t_true) {
  const Sensors& s = log.sensors;
  std::array<int, kBiasSigns> signs{};
  for (std::size_t i = 0; i < kBiasSigns; ++i) {
    signs[i] = s.signs[i];
  }
  const std::optional<marv_plant_imu_config> want = marv::sim::imu_corner_config(signs);
  if (!want) {
    c.mismatch("config", std::nullopt, "turn_on_bias_signs");
  } else {
    if (s.imu.latency_samples != want->latency_samples) {
      c.mismatch("config", std::nullopt, "latency_samples");
    }
    check_axis(c, "gyro", s.imu.gyro, want->gyro);
    check_axis(c, "accel", s.imu.accel, want->accel);
  }
  if (!same(s.gyro_bound, marv::sim::kImuGyroTurnOnBiasBound)) {
    c.mismatch("config", std::nullopt, "gyro_turn_on_bias_bound");
  }
  if (!same(s.accel_bound, marv::sim::kImuAccelTurnOnBiasBound)) {
    c.mismatch("config", std::nullopt, "accel_turn_on_bias_bound");
  }
  if (!same(s.odr_error, s.clock != 0 ? marv::sim::kImuOdrError : 0.0) || (s.clock == 0 && s.clock_corner != 0)) {
    c.mismatch("config", std::nullopt, "odr_error");
  }
  if (!same(s.tick_s, t_true)) {
    c.mismatch("config", std::nullopt, "tick_s");
  }
  if (s.imu_stream_id != marv::gz::kImuNoiseStreamId) {
    c.mismatch("config", std::nullopt, "imu_stream_id");
  }
  if (s.imu_counter_base != marv::gz::kImuCounterBase) {
    c.mismatch("config", std::nullopt, "imu_counter_base");
  }
  if (s.rotor_speed != 0 && s.rotor.pole_count != pc.pole_count) {
    c.mismatch("config", std::nullopt, "rotor pole_count");
  }
  if (s.rotor_speed != 0 && grid) {
    if (s.rotor.latency_ticks != grid->latency_ticks) {
      c.mismatch("config", std::nullopt, "rotor_latency_ticks");
    }
    if (s.rotor.exponent_bits != grid->exponent_bits) {
      c.mismatch("config", std::nullopt, "rotor_exponent_bits");
    }
    if (s.rotor.mantissa_bits != grid->mantissa_bits) {
      c.mismatch("config", std::nullopt, "rotor_mantissa_bits");
    }
    if (!same(s.rotor.period_unit_s, grid->period_unit_s)) {
      c.mismatch("config", std::nullopt, "rotor_period_unit_s");
    }
  }
}

// The first differing field of the plant output, or empty.
std::string plant_difference(const marv_plant_out& o, const Tick& t) {
  if (o.erpm_valid != t.erpm_valid) {
    return "erpm_valid";
  }
  const auto vec = [](const char* name, const double* got, const double* want, std::size_t n) -> std::string {
    for (std::size_t i = 0; i < n; ++i) {
      if (!same(got[i], want[i])) {
        return std::string(name) + "[" + std::to_string(i) + "]";
      }
    }
    return {};
  };
  for (const std::string& d : {vec("force_ned", o.force_ned_n, t.force.data(), kAxes),
                               vec("torque_ned", o.torque_ned_nm, t.torque.data(), kAxes),
                               vec("rotor_speed", o.rotor_speed_rad_s, t.rotor_speed.data(), kMotors),
                               vec("erpm", o.erpm, t.erpm.data(), kMotors)}) {
    if (!d.empty()) {
      return d;
    }
  }
  return {};
}

int error_exit(const std::string& what) {
  std::fprintf(stderr, "sensor_replay: %s\n", what.c_str());
  return kExitError;
}

int replay(const Log& log, marv_plant_config pc, const std::optional<RotorGrid>& grid) {
  const Sensors& s = log.sensors;
  if (s.rotor_speed != 0 && !grid) {
    return error_exit("record 6 has rotor_speed 1 and the plant file has no rotor_speed_model line");
  }
  const double t_true =
      s.clock != 0 ? static_cast<double>(s.host_step_ns) / (static_cast<double>(log.m) * marv::gz::kNanosecondsPerSecond)
                   : static_cast<double>(log.num_us) / (static_cast<double>(log.den) * marv::gz::kMicrosecondsPerSecond);
  Counts counts;
  check_config(counts, log, pc, grid, t_true);

  pc.rng_seed = s.seed;
  marv_plant* plant = nullptr;
  marv_plant_status st = marv_plant_create(&pc, &plant);
  if (st != MARV_PLANT_OK) {
    return error_exit(std::string("marv_plant_create: ") + marv_plant_status_str(st));
  }
  struct Destroy {
    marv_plant* p;
    ~Destroy() { marv_plant_destroy(p); }
  } destroy{plant};
  st = marv_plant_imu_attach(plant, &s.imu);
  if (st != MARV_PLANT_OK) {
    return error_exit(std::string("marv_plant_imu_attach: ") + marv_plant_status_str(st));
  }
  const bool rotor = s.rotor_speed != 0;
  if (rotor) {
    st = marv_plant_rotor_speed_attach(plant, &s.rotor);
    if (st != MARV_PLANT_OK) {
      return error_exit(std::string("marv_plant_rotor_speed_attach: ") + marv_plant_status_str(st));
    }
  }

  std::uint64_t ticks = 0;
  for (const Step& step : log.steps) {
    for (const Tick& t : step.ticks) {
      marv_plant_imu_out imu{};
      st = marv_plant_imu_sample(plant, &step.body, t_true, &imu);
      if (st != MARV_PLANT_OK) {
        return error_exit("marv_plant_imu_sample at tick " + std::to_string(t.tick) + ": " + marv_plant_status_str(st));
      }
      if (std::memcmp(&imu, t.imu.data(), t.imu.size()) != 0) {
        counts.mismatch("imu", t.tick, "bytes");
      }
      if (rotor) {
        marv_plant_rotor_speed_out ro{};
        st = marv_plant_rotor_speed_sample(plant, &ro);
        if (st != MARV_PLANT_OK) {
          return error_exit("marv_plant_rotor_speed_sample at tick " + std::to_string(t.tick) + ": " +
                            marv_plant_status_str(st));
        }
        if (std::memcmp(&ro, t.rotor->data(), t.rotor->size()) != 0) {
          counts.mismatch("rotor", t.tick, "bytes");
        }
      }
      marv_plant_cmd cmd{};
      cmd.struct_size = sizeof(cmd);
      for (std::size_t k = 0; k < kMotors; ++k) {
        cmd.dshot[k] = t.dshot[k];
      }
      marv_plant_out out{};
      out.struct_size = sizeof(out);
      st = marv_plant_step(plant, &step.body, &cmd, t_true, &out);
      if (st != MARV_PLANT_OK) {
        return error_exit("marv_plant_step at tick " + std::to_string(t.tick) + ": " + marv_plant_status_str(st));
      }
      const std::string d = plant_difference(out, t);
      if (!d.empty()) {
        counts.mismatch("plant", t.tick, d);
      }
      ++ticks;
    }
  }
  counts.summary(ticks);
  return counts.any() ? kExitMismatch : kExitEqual;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 3) {
    return error_exit("usage: sensor_replay <plant> <log>");
  }
  marv_plant_config pc{};
  std::optional<RotorGrid> grid;
  std::string error;
  if (!read_plant(argv[1], pc, grid, error)) {
    return error_exit(error);
  }
  std::vector<std::uint8_t> data;
  if (!read_file(argv[2], data)) {
    return error_exit(std::string("cannot read the log ") + argv[2]);
  }
  Log log;
  if (!read_log(data, log, error)) {
    return error_exit(std::string(argv[2]) + ": " + error);
  }
  return replay(log, pc, grid);
}
