// The gz-sim 8 (Harmonic) lockstep plugin marv::gz::Lockstep (quad spec 3.1, decision 0003 items 4 to 10): a thin shell
// over the host-free adapter (sim/gz/adapter). Configure reads the world's <plugin> element (schema in the docstring
// of tools/card/gen_world.py), checks the world against it and refuses anything that does not match (a REFUSED line
// on stderr, then std::abort: a gz System cannot stop the server with a failure status, and an abort is the one
// outcome that cannot be mistaken for a run). PreUpdate reads the state, steps the adapter, applies the wrench and
// logs (lockstep_log.hpp). Everything is serial, in PreUpdate; nothing reads the wall clock.
//
// The namespace marv::gz hides ::gz, so gz names are reached through the aliases below.
#include <gz/math/Inertial.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/config.hh>
#include <gz/sim/components/Gravity.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Physics.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <sdf/Element.hh>
#include <sdf/Physics.hh>

#include <array>
#include <atomic>
#include <bit>
#include <charconv>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

#include <marv/params/param_ids.hpp>

#include "constants.hpp"
#include "lockstep_log.hpp"
#include "marv/gz/adapter.hpp"
#include "marv/gz/frames.hpp"
#include "marv/gz/truth_gyro.hpp"
#include "marv_plant.h"
#include "marv_sil.h"
#include "marv_truth.h"
#ifdef MARV_GZ_TRUTH_STATE
#include "marv/gz/truth_attitude.hpp"
#endif

namespace gzs = ::gz::sim;
namespace gzm = ::gz::math;

namespace marv::gz {
namespace {

constexpr std::size_t kMotors = MARV_PLANT_N_MOTORS;

[[noreturn]] void refuse(const std::string& reason) {
  std::fprintf(stderr, "marv_gz_lockstep: REFUSED: %s\n", reason.c_str());
  std::fflush(stderr);
  std::abort();
}

std::string trim(const std::string& s) {
  const char* ws = " \t\r\n";
  const std::size_t b = s.find_first_not_of(ws);
  if (b == std::string::npos) {
    return {};
  }
  return s.substr(b, s.find_last_not_of(ws) - b + 1);
}

template <typename T>
bool parse_number(const std::string& text, T& out) {
  const std::string t = trim(text);
  const char* first = t.data();
  const char* last = t.data() + t.size();
  const auto r = std::from_chars(first, last, out);
  return !t.empty() && r.ec == std::errc{} && r.ptr == last;
}

bool same_bits(double a, double b) { return std::bit_cast<std::uint64_t>(a) == std::bit_cast<std::uint64_t>(b); }

// The plugin element, parsed. Every schema element must be present exactly once (rotor, sil_override: as documented);
// an unknown element, a duplicate or an unparsable text refuses.
struct Override {
  std::string param;
  std::uint32_t type = 0;
  float f32 = 0.0F;
  std::int32_t i32 = 0;
};

constexpr std::size_t kAxes = std::extent_v<decltype(marv_plant_imu_axis_config::turn_on_bias)>;
constexpr std::size_t kBiasSigns = 2 * kAxes;  // gyro x y z, then accel x y z

// <imu_model> (decision 0019): the plant IMU configuration of the world, turn_on_bias = sign * bound per axis (the rule of
// imu_corner_config, tools/card/gen_imu_config.py), and what record 6 logs besides.
struct ImuModel {
  marv_plant_imu_config cfg{};
  double gyro_bound = 0.0;
  double accel_bound = 0.0;
  std::array<std::int8_t, kBiasSigns> signs{};
  std::array<std::uint8_t, kSha256DigestBytes> profile_sha256{};
};

struct Parsed {
  marv_plant_config plant{};
  std::uint32_t m = 0;
  std::uint32_t num_us = 0;
  std::uint32_t den = 0;
  std::uint64_t seed = 0;
  std::vector<Override> overrides;
  std::optional<Vec3> v_ned;
  std::optional<Vec3> w_frd;
  std::optional<std::string> log_path;
  bool truth_gyro = false;
  bool truth_attitude = false;
  bool model_gyro = false;
  std::optional<ImuModel> imu_model;
  std::optional<marv_plant_rotor_speed_config> rotor_speed;
  std::optional<std::int32_t> clock_corner;
  std::optional<double> odr_error;
};

std::string text_of(const sdf::ElementPtr& e) { return e->Get<std::string>(); }

std::string attribute_of(const sdf::ElementPtr& e, const std::string& name) {
  const sdf::ParamPtr a = e->GetAttribute(name);
  if (!a) {
    refuse("element <" + e->GetName() + "> has no attribute '" + name + "'");
  }
  return a->GetAsString();
}

double number_of(const sdf::ElementPtr& e) {
  double v = 0.0;
  if (!parse_number(text_of(e), v)) {
    refuse("element <" + e->GetName() + "> is not a number: '" + text_of(e) + "'");
  }
  return v;
}

template <typename T>
T integer_of(const sdf::ElementPtr& e) {
  T v{};
  if (!parse_number(text_of(e), v)) {
    refuse("element <" + e->GetName() + "> is not an integer of the expected range: '" + text_of(e) + "'");
  }
  return v;
}

template <std::size_t N>
std::array<double, N> vector_of(const sdf::ElementPtr& e) {
  std::array<double, N> v{};
  const std::string t = trim(text_of(e));
  std::size_t pos = 0;
  for (double& c : v) {
    while (pos < t.size() && (t[pos] == ' ' || t[pos] == '\t' || t[pos] == '\n' || t[pos] == '\r')) {
      ++pos;
    }
    const char* first = t.data() + pos;
    const char* last = t.data() + t.size();
    const auto r = std::from_chars(first, last, c);
    if (r.ec != std::errc{}) {
      refuse("element <" + e->GetName() + "> is not a vector: '" + t + "'");
    }
    pos = static_cast<std::size_t>(r.ptr - t.data());
  }
  if (t.find_first_not_of(" \t\r\n", pos) != std::string::npos) {
    refuse("element <" + e->GetName() + "> has more components than a vector: '" + t + "'");
  }
  return v;
}

Vec3 vec3_of(const sdf::ElementPtr& e) { return vector_of<3>(e); }

// The children of a sensor block (decision 0019): each of `names` exactly once, in any order, and nothing else.
std::vector<sdf::ElementPtr> children_of(const sdf::ElementPtr& block, const std::vector<std::string>& names) {
  std::vector<sdf::ElementPtr> out(names.size());
  for (sdf::ElementPtr c = block->GetFirstElement(); c; c = c->GetNextElement("")) {
    std::size_t i = 0;
    while (i < names.size() && names[i] != c->GetName()) {
      ++i;
    }
    if (i == names.size()) {
      refuse("unknown element <" + c->GetName() + "> in <" + block->GetName() + ">");
    }
    if (out[i]) {
      refuse("<" + block->GetName() + "> element <" + c->GetName() + "> appears twice");
    }
    out[i] = c;
  }
  for (std::size_t i = 0; i < names.size(); ++i) {
    if (!out[i]) {
      refuse("<" + block->GetName() + "> lacks <" + names[i] + ">");
    }
  }
  return out;
}

// <gyro> or <accel> of <imu_model>: the axis configuration without its turn-on bias; returns the turn-on bias bound.
double imu_axis_of(const sdf::ElementPtr& e, marv_plant_imu_axis_config& a) {
  const std::vector<sdf::ElementPtr> c =
      children_of(e, {"noise_density", "bias_instability", "lsb", "full_scale", "turn_on_bias_bound"});
  a.noise_density = number_of(c[0]);
  a.bias_instability = number_of(c[1]);
  a.lsb = number_of(c[2]);
  a.full_scale = number_of(c[3]);
  return number_of(c[4]);
}

std::array<std::int8_t, kBiasSigns> signs_of(const sdf::ElementPtr& e) {
  std::array<std::int8_t, kBiasSigns> s{};
  const std::string t = trim(text_of(e));
  std::size_t pos = 0;
  for (std::size_t i = 0; i < s.size(); ++i) {
    pos = t.find_first_not_of(" \t\r\n", pos);
    std::int32_t v = 0;
    const auto r = pos == std::string::npos ? std::from_chars_result{nullptr, std::errc::invalid_argument}
                                            : std::from_chars(t.data() + pos, t.data() + t.size(), v);
    if (r.ec != std::errc{}) {
      refuse("<turn_on_bias_signs> is not " + std::to_string(s.size()) + " integers: '" + t + "'");
    }
    if (v < -1 || v > 1) {
      refuse("<turn_on_bias_signs> component " + std::to_string(i) + " is " + std::to_string(v) + ", not -1, 0 or +1");
    }
    s[i] = static_cast<std::int8_t>(v);
    pos = static_cast<std::size_t>(r.ptr - t.data());
  }
  if (t.find_first_not_of(" \t\r\n", pos) != std::string::npos) {
    refuse("<turn_on_bias_signs> has more than " + std::to_string(s.size()) + " components: '" + t + "'");
  }
  return s;
}

std::array<std::uint8_t, kSha256DigestBytes> sha256_of(const sdf::ElementPtr& e) {
  constexpr std::string_view kHex = "0123456789abcdef";
  std::array<std::uint8_t, kSha256DigestBytes> d{};
  const std::string t = trim(text_of(e));
  const auto bad = [&] {
    refuse("<profile_sha256> is not " + std::to_string(2 * d.size()) + " lowercase hexadecimal digits: '" + t + "'");
  };
  if (t.size() != 2 * d.size()) {
    bad();
  }
  for (std::size_t i = 0; i < d.size(); ++i) {
    const std::size_t hi = kHex.find(t[2 * i]);
    const std::size_t lo = kHex.find(t[2 * i + 1]);
    if (hi == std::string_view::npos || lo == std::string_view::npos) {
      bad();
    }
    d[i] = static_cast<std::uint8_t>(hi * kHex.size() + lo);
  }
  return d;
}

ImuModel imu_model_of(const sdf::ElementPtr& e) {
  const std::vector<sdf::ElementPtr> c =
      children_of(e, {"latency_samples", "gyro", "accel", "turn_on_bias_signs", "profile_sha256"});
  ImuModel m;
  m.cfg.struct_size = sizeof(m.cfg);
  m.cfg.latency_samples = integer_of<std::uint32_t>(c[0]);
  m.gyro_bound = imu_axis_of(c[1], m.cfg.gyro);
  m.accel_bound = imu_axis_of(c[2], m.cfg.accel);
  m.signs = signs_of(c[3]);
  m.profile_sha256 = sha256_of(c[4]);
  for (std::size_t i = 0; i < kAxes; ++i) {
    m.cfg.gyro.turn_on_bias[i] = static_cast<double>(m.signs[i]) * m.gyro_bound;
    m.cfg.accel.turn_on_bias[i] = static_cast<double>(m.signs[kAxes + i]) * m.accel_bound;
  }
  return m;
}

// <rotor_speed_model>; pole_count is the plugin's <pole_count>, filled in by the caller.
marv_plant_rotor_speed_config rotor_speed_model_of(const sdf::ElementPtr& e) {
  const std::vector<sdf::ElementPtr> c =
      children_of(e, {"latency_ticks", "exponent_bits", "mantissa_bits", "period_unit_s"});
  marv_plant_rotor_speed_config r{};
  r.struct_size = sizeof(r);
  r.latency_ticks = integer_of<std::uint32_t>(c[0]);
  r.exponent_bits = integer_of<std::uint32_t>(c[1]);
  r.mantissa_bits = integer_of<std::uint32_t>(c[2]);
  r.period_unit_s = number_of(c[3]);
  return r;
}

Parsed parse_plugin(const sdf::ElementPtr& root) {
  Parsed p;
  p.plant.struct_size = sizeof(p.plant);
  std::vector<std::string> seen;
  std::array<bool, kMotors> rotor_seen{};
  bool have_esc = false;
  bool have_m = false;
  bool have_num = false;
  bool have_den = false;
  bool have_seed = false;
  const auto once = [&](const std::string& name) {
    for (const std::string& s : seen) {
      if (s == name) {
        refuse("plugin element <" + name + "> appears twice");
      }
    }
    seen.push_back(name);
  };
  struct Scalar {
    const char* name;
    double* dst;
  };
  const Scalar scalars[] = {
      {"mass_kg", &p.plant.mass_kg},
      {"thrust_coeff", &p.plant.thrust_coeff},
      {"torque_ratio_m", &p.plant.torque_ratio_m},
      {"omega_min_rad_s", &p.plant.omega_min_rad_s},
      {"omega_max_rad_s", &p.plant.omega_max_rad_s},
      {"motor_tau_s", &p.plant.motor_tau_s},
      {"site_lat_rad", &p.plant.site_lat_rad},
      {"site_height_m", &p.plant.site_height_m},
      {"motor_substep_s", &p.plant.motor_substep_s},
  };
  for (sdf::ElementPtr e = root->GetFirstElement(); e; e = e->GetNextElement("")) {
    const std::string name = e->GetName();
    bool matched = false;
    for (const Scalar& s : scalars) {
      if (name == s.name) {
        once(name);
        *s.dst = number_of(e);
        matched = true;
        break;
      }
    }
    if (matched) {
      continue;
    }
    if (name == "esc_map") {
      once(name);
      p.plant.esc_map = integer_of<std::uint32_t>(e);
      have_esc = true;
    } else if (name == "pole_count") {
      once(name);
      p.plant.pole_count = integer_of<std::uint32_t>(e);
    } else if (name == "ticks_per_step") {
      once(name);
      p.m = integer_of<std::uint32_t>(e);
      have_m = true;
    } else if (name == "tick_period_num_us") {
      once(name);
      p.num_us = integer_of<std::uint32_t>(e);
      have_num = true;
    } else if (name == "tick_period_den") {
      once(name);
      p.den = integer_of<std::uint32_t>(e);
      have_den = true;
    } else if (name == "seed") {
      once(name);
      p.seed = integer_of<std::uint64_t>(e);
      p.plant.rng_seed = p.seed;
      have_seed = true;
    } else if (name == "rotor") {
      const std::uint32_t motor = [&] {
        std::uint32_t v = 0;
        if (!parse_number(attribute_of(e, "motor"), v) || v < 1 || v > kMotors) {
          refuse("<rotor> motor attribute is not a logical motor number 1.." + std::to_string(kMotors));
        }
        return v;
      }();
      if (rotor_seen[motor - 1]) {
        refuse("<rotor motor=" + std::to_string(motor) + "> appears twice");
      }
      rotor_seen[motor - 1] = true;
      const sdf::ElementPtr pos = e->FindElement("position_frd_m");
      const sdf::ElementPtr yaw = e->FindElement("yaw_sign");
      if (!pos || !yaw) {
        refuse("<rotor motor=" + std::to_string(motor) + "> lacks position_frd_m or yaw_sign");
      }
      const Vec3 v = vec3_of(pos);
      for (std::size_t i = 0; i < v.size(); ++i) {
        p.plant.rotor_position_frd_m[motor - 1][i] = v[i];
      }
      p.plant.yaw_sign[motor - 1] = integer_of<std::int32_t>(yaw);
    } else if (name == "sil_override") {
      Override o;
      o.param = attribute_of(e, "param");
      const std::string type = attribute_of(e, "type");
      if (type == "i32") {
        o.type = MARV_PARAM_I32;
        o.i32 = integer_of<std::int32_t>(e);
      } else if (type == "f32") {
        o.type = MARV_PARAM_F32;
        if (!parse_number(text_of(e), o.f32)) {
          refuse("sil_override '" + o.param + "' is not a float: '" + text_of(e) + "'");
        }
      } else {
        refuse("sil_override '" + o.param + "' has type '" + type + "', not i32 or f32");
      }
      p.overrides.push_back(std::move(o));
    } else if (name == "initial_velocity_ned_m_s") {
      once(name);
      p.v_ned = vec3_of(e);
    } else if (name == "initial_body_rates_frd") {
      once(name);
      p.w_frd = vec3_of(e);
    } else if (name == "initial_rotor_speed_rad_s") {
      once(name);
      const std::array<double, kMotors> w = vector_of<kMotors>(e);
      for (std::size_t i = 0; i < kMotors; ++i) {
        p.plant.initial_omega_rad_s[i] = w[i];
      }
    } else if (name == "gyro_source") {
      once(name);
      const std::string source = trim(text_of(e));
      if (source == "truth") {
        p.truth_gyro = true;
      } else if (source == "model") {
        p.model_gyro = true;
      } else {
        refuse("<gyro_source> is '" + source + "', not 'truth' or 'model'");
      }
    } else if (name == "imu_model") {
      once(name);
      p.imu_model = imu_model_of(e);
    } else if (name == "rotor_speed_model") {
      once(name);
      p.rotor_speed = rotor_speed_model_of(e);
    } else if (name == "clock_corner") {
      once(name);
      p.clock_corner = integer_of<std::int32_t>(e);
    } else if (name == "odr_error") {
      once(name);
      p.odr_error = number_of(e);
    } else if (name == "attitude_source") {
      once(name);
      if (trim(text_of(e)) != "truth") {
        refuse("<attitude_source> is '" + trim(text_of(e)) + "', not 'truth'");
      }
      p.truth_attitude = true;
    } else if (name == "log_path") {
      once(name);
      const std::string t = trim(text_of(e));
      if (t.empty()) {
        refuse("<log_path> is empty");
      }
      p.log_path = t;
    } else {
      refuse("unknown plugin element <" + name + ">");
    }
  }
  const char* required[] = {"mass_kg",         "thrust_coeff",  "torque_ratio_m", "omega_min_rad_s",
                            "omega_max_rad_s", "motor_tau_s",   "site_lat_rad",   "site_height_m",
                            "motor_substep_s", "pole_count"};
  for (const char* r : required) {
    bool found = false;
    for (const std::string& s : seen) {
      found = found || s == r;
    }
    if (!found) {
      refuse(std::string("plugin element <") + r + "> is missing");
    }
  }
  for (std::size_t i = 0; i < kMotors; ++i) {
    if (!rotor_seen[i]) {
      refuse("<rotor motor=" + std::to_string(i + 1) + "> is missing");
    }
  }
  if (!have_esc || !have_m || !have_num || !have_den || !have_seed) {
    refuse("a required plugin element is missing (esc_map, ticks_per_step, tick_period_num_us, tick_period_den, seed)");
  }
  return p;
}

// The realised host step of a clock corner (decision 0019, ruling 10): n_true = outward(n / (1 + c e)) ns, exactly. The
// binary64 e is M 2^-K (M odd), so n / (1 + c e) = n 2^K / (2^K + c M), evaluated in unsigned 128-bit integers; outward is
// floor for c = +1 (the fast clock's shorter step), ceil for c = -1, and n itself for c = 0 or e = 0. e is finite, in
// [0, 1), and c in {-1, 0, +1} (checked by the caller); an e whose 2^K does not fit the exact arithmetic is refused.
std::uint64_t realised_host_step_ns(std::uint64_t n, std::int32_t corner, double e) {
  if (corner == 0) {
    return n;
  }
  __extension__ using Wide = unsigned __int128;
  constexpr int kWideBits = static_cast<int>(sizeof(Wide) * CHAR_BIT);
  constexpr int kDigits = std::numeric_limits<double>::digits;
  int exponent = 0;
  const double fraction = std::frexp(e, &exponent);
  auto mantissa = static_cast<std::uint64_t>(std::ldexp(fraction, kDigits));
  int k = kDigits - exponent;
  if (mantissa == 0) {
    return n;
  }
  while ((mantissa & 1U) == 0 && k > 0) {
    mantissa >>= 1U;
    --k;
  }
  if (k < 1 || static_cast<int>(std::bit_width(n)) + k >= kWideBits) {
    refuse("<odr_error> is not M 2^-K with n 2^K inside the exact 128-bit arithmetic of the realised host step");
  }
  const Wide scaled = static_cast<Wide>(n) << static_cast<unsigned>(k);
  const Wide unit = static_cast<Wide>(1) << static_cast<unsigned>(k);
  Wide q = 0;
  if (corner > 0) {
    q = scaled / (unit + mantissa);
  } else {
    const Wide den = unit - mantissa;
    q = (scaled + den - 1) / den;
  }
  if (q > std::numeric_limits<std::uint64_t>::max()) {
    refuse("the realised host step does not fit in 64 bits of nanoseconds");
  }
  return static_cast<std::uint64_t>(q);
}

// A command source that keeps the SIL's stamp and the IMU sample of every tick of the current host step. The sample is
// the one passed to the SIL: SilCommandSource passes a zeroed one; the truth-gyro source (optional <gyro_source>) its own.
// With the optional <attitude_source> (which needs the truth gyro and a SIL library built with TRUTH_STATE) the truth-
// attitude source wraps the truth-gyro one, and the marv_truth_state passed for every tick is kept as well.
// With <gyro_source>model</gyro_source> (decision 0019) the adapter hands each tick the plant's IMU sample (and, with
// <rotor_speed_model>, its rotor-speed sample): the three- and four-argument forms below pass them to the SIL through
// RotorSilCommandSource (marv_sil_tick, or marv_sil_tick_with_rotor_speed with a rotor sample), wrapped by the truth-
// attitude source with <attitude_source>, and keep both samples. Without that element only the two-argument form runs.
class RecordingSource final : public CommandSource {
 public:
  bool dshot(std::uint64_t tick, Dshot& out) override {
    bool ok = false;
#ifdef MARV_GZ_TRUTH_STATE
    if (use_attitude_) {
      ok = attitude_.dshot(tick, out);
      if (ok) {
        truths_.push_back(attitude_.state());
      }
    } else
#endif
    {
      ok = use_truth_ ? truth_.dshot(tick, out) : inner_.dshot(tick, out);
    }
    if (ok) {
      stamps_.push_back(use_truth_ ? truth_.last_stamp_us() : inner_.last_stamp_us());
      imus_.push_back(use_truth_ ? truth_.imu() : marv_imu_meas{});
    }
    return ok;
  }
  bool dshot(std::uint64_t tick, const marv_imu_meas* imu, Dshot& out) override { return dshot(tick, imu, nullptr, out); }
  bool dshot(std::uint64_t tick, const marv_imu_meas* imu, const marv_rotor_speed_meas* rotor, Dshot& out) override {
    if (!use_model_) {
      return CommandSource::dshot(tick, imu, rotor, out);
    }
    bool ok = false;
#ifdef MARV_GZ_TRUTH_STATE
    if (use_attitude_) {
      ok = attitude_model_.dshot(tick, imu, rotor, out);
      if (ok) {
        truths_.push_back(attitude_model_.state());
      }
    } else
#endif
    {
      ok = model_.dshot(tick, imu, rotor, out);
    }
    if (ok) {
      stamps_.push_back(model_.last_stamp_us());
      imus_.push_back(imu != nullptr ? *imu : marv_imu_meas{});
      rotors_.push_back(rotor != nullptr ? *rotor : marv_rotor_speed_meas{});
    }
    return ok;
  }
  // The failed call of the last dshot(), as a message.
  std::string failure() const {
#ifdef MARV_GZ_TRUTH_STATE
    if (use_attitude_ && use_model_ && attitude_model_.truth_status() != MARV_SIL_OK) {
      return std::string("marv_truth_state_set: ") +
             marv_sil_status_str(static_cast<marv_sil_status>(attitude_model_.truth_status()));
    }
    if (use_attitude_ && !use_model_ && attitude_.truth_status() != MARV_SIL_OK) {
      return std::string("marv_truth_state_set: ") +
             marv_sil_status_str(static_cast<marv_sil_status>(attitude_.truth_status()));
    }
#endif
    if (use_model_) {
      return std::string("marv_sil_tick (model IMU): ") +
             marv_sil_status_str(static_cast<marv_sil_status>(model_.last_status()));
    }
    return std::string("marv_sil_tick: ") +
           marv_sil_status_str(static_cast<marv_sil_status>(use_truth_ ? truth_.last_status() : inner_.last_status()));
  }
  void use_truth_gyro() { use_truth_ = true; }
  void use_model_gyro() { use_model_ = true; }
  void use_truth_attitude() {
#ifdef MARV_GZ_TRUTH_STATE
    use_attitude_ = true;
#endif
  }
  void set_body(const marv_plant_body& body) {
    truth_.set_body(body);
#ifdef MARV_GZ_TRUTH_STATE
    attitude_.set_body(body);
    if (use_model_) {
      attitude_model_.set_body(body);
    }
#endif
  }
  void begin_step() {
    stamps_.clear();
    imus_.clear();
    truths_.clear();
    rotors_.clear();
  }
  const std::vector<std::uint64_t>& stamps() const { return stamps_; }
  const std::vector<marv_imu_meas>& imus() const { return imus_; }
  const std::vector<marv_truth_state>& truths() const { return truths_; }
  const std::vector<marv_rotor_speed_meas>& rotors() const { return rotors_; }

 private:
  SilCommandSource inner_;
  truth::TruthGyroSilCommandSource truth_;
  RotorSilCommandSource model_;
#ifdef MARV_GZ_TRUTH_STATE
  truth::TruthAttitude attitude_{truth_};
  truth::TruthAttitude attitude_model_{model_};
  bool use_attitude_ = false;
#endif
  bool use_truth_ = false;
  bool use_model_ = false;
  std::vector<std::uint64_t> stamps_;
  std::vector<marv_imu_meas> imus_;
  std::vector<marv_truth_state> truths_;
  std::vector<marv_rotor_speed_meas> rotors_;
};

std::atomic<bool> g_configured{false};

void put3(LogBuffer& b, const Vec3& v) {
  for (const double c : v) {
    b.f64(c);
  }
}

}  // namespace

class Lockstep final : public gzs::System,
                       public gzs::ISystemConfigure,
                       public gzs::ISystemPreUpdate,
                       public gzs::ISystemReset {
 public:
  ~Lockstep() override {
    if (log_.is_open()) {
      log_.close_with_trailer(steps_, ticks_, steps_);
    }
    if (sil_ready_) {
      marv_sil_shutdown();
    }
  }

  void Configure(const gzs::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf, gzs::EntityComponentManager& ecm,
                 gzs::EventManager& /*event_mgr*/) override {
    if (g_configured.exchange(true)) {
      refuse("a second marv::gz::Lockstep instance in this process (one SIL init per process)");
    }
    const Parsed p = parse_plugin(sdf->Clone());
    if (p.truth_attitude && !p.truth_gyro && !p.model_gyro) {
      refuse("<attitude_source> needs <gyro_source>truth</gyro_source>");
    }
#ifndef MARV_GZ_TRUTH_STATE
    if (p.truth_attitude) {
      refuse("<attitude_source> needs a SIL library with marv_truth_state_set (built with TRUTH_STATE, target property "
             "MARV_SIL_TRUTH_STATE); the linked one has none");
    }
#endif
    m_ = p.m;
    if (m_ < 1) {
      refuse("ticks_per_step is 0");
    }
    if (p.den == 0 || p.num_us == 0) {
      refuse("the tick period is not positive");
    }
    const std::uint64_t num_ns = static_cast<std::uint64_t>(p.num_us) * kNanosecondsPerMicrosecond;
    if (num_ns % p.den != 0) {
      refuse("tick_ns = num*1000/den is not an integer");
    }
    tick_ns_ = num_ns / p.den;
    t_tick_s_ = tick_period_s(p.num_us, p.den);
    if (!same_bits(p.plant.motor_substep_s, t_tick_s_)) {
      refuse("motor_substep_s is not the tick period");
    }
    host_ns_ = m_ * tick_ns_;
    check_sensors(p);

    check_world(entity, ecm, p);

    marv_plant* plant = nullptr;
    const marv_plant_status ps = marv_plant_create(&p.plant, &plant);
    if (ps != MARV_PLANT_OK) {
      refuse(std::string("marv_plant_create: ") + marv_plant_status_str(ps));
    }

    marv_sil_info info{};
    info.struct_size = sizeof(info);
    const marv_sil_status is = marv_sil_info_get(&info);
    if (is != MARV_SIL_OK) {
      refuse(std::string("marv_sil_info_get: ") + marv_sil_status_str(is));
    }
    if (info.n_motors != kMotors) {
      refuse("the SIL composition has a different number of motors than marv_plant");
    }
    std::vector<marv_sil_param_override> ov;
    for (const Override& o : p.overrides) {
      std::optional<std::uint32_t> id;
      for (std::size_t i = 0; i < kParamCount; ++i) {
        if (o.param == generated::kParamNames[i]) {
          id = static_cast<std::uint32_t>(i);
        }
      }
      if (!id) {
        refuse("sil_override names a parameter the parameter set does not have: '" + o.param + "'");
      }
      marv_sil_param_override r{};
      r.id = *id;
      r.type = o.type;
      r.f32 = o.f32;
      r.i32 = o.i32;
      r.sigma = 0.0F;
      ov.push_back(r);
    }
    marv_sil_config sc{};
    sc.struct_size = sizeof(sc);
    sc.imu_meas_size = sizeof(marv_imu_meas);
    sc.override_size = sizeof(marv_sil_param_override);
    sc.tick_period_num_us = p.num_us;
    sc.tick_period_den = p.den;
    sc.n_overrides = static_cast<std::uint32_t>(ov.size());
    sc.overrides = ov.empty() ? nullptr : ov.data();
    sc.param_schema_hash = info.param_schema_hash;
    const marv_sil_status ss = marv_sil_init(&sc);
    if (ss != MARV_SIL_OK) {
      marv_plant_destroy(plant);
      refuse(std::string("marv_sil_init: ") + marv_sil_status_str(ss) + " (index " +
             std::to_string(marv_sil_error_index()) + ")");
    }
    sil_ready_ = true;

    if (!sensors_) {
      adapter_ = std::make_unique<Adapter>(plant, source_, t_tick_s_);
    } else {
      start_sensors(plant, p);
    }
    link_.EnableVelocityChecks(ecm, true);
    // Exists only for the negative control of tests/regression/quad/L02/gz/test_plugin_smoke.py: with it set, the
    // velocity command components are not removed, which is the defect the removal fixes. Never set outside that test.
    keep_vel_cmd_ = std::getenv("MARV_GZ_TEST_KEEP_VEL_CMD") != nullptr;
    // Exists only for the negative control of tests/regression/quad/L02/gz/test_first_read.py: with it set, the body
    // fed at step 0 is gz's read (zero rates), which is the defect decision 0016 fixes. Never set outside that test.
    zero_first_read_ = std::getenv("MARV_GZ_TEST_ZERO_FIRST_READ") != nullptr;
    v_ned_ = p.v_ned;
    w_frd_ = p.w_frd;

    if (p.truth_gyro) {
      source_.use_truth_gyro();
    }
    if (p.truth_attitude) {
      source_.use_truth_attitude();
      log_truth_ = true;
    }

    log_path_ = p.log_path;
    log_params_ = {m_, p.num_us, p.den, p.seed};
  }

  void PreUpdate(const gzs::UpdateInfo& info, gzs::EntityComponentManager& ecm) override {
    if (info.paused) {
      return;
    }
    if (!started_) {
      check_physics(ecm);
      open_log();
      started_ = true;
    }
    // The Physics system applies a link velocity command and keeps the component (measured: it resets it to zero
    // rather than removing it, so a kept command pins the velocity to zero at every later step). The command is
    // removed at the PreUpdate after the one that set it.
    if (vel_cmd_pending_) {
      vel_cmd_pending_ = false;
      if (!keep_vel_cmd_) {
        ecm.RemoveComponent<gzs::components::LinearVelocityCmd>(link_.Entity());
        ecm.RemoveComponent<gzs::components::AngularVelocityCmd>(link_.Entity());
      }
    }
    const std::chrono::nanoseconds host_dt{host_ns_};
    if (info.dt != host_dt) {
      refuse("UpdateInfo dt " + std::to_string(std::chrono::nanoseconds(info.dt).count()) + " ns is not " +
             (clock_ ? "the realised host step n_true " : "m*tick ") + std::to_string(host_dt.count()) + " ns");
    }
    const std::chrono::nanoseconds expected_sim{(steps_ + 1) * host_ns_};
    if (info.simTime != expected_sim) {
      refuse("simTime " + std::to_string(std::chrono::nanoseconds(info.simTime).count()) + " ns is not " +
             (clock_ ? "steps*n_true " : "ticks*tick ") + std::to_string(expected_sim.count()) + " ns");
    }

    const auto* pose_c = ecm.Component<gzs::components::WorldPose>(link_.Entity());
    const auto* lin_c = ecm.Component<gzs::components::WorldLinearVelocity>(link_.Entity());
    const auto* ang_c = ecm.Component<gzs::components::WorldAngularVelocity>(link_.Entity());
    if (pose_c == nullptr || lin_c == nullptr || ang_c == nullptr) {
      refuse("the link lacks WorldPose, WorldLinearVelocity or WorldAngularVelocity");
    }
    const gzm::Pose3d pose = pose_c->Data();
    const gzm::Vector3d lin = lin_c->Data();
    const gzm::Vector3d ang = ang_c->Data();
    GzState gs{};
    gs.pos_enu_m = {pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z()};
    gs.q_eu_wxyz = {pose.Rot().W(), pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z()};
    gs.lin_vel_world_m_s = {lin.X(), lin.Y(), lin.Z()};
    gs.ang_vel_world_rad_s = {ang.X(), ang.Y(), ang.Z()};
    marv_plant_body body = to_plant_body(gs);
    // Decision 0016: the body starts from (q0, w0) at t = 0, but gz reads zero rates before the first physics step
    // (the scenario's rates are applied at the end of this PreUpdate), so step 0 is fed the scenario's FRD rates. The
    // log keeps the raw gz read in the gz fields.
    if (steps_ == 0 && w_frd_ && !zero_first_read_) {
      for (std::size_t i = 0; i < std::size(body.omega_frd_rad_s); ++i) {
        body.omega_frd_rad_s[i] = (*w_frd_)[i];
      }
    }

    const std::uint64_t first_tick = steps_ * m_;
    source_.begin_step();
    source_.set_body(body);
    const StepResult r = adapter_->step(body, first_tick, m_);

    LogBuffer b;
    b.u8(static_cast<std::uint8_t>(LogRecord::kStep));
    b.u64(info.iterations);
    b.u64(static_cast<std::uint64_t>(std::chrono::nanoseconds(info.simTime).count()));
    put3(b, gs.pos_enu_m);
    for (const double c : gs.q_eu_wxyz) {
      b.f64(c);
    }
    put3(b, gs.lin_vel_world_m_s);
    put3(b, gs.ang_vel_world_rad_s);
    put3(b, {body.pos_ned_m[0], body.pos_ned_m[1], body.pos_ned_m[2]});
    put3(b, {body.vel_ned_m_s[0], body.vel_ned_m_s[1], body.vel_ned_m_s[2]});
    for (const double c : body.q_wxyz) {
      b.f64(c);
    }
    put3(b, {body.omega_frd_rad_s[0], body.omega_frd_rad_s[1], body.omega_frd_rad_s[2]});
    for (std::size_t j = 0; j < r.ticks.size(); ++j) {
      const TickOutput& t = r.ticks[j];
      b.u8(static_cast<std::uint8_t>(LogRecord::kTick));
      b.u64(t.tick);
      b.u64(source_.stamps()[j]);
      b.bytes(&source_.imus()[j], sizeof(marv_imu_meas));  // the sample passed to the SIL for this tick
      for (const std::uint16_t d : t.dshot) {
        b.u16(d);
      }
      b.u32(t.out.erpm_valid);
      b.u32(0);
      put3(b, {t.out.force_ned_n[0], t.out.force_ned_n[1], t.out.force_ned_n[2]});
      put3(b, {t.out.torque_ned_nm[0], t.out.torque_ned_nm[1], t.out.torque_ned_nm[2]});
      for (const double c : t.out.rotor_speed_rad_s) {
        b.f64(c);
      }
      for (const double c : t.out.erpm) {
        b.f64(c);
      }
      put3(b, t.wrench_enu.force);
      put3(b, t.wrench_enu.torque);
      if (log_truth_) {
        b.u8(static_cast<std::uint8_t>(LogRecord::kTruth));
        b.u64(t.tick);
        b.bytes(&source_.truths()[j], sizeof(marv_truth_state));  // the state passed before this tick's marv_sil_tick
      }
      if (log_rotor_) {
        b.u8(static_cast<std::uint8_t>(LogRecord::kRotor));
        b.u64(t.tick);
        b.bytes(&source_.rotors()[j], sizeof(marv_rotor_speed_meas));  // the sample passed to the SIL for this tick
      }
    }
    if (r.status == Status::kOk) {
      b.u8(static_cast<std::uint8_t>(LogRecord::kApplied));
      put3(b, r.wrench_enu.force);
      put3(b, r.wrench_enu.torque);
    }
    if (log_.is_open() && !log_.write(b)) {
      refuse("cannot write the log");
    }
    ticks_ += r.ticks.size();
    if (r.status != Status::kOk) {
      switch (r.status) {
        case Status::kCommandSource:
          refuse(source_.failure());
        case Status::kPlant:
          refuse(std::string("marv_plant_step: ") + marv_plant_status_str(r.plant_status));
        default:
          refuse("adapter step failed");
      }
    }
    ++steps_;

    link_.AddWorldWrench(ecm, gzm::Vector3d(r.wrench_enu.force[0], r.wrench_enu.force[1], r.wrench_enu.force[2]),
                         gzm::Vector3d(r.wrench_enu.torque[0], r.wrench_enu.torque[1], r.wrench_enu.torque[2]));

    // First step only. gz-sim 8.15 Link::SetLinearVelocity and SetAngularVelocity take vectors in the link frame (the
    // FLU body frame here), not the world frame their header text suggests: measured, see the run report. The
    // linear velocity is the ENU world velocity rotated into the link frame; the angular velocity is the FLU body
    // rate itself (an exact map of the FRD rate).
    if (steps_ == 1) {
      if (v_ned_) {
        const Vec3 v = ned_to_enu_world(*v_ned_);
        link_.SetLinearVelocity(ecm, pose.Rot().RotateVectorReverse(gzm::Vector3d(v[0], v[1], v[2])));
      }
      if (w_frd_) {
        const Vec3 flu = frd_to_flu_body(*w_frd_);
        link_.SetAngularVelocity(ecm, gzm::Vector3d(flu[0], flu[1], flu[2]));
      }
      vel_cmd_pending_ = true;
    }
  }

  void Reset(const gzs::UpdateInfo& /*info*/, gzs::EntityComponentManager& /*ecm*/) override {
    refuse("reset is not supported (one SIL init per process)");
  }

 private:
  // The physics element is not in the entity-component manager while model plugins are configured, so its check is
  // made at the first unpaused PreUpdate, before any state is read, any log is written or any wrench applied.
  void check_physics(gzs::EntityComponentManager& ecm) {
    const gzs::Entity world = ecm.EntityByComponents(gzs::components::World());
    const auto* phys = ecm.Component<gzs::components::Physics>(world);
    if (phys == nullptr) {
      refuse("the world has no physics component");
    }
    const double want_step = static_cast<double>(host_ns_) / kNanosecondsPerSecond;
    if (!same_bits(phys->Data().MaxStepSize(), want_step)) {
      if (clock_) {
        refuse("physics max_step_size is not the realised host step n_true = " + std::to_string(host_ns_) +
               " ns (clock corner " + std::to_string(clock_corner_) + ")");
      }
      refuse("physics max_step_size is not m*tick");
    }
  }

  // The sensor elements of decision 0019 (each optional). Without any of them this changes nothing: host_ns_ stays
  // m * tick_ns and sensors_ false, so the plugin builds the three-argument Adapter and writes no new record.
  void check_sensors(const Parsed& p) {
    if (p.model_gyro && !p.imu_model) {
      refuse("<gyro_source>model</gyro_source> needs an <imu_model> element");
    }
    if (p.imu_model && !p.model_gyro) {
      refuse("<imu_model> needs <gyro_source>model</gyro_source>");
    }
    if (p.rotor_speed) {
      if (!p.model_gyro) {
        refuse("<rotor_speed_model> needs <gyro_source>model</gyro_source>");
      }
      if (p.rotor_speed->latency_ticks > MARV_PLANT_ROTOR_SPEED_MAX_LATENCY_TICKS) {
        refuse("<rotor_speed_model> latency_ticks " + std::to_string(p.rotor_speed->latency_ticks) +
               " is above the plant's delay-line capacity MARV_PLANT_ROTOR_SPEED_MAX_LATENCY_TICKS = " +
               std::to_string(MARV_PLANT_ROTOR_SPEED_MAX_LATENCY_TICKS));
      }
      if (p.plant.pole_count == 0) {
        refuse("<rotor_speed_model> needs a nonzero <pole_count> (0 is unknown)");
      }
    }
    if (p.clock_corner.has_value() != p.odr_error.has_value()) {
      refuse("<clock_corner> and <odr_error> go together; one of them is missing");
    }
    if (p.clock_corner) {
      const std::int32_t c = *p.clock_corner;
      const double e = *p.odr_error;
      if (c < -1 || c > 1) {
        refuse("<clock_corner> is " + std::to_string(c) + ", not -1, 0 or +1");
      }
      if (!std::isfinite(e) || e < 0.0 || !(e < 1.0)) {
        refuse("<odr_error> is not a finite magnitude in [0, 1)");
      }
      // Exists only for the clock T1 test, tests/regression/quad/L06/gz/test_sensor_clock.py (decision 0019, ruling on
      // Q6): with it set, a clock corner is accepted with the truth gyro or the zeroed sample. Never set outside that test;
      // tests/regression/quad/L06/tools/test_world_sensors.py checks that no other file names it.
      if (!p.model_gyro && std::getenv("MARV_GZ_TEST_CLOCK_CORNER_ANY_GYRO") == nullptr) {
        refuse("<clock_corner> needs <gyro_source>model</gyro_source> (the clock error is the IMU's ODR error)");
      }
      host_ns_ = realised_host_step_ns(host_ns_, c, e);
      clock_ = true;
      clock_corner_ = c;
      odr_error_ = e;
    }
    sensors_ = p.model_gyro || p.rotor_speed.has_value() || clock_;
  }

  // The adapter of a world with a sensor element (decision 0019), the source's model path and record 6.
  void start_sensors(marv_plant* plant, const Parsed& p) {
    marv_plant_rotor_speed_config rotor{};
    if (p.rotor_speed) {
      rotor = *p.rotor_speed;
      rotor.pole_count = p.plant.pole_count;
    }
    AdapterConfig ac;
    ac.t_tick_nominal_s = t_tick_s_;
    ac.imu = p.imu_model ? &p.imu_model->cfg : nullptr;
    ac.rotor_speed = p.rotor_speed ? &rotor : nullptr;
    // t_true = n_true / (m * 1e9), rounded once: at c = 0, and without the element, the same rational as t_tick_s_.
    ac.t_tick_true_s = clock_ ? static_cast<double>(host_ns_) / (static_cast<double>(m_) * kNanosecondsPerSecond) : 0.0;
    adapter_ = std::make_unique<Adapter>(plant, source_, ac);
    if (adapter_->imu_attach_status() != MARV_PLANT_OK) {
      refuse(std::string("marv_plant_imu_attach: ") + marv_plant_status_str(adapter_->imu_attach_status()));
    }
    if (adapter_->rotor_speed_attach_status() != MARV_PLANT_OK) {
      refuse(std::string("marv_plant_rotor_speed_attach: ") + marv_plant_status_str(adapter_->rotor_speed_attach_status()));
    }
    if (p.model_gyro) {
      source_.use_model_gyro();
    }
    log_rotor_ = p.rotor_speed.has_value();

    LogBuffer& b = sensors_record_;
    b.u8(static_cast<std::uint8_t>(LogRecord::kSensors));
    b.u8(static_cast<std::uint8_t>(p.model_gyro   ? LogGyroSource::kModel
                                   : p.truth_gyro ? LogGyroSource::kTruth
                                                  : LogGyroSource::kNone));
    b.u8(log_rotor_ ? 1 : 0);
    b.u8(clock_ ? 1 : 0);
    b.u8(static_cast<std::uint8_t>(static_cast<std::int8_t>(clock_corner_)));
    b.f64(odr_error_);
    b.u64(host_ns_);
    b.f64(adapter_->t_tick_s());
    b.u64(p.seed);
    b.u32(kImuNoiseStreamId);
    b.u64(kImuCounterBase);
    const ImuModel imu = p.imu_model.value_or(ImuModel{});
    b.u32(imu.cfg.latency_samples);
    for (const marv_plant_imu_axis_config* a : {&imu.cfg.gyro, &imu.cfg.accel}) {
      b.f64(a->noise_density);
      b.f64(a->bias_instability);
      b.f64(a->lsb);
      b.f64(a->full_scale);
      for (const double v : a->turn_on_bias) {
        b.f64(v);
      }
    }
    b.f64(imu.gyro_bound);
    b.f64(imu.accel_bound);
    for (const std::int8_t s : imu.signs) {
      b.u8(static_cast<std::uint8_t>(s));
    }
    b.bytes(imu.profile_sha256.data(), imu.profile_sha256.size());
    b.u32(rotor.pole_count);
    b.u32(rotor.latency_ticks);
    b.u32(rotor.exponent_bits);
    b.u32(rotor.mantissa_bits);
    b.f64(rotor.period_unit_s);
  }

  void open_log() {
    if (!log_path_) {
      return;
    }
    std::string error;
    if (!log_.open(*log_path_, log_params_.m, log_params_.num_us, log_params_.den, log_params_.seed, GZ_SIM_VERSION_FULL,
                   error)) {
      refuse(error);
    }
    if (sensors_ && !log_.write(sensors_record_)) {
      refuse("cannot write the log");
    }
  }

  // The world checks of the Configure refusals that need the entity-component manager.
  void check_world(const gzs::Entity& entity, gzs::EntityComponentManager& ecm, const Parsed& p) {
    const gzs::Entity world = ecm.EntityByComponents(gzs::components::World());
    if (world == gzs::kNullEntity) {
      refuse("no world entity");
    }
    const auto* grav = ecm.Component<gzs::components::Gravity>(world);
    if (grav == nullptr) {
      refuse("the world has no gravity component");
    }
    const gzm::Vector3d g = grav->Data();
    if (!same_bits(g.X(), 0.0) || !same_bits(g.Y(), 0.0) || !same_bits(g.Z(), 0.0)) {
      refuse("world gravity is not zero (marv_plant applies g)");
    }
    const gzs::Model model(entity);
    if (!model.Valid(ecm)) {
      refuse("the plugin is not a child of a model");
    }
    const std::vector<gzs::Entity> links = model.Links(ecm);
    if (links.size() != 1) {
      refuse("the model does not have exactly one link");
    }
    link_ = gzs::Link(links.front());
    const auto* lpose = ecm.Component<gzs::components::Pose>(link_.Entity());
    if (lpose == nullptr || !is_identity(lpose->Data())) {
      refuse("the link origin is not the model frame (link pose is not identity)");
    }
    const auto* inertial = ecm.Component<gzs::components::Inertial>(link_.Entity());
    if (inertial == nullptr) {
      refuse("the link has no inertial");
    }
    const gzm::Inertiald& in = inertial->Data();
    if (!is_identity(in.Pose())) {
      refuse("the link's inertial pose is not identity (the CM is not the link origin)");
    }
    if (!same_bits(in.MassMatrix().Mass(), p.plant.mass_kg)) {
      refuse("SDF mass is not the plant mass_kg, bitwise");
    }
    const gzm::Vector3d off = in.MassMatrix().OffDiagonalMoments();
    const gzm::Vector3d diag = in.MassMatrix().DiagonalMoments();
    if (!same_bits(off.X(), 0.0) || !same_bits(off.Y(), 0.0) || !same_bits(off.Z(), 0.0)) {
      refuse("SDF inertia has nonzero products of inertia (the card inertia is diagonal)");
    }
    if (!(diag.X() > 0.0) || !(diag.Y() > 0.0) || !(diag.Z() > 0.0)) {
      refuse("SDF inertia diagonal is not positive");
    }
  }

  static bool is_identity(const gzm::Pose3d& q) {
    return same_bits(q.Pos().X(), 0.0) && same_bits(q.Pos().Y(), 0.0) && same_bits(q.Pos().Z(), 0.0) &&
           same_bits(q.Rot().W(), 1.0) && same_bits(q.Rot().X(), 0.0) && same_bits(q.Rot().Y(), 0.0) &&
           same_bits(q.Rot().Z(), 0.0);
  }

  std::uint32_t m_ = 1;
  std::uint64_t tick_ns_ = 0;
  double t_tick_s_ = 0.0;
  std::uint64_t steps_ = 0;
  std::uint64_t ticks_ = 0;
  bool sil_ready_ = false;
  bool started_ = false;
  bool vel_cmd_pending_ = false;
  bool keep_vel_cmd_ = false;
  bool log_truth_ = false;
  // Decision 0019: the host step in ns (m * tick_ns, or n_true with <clock_corner>), whether any sensor element is present,
  // the clock element's values, and record 6 (built in Configure, written after the header).
  std::uint64_t host_ns_ = 0;
  bool sensors_ = false;
  bool clock_ = false;
  std::int32_t clock_corner_ = 0;
  double odr_error_ = 0.0;
  bool log_rotor_ = false;
  LogBuffer sensors_record_;
  std::optional<std::string> log_path_;
  struct LogParams {
    std::uint32_t m, num_us, den;
    std::uint64_t seed;
  } log_params_{};
  gzs::Link link_{gzs::kNullEntity};
  RecordingSource source_;
  std::unique_ptr<Adapter> adapter_;
  LogFile log_;
  std::optional<Vec3> v_ned_;
  std::optional<Vec3> w_frd_;
  bool zero_first_read_ = false;
};

}  // namespace marv::gz

GZ_ADD_PLUGIN(marv::gz::Lockstep, ::gz::sim::System, ::gz::sim::ISystemConfigure, ::gz::sim::ISystemPreUpdate,
              ::gz::sim::ISystemReset)
GZ_ADD_PLUGIN_ALIAS(marv::gz::Lockstep, "marv::gz::Lockstep")
