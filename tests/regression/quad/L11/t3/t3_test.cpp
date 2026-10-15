// The L11 T3 golden (quad spec 4 L11 pass bar "T3: height-step and thrust-offset responses against the design model; controls
// gains x 1.1, one tick of added delay"; decision 0020 section 5, "T3 golden (core)"): the float firmware path of the vertical
// law (vertical::VerticalLaw::execute, as fw/compositions/l11_vertical_scripted tick uses it), configured from the committed
// fixture reference/vertical_t3_inputs.txt (not from the live parameters), is closed with the design plant in double at tick
// resolution and compared with the committed double oracle reference/vertical_t3_oracle.py within the tolerance the golden
// records (the oracle's docstring defines every golden line, the per-tick order and the tolerance).
//
// Harness (the oracle's per-tick order). Tick j has the stamp floor(j num / den) us. The law executes at j = 0 mod D, D =
// rate_loop_divisor x vert_loop_ratio (the seed execution at j = 0), before the plant step of that tick, on the plant's z and v
// at the start of the tick as float32 with valid set; the command T_req + d (the sum in float, as the composition adds the thrust
// disturbance) is held for the D ticks that follow with zero computation delay. The plant is the collective rotor speed w with
// dw/dt = (w_c - w) / tau, w_c = sqrt(T_cmd / (4 k)), T = 4 k w^2, dv/dt = g - T / m, dz/dt = v (NED, z down), exact for a held
// command, g the WGS 84 normal gravity at the height h0 - z evaluated at the start of the tick (the oracle's Plant.step). The
// mixer and the allocation record (the anti-windup) stay out of the loop: the oracle refuses unless T_cmd stays inside the
// mixer's zero-torque collective range, so the freeze never acts.
//
// Numbers in this file are one of: read from the committed reference, derived (the rule is stated), or a "scenario test
// value" named with its reason.
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <marv/mixer/mixer.hpp>
#include <marv/params/param.hpp>
#include <marv/prim/constants.hpp>
#include <marv/prim/gravity.hpp>
#include <marv/types/actuator.hpp>
#include <marv/types/vertical_state.hpp>
#include <marv/vertical/config.hpp>
#include <marv/vertical/vertical_law.hpp>

namespace {

using namespace marv;
using vertical::VerticalConfig;
using vertical::VerticalLaw;

constexpr std::array<const char*, 2> kScript{"height_step", "thrust_offset"};
constexpr std::array<const char*, 3> kChannel{"t_req", "z", "v"};
// The fixture keys that are scenario values derived by the oracle's --refresh-inputs, not parameter table entries.
constexpr std::array<const char*, 2> kScenarioKeys{"hover_thrust_n", "thrust_offset_n"};

// Scenario test value: negative control (a), a gain multiplied by 1.1 (quad spec 4 L11 pass bar).
constexpr float kGainScale = 1.1F;
// Scenario test value: negative control (b), one tick of delay between the controller output and the plant (quad spec 4 L11
// pass bar).
constexpr unsigned kControlDelayTicks = 1;

// ---- the committed reference -------------------------------------------------------------------------------------------

struct Golden {
  std::size_t executions = 0;
  std::map<std::string, double> numbers;  // z_ref_m, thrust_disturbance_n, tolerance_*, ...
  std::map<std::string, std::vector<double>> channel;
};

std::string reference_path(const char* name) { return std::string(MARV_L11_T3_REFERENCE_DIR) + "/" + name; }

double parse(const std::string& text) { return std::stod(text); }  // decimal and hex floats

// Removes '#' comment lines and trailing comments.
std::vector<std::string> tokens_of(const char* name) {
  std::ifstream f(reference_path(name));
  std::vector<std::string> out;
  std::string line;
  while (std::getline(f, line)) {
    line = line.substr(0, line.find('#'));
    std::istringstream words(line);
    std::string w;
    while (words >> w) {
      out.push_back(w);
    }
  }
  return out;
}

std::map<std::string, Golden> read_golden() {
  std::map<std::string, Golden> out;
  const std::vector<std::string> t = tokens_of("vertical_t3_golden.txt");
  std::size_t i = 0;
  Golden* cur = nullptr;
  while (i < t.size()) {
    if (t[i] == "scenario") {
      cur = &out[t[i + 1]];
      i += 2;
    } else if (cur == nullptr) {
      return {};
    } else if (t[i] == "executions") {
      cur->executions = static_cast<std::size_t>(parse(t[i + 1]));
      i += 2;
    } else if (std::find(kChannel.begin(), kChannel.end(), t[i]) != kChannel.end()) {
      std::vector<double>& v = cur->channel[t[i]];
      ++i;
      for (std::size_t n = 0; n < cur->executions && i < t.size(); ++n, ++i) {
        v.push_back(parse(t[i]));
      }
    } else {
      cur->numbers[t[i]] = parse(t[i + 1]);
      i += 2;
    }
  }
  return out;
}

const std::map<std::string, Golden>& golden() {
  static const std::map<std::string, Golden> g = read_golden();
  return g;
}

// ---- the fixture -------------------------------------------------------------------------------------------------------
// reference/vertical_t3_inputs.txt: the f32 values the oracle used (hex floats, exact; the integers in decimal). Every key but
// the two scenario values equals the parameter table of marv_params_l11_vertical_scripted (checked below); the two scenario
// values are derived by the oracle's --refresh-inputs and have no table source.

using Inputs = std::map<std::string, float>;

Inputs read_inputs() {
  Inputs v;
  const std::vector<std::string> t = tokens_of("vertical_t3_inputs.txt");
  for (std::size_t i = 0; i + 1 < t.size(); i += 2) {
    v[t[i]] = static_cast<float>(parse(t[i + 1]));
  }
  return v;
}

struct Product {
  Inputs v;
  VerticalConfig<float> cfg;
  double mass = 0;
  double thrust_coeff = 0;
  double motor_tau = 0;
  double latitude = 0;
  double site_height = 0;
  std::uint64_t num_us = 0;
  std::uint64_t den = 0;
  std::uint64_t divisor = 0;  // D, ticks per vertical execution
  double tick_s = 0;
};

const Product* product() {
  static const Product* built = []() -> const Product* {
    static Product p;
    p.v = read_inputs();
    const Inputs& v = p.v;
    p.cfg.kp_z = v.at("vert_kp_z");
    p.cfg.kp_v = v.at("vert_kp_v");
    p.cfg.ki_v = v.at("vert_ki_v");
    p.cfg.hover_thrust = v.at("hover_thrust_n");
    // The period as load_vertical_config forms it: the tick count a float product, then one float quotient in microseconds,
    // then one division.
    const float ticks = v.at("vert_loop_ratio") * v.at("rate_loop_divisor");
    p.cfg.period = ticks * v.at("tick_period_num_us") / v.at("tick_period_den") /
                   static_cast<float>(prim::kMicrosecondsPerSecond);
    p.mass = static_cast<double>(v.at("mass"));
    p.thrust_coeff = static_cast<double>(v.at("rotor_thrust_coeff"));
    p.motor_tau = static_cast<double>(v.at("motor_tau"));
    p.latitude = static_cast<double>(v.at("site_latitude_rad"));
    p.site_height = static_cast<double>(v.at("site_height_m"));
    p.num_us = static_cast<std::uint64_t>(v.at("tick_period_num_us"));
    p.den = static_cast<std::uint64_t>(v.at("tick_period_den"));
    p.divisor = static_cast<std::uint64_t>(v.at("rate_loop_divisor")) * static_cast<std::uint64_t>(v.at("vert_loop_ratio"));
    p.tick_s = static_cast<double>(p.num_us) / static_cast<double>(p.den) / prim::kMicrosecondsPerSecond;
    return vertical::validate(p.cfg) == vertical::ConfigError::None ? &p : nullptr;
  }();
  return built;
}

#define REQUIRE_PRODUCT()                                                 \
  const Product* const pp = product();                                    \
  ASSERT_NE(pp, nullptr) << "reading vertical_t3_inputs.txt";             \
  [[maybe_unused]] const Product& prod = *pp;                             \
  ASSERT_EQ(golden().size(), kScript.size()) << "reading vertical_t3_golden.txt"

// The fixture keys (the scenario values excepted) that differ from the built parameter defaults, or are not in the table.
std::vector<std::string> fixture_mismatches(const Inputs& fixture) {
  std::map<std::string, float> table;
  for (std::size_t id = 0; id < kParamCount; ++id) {
    const ParamRecord& r = param_defaults()[id];
    table[param_name(static_cast<ParamId>(id))] = r.value.type == ParamType::F32 ? r.value.f32 : static_cast<float>(r.value.i32);
  }
  std::vector<std::string> bad;
  for (const auto& [key, value] : fixture) {
    if (std::find(kScenarioKeys.begin(), kScenarioKeys.end(), key) != kScenarioKeys.end()) {
      continue;
    }
    const auto it = table.find(key);
    if (it == table.end() || it->second != value) {
      bad.push_back(key);
    }
  }
  return bad;
}

// ---- the closed loop ---------------------------------------------------------------------------------------------------

// The collective rotor speed w, v and z (NED) of the design plant: the oracle's Plant.step with one sub-step.
struct Plant {
  const Product& p;
  double decay = 0;
  double i1 = 0;  // int_0^h exp(-t / tau) dt
  double i2 = 0;  // int_0^h exp(-2 t / tau) dt
  double j1 = 0;  // int_0^h int_0^t exp(-r / tau) dr dt
  double j2 = 0;  // int_0^h int_0^t exp(-2 r / tau) dr dt
  double a = 0;   // (number of rotors) k / m
  double w = 0;
  double v = 0;
  double z = 0;

  Plant(const Product& prod, double w0) : p(prod), w(w0) {
    const double h = p.tick_s;
    const double tau = p.motor_tau;
    decay = std::exp(-h / tau);
    const double e1 = -std::expm1(-h / tau);
    const double e2 = -std::expm1(-2 * h / tau);
    i1 = tau * e1;
    i2 = tau / 2 * e2;
    j1 = tau * (h - tau * e1);
    j2 = tau / 2 * (h - tau / 2 * e2);
    a = static_cast<double>(kQuadXMotors) * p.thrust_coeff / p.mass;
  }

  void step(double w_c) {
    const double h = p.tick_s;
    const double cc = w_c * w_c;
    const double g = prim::normal_gravity(p.latitude, p.site_height - z);
    const double dw = w - w_c;
    const double lin = 2 * w_c * dw;
    const double sq = dw * dw;
    const double dv = g * h - a * (cc * h + lin * i1 + sq * i2);
    const double dz = v * h + g * h * h / 2 - a * (cc * h * h / 2 + lin * j1 + sq * j2);
    z += dz;
    v += dv;
    w = w_c + dw * decay;
  }
};

struct Trace {
  std::map<std::string, std::vector<double>> channel;  // per execution, before that execution's thrust acts
  bool clean = true;                                   // the law's fault latch never set
};

// One run of a script: the first `delay_ticks` ticks after a vertical execution still apply the previous command.
Trace run_script(const Product& prod, const VerticalConfig<float>& cfg, const char* script, unsigned delay_ticks) {
  const Golden& g = golden().at(script);
  const float z_ref = static_cast<float>(g.numbers.at("z_ref_m"));
  const float dist = static_cast<float>(g.numbers.at("thrust_disturbance_n"));
  const std::size_t step_execution = static_cast<std::size_t>(g.numbers.at("step_execution"));
  VerticalLaw<float> law;
  law.init(cfg, mixer::MixerConfig<float>());
  // The hover trim: w = sqrt(T_h / (4 k)).
  const double w_hover = std::sqrt(static_cast<double>(cfg.hover_thrust) / (static_cast<double>(kQuadXMotors) * prod.thrust_coeff));
  Plant plant(prod, w_hover);
  double w_old = w_hover;
  double w_new = w_hover;
  Trace run;
  for (std::uint64_t j = 0; j < g.executions * prod.divisor; ++j) {
    const std::uint64_t k = j % prod.divisor;
    if (k == 0) {
      const std::size_t n = static_cast<std::size_t>(j / prod.divisor);
      VerticalState<float> s;
      s.t_us = j * prod.num_us / prod.den;
      s.z_ned_m = static_cast<float>(plant.z);
      s.vz_ned_m_s = static_cast<float>(plant.v);
      s.valid = true;
      const bool stepped = n >= step_execution;
      const vertical::VerticalOutput<float> out = law.execute(s, stepped ? z_ref : 0.0F);
      run.clean = run.clean && !out.fault;
      run.channel["t_req"].push_back(static_cast<double>(out.thrust_request));
      run.channel["z"].push_back(plant.z);
      run.channel["v"].push_back(plant.v);
      const float t_cmd = out.thrust_request + (stepped ? dist : 0.0F);
      w_old = w_new;
      w_new = std::sqrt(static_cast<double>(t_cmd) / (static_cast<double>(kQuadXMotors) * prod.thrust_coeff));
    }
    plant.step(k < delay_ticks ? w_old : w_new);
  }
  return run;
}

double max_abs_diff(const std::vector<double>& a, const std::vector<double>& b) {
  double m = 0;
  for (std::size_t n = 0; n < std::min(a.size(), b.size()); ++n) {
    m = std::max(m, std::abs(a[n] - b[n]));
  }
  return m;
}

// The largest ratio of a channel difference to its tolerance.
double factor(const Golden& g, const Trace& run, const char* channel) {
  return max_abs_diff(run.channel.at(channel), g.channel.at(channel)) / g.numbers.at(std::string("tolerance_") + channel);
}

// ---- 1. the fixture is the built parameter table ---------------------------------------------------------------------------

TEST(L11T3Reference, FixtureEqualsTheBuiltParameterDefaultsExceptTheScenarioValues) {
  REQUIRE_PRODUCT();
  EXPECT_EQ(fixture_mismatches(prod.v), std::vector<std::string>{});
  // The scenario values are in the fixture and agree with the golden's record of them.
  EXPECT_EQ(static_cast<double>(prod.v.at("thrust_offset_n")), golden().at("thrust_offset").numbers.at("thrust_disturbance_n"));
  // Control: one float ulp on a gain is a mismatch, so a stale fixture is detected.
  Inputs stale = prod.v;
  stale["vert_kp_v"] = std::nextafter(stale.at("vert_kp_v"), std::numeric_limits<float>::infinity());
  EXPECT_EQ(fixture_mismatches(stale), std::vector<std::string>{"vert_kp_v"});
}

// ---- 2. the golden and its negative controls -------------------------------------------------------------------------------

TEST(L11T3Step, FloatPathMatchesTheDoubleOracleWithinTheRecordedTolerance) {
  REQUIRE_PRODUCT();
  for (const char* script : kScript) {
    const Golden& g = golden().at(script);
    const Trace run = run_script(prod, prod.cfg, script, 0);
    EXPECT_TRUE(run.clean) << script << ": a fault was raised";
    for (const char* ch : kChannel) {
      ASSERT_EQ(run.channel.at(ch).size(), g.channel.at(ch).size()) << script << " " << ch;
      const double d = max_abs_diff(run.channel.at(ch), g.channel.at(ch));
      const double tol = g.numbers.at(std::string("tolerance_") + ch);
      std::printf("[T3] %-13s %-5s max|diff| = %.6e  tolerance = %.6e  (%.2f %%)\n", script, ch, d, tol, 100 * d / tol);
      EXPECT_LE(d, tol) << script << " " << ch;
    }
    // Both scripts start at rest at their reference.
    EXPECT_EQ(run.channel.at("z").front(), 0.0);
    EXPECT_EQ(run.channel.at("v").front(), 0.0);
  }
}

TEST(L11T3Step, NegativeControlEachGainTimesOnePointOneLeavesTheTolerance) {
  REQUIRE_PRODUCT();
  const std::array<float VerticalConfig<float>::*, 3> gains{&VerticalConfig<float>::kp_v, &VerticalConfig<float>::ki_v,
                                                            &VerticalConfig<float>::kp_z};
  const std::array<const char*, 3> names{"k_v", "k_i", "k_z"};
  for (std::size_t i = 0; i < gains.size(); ++i) {
    VerticalConfig<float> cfg = prod.cfg;
    cfg.*gains[i] *= kGainScale;
    for (const char* script : kScript) {
      const Golden& g = golden().at(script);
      const Trace run = run_script(prod, cfg, script, 0);
      double f = 0;
      for (const char* ch : kChannel) {
        f = std::max(f, factor(g, run, ch));
      }
      std::printf("[T3] %-13s control %s x %.1f: max(diff / tolerance) = x%.1f\n", script, names[i],
                  static_cast<double>(kGainScale), f);
      EXPECT_GT(f, 1.0) << script << ": the golden cannot detect a " << names[i] << " error";
    }
  }
}

TEST(L11T3Step, NegativeControlOneTickOfDelayLeavesTheTolerance) {
  REQUIRE_PRODUCT();
  for (const char* script : kScript) {
    const Golden& g = golden().at(script);
    const Trace run = run_script(prod, prod.cfg, script, kControlDelayTicks);
    double f = 0;
    for (const char* ch : kChannel) {
      f = std::max(f, factor(g, run, ch));
    }
    std::printf("[T3] %-13s control delay %u tick: max(diff / tolerance) = x%.1f\n", script, kControlDelayTicks, f);
    EXPECT_GT(f, 1.0) << script << ": the golden cannot detect a one-tick delay";
  }
}

}  // namespace
