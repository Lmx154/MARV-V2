// l4_acro_replay: a deterministic host replay of the l4_rate_scripted composition on the IMU samples of a logged run
// (harness only, not firmware). It exposes what the SIL ABI does not output: the torque request, the L3 allocation
// (achieved torque and thrust, saturation flags) and the DShot commands of every rate-loop execution.
//
//   l4_acro_replay <input> <output>
//
// Input (text, tools/sim/run_l4.py replay_input writes it from the run's lockstep log):
//   override <name> <f32|i32> <text>       every sil_override of the run, as the plugin passes them to marv_sil_init
//   tick <n> <t_us> <g0> <g1> <g2> <a0> <a1> <a2> <temp> <flags>
//                                          one line per SIL tick n = 0, 1, ... in order: the stamp the SIL gave the
//                                          tick and the marv_imu_meas it received, each float as its 8-hex-digit
//                                          IEEE-754 binary32 bit pattern, flags in decimal
// Parameters. The overrides go through the SIL init's own mechanism: marv::sil::apply_overrides (fw/sil/src/
// param_override.cpp, compiled into this tool) on this build's default table, sigma 0 as the plugin passes it, then
// params_init. The parameter runtime linked first is marv_params_l4_rate_scripted's, as in the SIL library.
//
// Output (text): a header line of column names, then one row per rate-loop execution k (tick D k):
//   k tick t_us sp_{roll,pitch,yaw} req_{roll,pitch,yaw} ach_{roll,pitch,yaw} flag_{roll,pitch,yaw} s t ach_thrust
//   dshot1..dshot4 fault
// sp the setpoint, req the torque request passed to allocate (rate output + chirp), ach the allocation's achieved
// torque, flag its saturation flags (0/1), ach_thrust its achieved collective thrust, dshot the rate group's DshotDiffuser
// commands (decision 0017),
// fault the rate loop's fault_active (0/1). Floats are written as the shortest decimal of their exact binary64 value,
// so reading one back as a double gives the binary32 value exactly. s and t are not outputs of allocate: they are
// derived as ach/req in binary64 (roll, else pitch, for s; yaw for t), nan where the request is zero.
//
// Exit status: 0 on success, 1 on a usage or input error (message on stderr); a hal_panic aborts.
#include <array>
#include <bit>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <span>
#include <sstream>
#include <string>
#include <system_error>
#include <type_traits>
#include <vector>

#include <marv/hal/tick.hpp>
#include <marv/l4_script.hpp>
#include <marv/mixer/mixer.hpp>
#include <marv/params/param.hpp>
#include <marv/prim/constants.hpp>
#include <marv/prim/vec.hpp>
#include <marv/rate/rate_loop.hpp>
#include <marv/rate_group/rate_group.hpp>
#include <marv/sched/rate_groups.hpp>
#include <marv/types/imu_sample.hpp>

#include "param_override.hpp"

namespace {

using namespace marv;
using Vec3f = prim::Vec3<float>;

constexpr int kHex = 16;
constexpr int kDecimal = 10;
// The longest shortest-round-trip binary64 text is 24 characters (sign, 17 digits, point, "e-308"); room to spare.
constexpr std::size_t kNumberBuffer = 32;
constexpr std::size_t kImuFloats = 7;  // marv_imu_meas: gyro 3, accel 3, temperature

// The segment ids of the composition, in its kSegmentIds order (l4_rate_scripted.cpp lines 40-48).
struct SegmentIds {
  ParamId t_us;
  ParamId roll;
  ParamId pitch;
  ParamId yaw;
};

constexpr std::array kSegmentIds{
    SegmentIds{ParamId::l4_seg1_t_us, ParamId::l4_seg1_roll, ParamId::l4_seg1_pitch, ParamId::l4_seg1_yaw},
    SegmentIds{ParamId::l4_seg2_t_us, ParamId::l4_seg2_roll, ParamId::l4_seg2_pitch, ParamId::l4_seg2_yaw},
    SegmentIds{ParamId::l4_seg3_t_us, ParamId::l4_seg3_roll, ParamId::l4_seg3_pitch, ParamId::l4_seg3_yaw},
    SegmentIds{ParamId::l4_seg4_t_us, ParamId::l4_seg4_roll, ParamId::l4_seg4_pitch, ParamId::l4_seg4_yaw},
    SegmentIds{ParamId::l4_seg5_t_us, ParamId::l4_seg5_roll, ParamId::l4_seg5_pitch, ParamId::l4_seg5_yaw},
    SegmentIds{ParamId::l4_seg6_t_us, ParamId::l4_seg6_roll, ParamId::l4_seg6_pitch, ParamId::l4_seg6_yaw},
    SegmentIds{ParamId::l4_seg7_t_us, ParamId::l4_seg7_roll, ParamId::l4_seg7_pitch, ParamId::l4_seg7_yaw},
    SegmentIds{ParamId::l4_seg8_t_us, ParamId::l4_seg8_roll, ParamId::l4_seg8_pitch, ParamId::l4_seg8_yaw}};

constexpr std::size_t kSegments = kSegmentIds.size();
enum Group : std::size_t { kRate, kGroupCount };  // the composition's rate groups (l4_rate_scripted.cpp line 55)
using Script = composition::SetpointScript<float, kSegments>;

[[nodiscard]] int fail(const std::string& reason) {
  std::cerr << "l4_acro_replay: " << reason << '\n';
  return 1;
}

template <class T>
[[nodiscard]] bool parse(const std::string& text, T& out, int base) {
  const char* first = text.data();
  const char* last = text.data() + text.size();
  std::from_chars_result r{};
  if constexpr (std::is_floating_point_v<T>) {
    r = std::from_chars(first, last, out);
    (void)base;
  } else {
    r = std::from_chars(first, last, out, base);
  }
  return !text.empty() && r.ec == std::errc{} && r.ptr == last;
}

[[nodiscard]] std::optional<std::uint32_t> param_id_of(const std::string& name) {
  for (std::size_t i = 0; i < kParamCount; ++i) {
    if (name == param_name(static_cast<ParamId>(i))) {
      return static_cast<std::uint32_t>(i);
    }
  }
  return std::nullopt;
}

[[nodiscard]] std::string number(double x) {
  std::array<char, kNumberBuffer> buf{};
  const std::to_chars_result r = std::to_chars(buf.data(), buf.data() + buf.size(), x);
  return std::string(buf.data(), r.ptr);
}

[[nodiscard]] std::string number(float x) { return number(static_cast<double>(x)); }

[[nodiscard]] double ratio(float achieved, float requested) {
  return requested != 0.0F ? static_cast<double>(achieved) / static_cast<double>(requested)
                           : std::numeric_limits<double>::quiet_NaN();
}

// Mirrors load_script (l4_rate_scripted.cpp lines 88-97).
[[nodiscard]] Script load_script() {
  Script s;
  s.count = param_value<ParamId::l4_seg_count>();
  for (std::size_t k = 0; k < kSegments; ++k) {
    s.segment[k].t_us = param_get(kSegmentIds[k].t_us).value.i32;
    s.segment[k].rate = Vec3f(param_get(kSegmentIds[k].roll).value.f32, param_get(kSegmentIds[k].pitch).value.f32,
                              param_get(kSegmentIds[k].yaw).value.f32);
  }
  return s;
}

// Mirrors load_chirp (l4_rate_scripted.cpp lines 99-108).
[[nodiscard]] composition::Chirp<float> load_chirp() {
  composition::Chirp<float> c;
  c.axis = static_cast<composition::ChirpAxis>(param_value<ParamId::l4_chirp_axis>());
  c.amp = param_value<ParamId::l4_chirp_amp_nm>();
  c.w_lo = param_value<ParamId::l4_chirp_w_lo>();
  c.w_hi = param_value<ParamId::l4_chirp_w_hi>();
  c.t0_us = param_value<ParamId::l4_chirp_t0_us>();
  c.dur_us = param_value<ParamId::l4_chirp_dur_us>();
  return c;
}

struct TickLine {
  Tick n = 0;
  ImuSample sample{};
};

[[nodiscard]] bool parse_tick(std::istringstream& in, TickLine& out) {
  std::string n;
  std::string t_us;
  std::array<std::string, kImuFloats> f;
  std::string flags;
  if (!(in >> n >> t_us)) {
    return false;
  }
  for (std::string& x : f) {
    if (!(in >> x)) {
      return false;
    }
  }
  if (!(in >> flags)) {
    return false;
  }
  std::string extra;
  if (in >> extra) {
    return false;
  }
  std::array<std::uint32_t, kImuFloats> bits{};
  if (!parse(n, out.n, kDecimal) || !parse(t_us, out.sample.t_us, kDecimal) ||
      !parse(flags, out.sample.flags, kDecimal)) {
    return false;
  }
  for (std::size_t i = 0; i < kImuFloats; ++i) {
    if (!parse(f[i], bits[i], kHex)) {
      return false;
    }
  }
  for (std::size_t i = 0; i < prim::kSpatialDim; ++i) {
    out.sample.gyro_rad_s[i] = std::bit_cast<float>(bits[i]);
    out.sample.accel_m_s2[i] = std::bit_cast<float>(bits[prim::kSpatialDim + i]);
  }
  out.sample.temp_k = std::bit_cast<float>(bits[2 * prim::kSpatialDim]);
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 3) {
    return fail("usage: l4_acro_replay <input> <output>");
  }
  std::ifstream in(argv[1]);
  if (!in) {
    return fail(std::string("cannot open ") + argv[1]);
  }

  std::vector<marv_sil_param_override> overrides;
  std::vector<TickLine> ticks;
  std::string line;
  std::size_t line_no = 0;
  while (std::getline(in, line)) {
    ++line_no;
    std::istringstream words(line);
    std::string kind;
    if (!(words >> kind)) {
      continue;
    }
    const std::string where = std::string(argv[1]) + ":" + std::to_string(line_no) + ": ";
    if (kind == "override") {
      if (!ticks.empty()) {
        return fail(where + "override after the first tick");
      }
      std::string name;
      std::string type;
      std::string text;
      std::string extra;
      if (!(words >> name >> type >> text) || (words >> extra)) {
        return fail(where + "expected: override <name> <f32|i32> <text>");
      }
      const std::optional<std::uint32_t> id = param_id_of(name);
      if (!id) {
        return fail(where + "the parameter set has no parameter '" + name + "'");
      }
      // As the plugin builds the override (sim/gz/plugin/src/lockstep.cpp): from_chars on the text, sigma 0.
      marv_sil_param_override o{};
      o.id = *id;
      if (type == "f32") {
        o.type = MARV_PARAM_F32;
        if (!parse(text, o.f32, kDecimal)) {
          return fail(where + "'" + text + "' is not a float");
        }
      } else if (type == "i32") {
        o.type = MARV_PARAM_I32;
        if (!parse(text, o.i32, kDecimal)) {
          return fail(where + "'" + text + "' is not an i32");
        }
      } else {
        return fail(where + "type '" + type + "' is not f32 or i32");
      }
      o.sigma = 0.0F;
      overrides.push_back(o);
    } else if (kind == "tick") {
      TickLine t;
      if (!parse_tick(words, t)) {
        return fail(where + "expected: tick <n> <t_us> <7 binary32 hex patterns> <flags>");
      }
      if (t.n != ticks.size()) {
        return fail(where + "tick " + std::to_string(t.n) + " is not the next tick " + std::to_string(ticks.size()));
      }
      ticks.push_back(t);
    } else {
      return fail(where + "unknown record '" + kind + "'");
    }
  }

  std::array<ParamRecord, kParamCount> merged{};
  const sil::OverrideStatus applied =
      sil::apply_overrides(param_defaults(), std::span<const marv_sil_param_override>{overrides},
                           std::span<ParamRecord, kParamCount>{merged});
  if (!applied.ok) {
    return fail("override " + std::to_string(applied.index) + " rejected by apply_overrides");
  }
  if (!params_init(std::span<const ParamRecord, kParamCount>{merged})) {
    return fail("params_init rejected the merged table");
  }

  // Mirrors composition::init (l4_rate_scripted.cpp lines 120-139); a violated rule is an input error here, where the
  // composition panics.
  const rate::RateConfig<float> rate_cfg = rate::load_config();
  const mixer::MixerConfig<float> mixer_cfg = mixer::load_config();
  const std::int32_t divisor = param_value<ParamId::rate_loop_divisor>();
  sched::RateGroups<kGroupCount> groups;
  if (divisor < 1 || !groups.init({static_cast<std::uint32_t>(divisor)})) {
    return fail("rate_loop_divisor is below 1");
  }
  rate_group::RateGroupStep step;
  step.init(rate_cfg, mixer_cfg, rate_group::load_chain_config());
  const float thrust = param_value<ParamId::l4_thrust_n>();
  if (!std::isfinite(thrust) || !(thrust >= 0.0F)) {
    return fail("l4_thrust_n is not finite and >= 0");
  }
  const Script script = load_script();
  const composition::Chirp<float> chirp = load_chirp();
  if (composition::validate(script) != composition::ScriptError::None ||
      composition::validate(chirp) != composition::ScriptError::None) {
    return fail("invalid scenario parameters (script or chirp)");
  }

  std::ofstream out(argv[2]);
  if (!out) {
    return fail(std::string("cannot open ") + argv[2]);
  }
  out << "k tick t_us sp_roll sp_pitch sp_yaw req_roll req_pitch req_yaw ach_roll ach_pitch ach_yaw flag_roll "
         "flag_pitch flag_yaw s t ach_thrust dshot1 dshot2 dshot3 dshot4 fault\n";

  // Mirrors composition::tick (l4_rate_scripted.cpp lines 141-157); its stamp check against hal_time_us is not
  // repeated (the stamps are the SIL's own, from the log).
  std::uint64_t k = 0;
  for (const TickLine& t : ticks) {
    const bool rate_due = (groups.due(t.n) & (std::uint32_t{1} << kRate)) != 0;
    step.filter(t.sample, rate_due);
    if (!rate_due) {
      continue;
    }
    const ImuSample& s = t.sample;
    const Vec3f sp = composition::setpoint_at(script, s.t_us);
    const rate_group::Execution e = step.execute(sp, composition::chirp_torque(chirp, s.t_us), thrust);
    const rate::RateOutput<float>& r = e.rate;
    const Vec3f& request = e.request;
    const mixer::Allocation<float>& alloc = e.alloc;
    const std::array<DshotValue, mixer::kMotors>& dshot = e.dshot;

    const double s_factor = request[0] != 0.0F ? ratio(alloc.achieved_torque[0], request[0])
                                               : ratio(alloc.achieved_torque[1], request[1]);
    out << k << ' ' << t.n << ' ' << s.t_us;
    for (std::size_t a = 0; a < prim::kSpatialDim; ++a) {
      out << ' ' << number(sp[a]);
    }
    for (std::size_t a = 0; a < prim::kSpatialDim; ++a) {
      out << ' ' << number(request[a]);
    }
    for (std::size_t a = 0; a < prim::kSpatialDim; ++a) {
      out << ' ' << number(alloc.achieved_torque[a]);
    }
    out << ' ' << int{alloc.flags.roll} << ' ' << int{alloc.flags.pitch} << ' ' << int{alloc.flags.yaw};
    out << ' ' << number(s_factor) << ' ' << number(ratio(alloc.achieved_torque[2], request[2]));
    out << ' ' << number(alloc.achieved_thrust);
    for (const DshotValue d : dshot) {
      out << ' ' << d.raw();
    }
    out << ' ' << int{r.fault_active} << '\n';
    ++k;
  }
  out.flush();
  if (!out) {
    return fail(std::string("write error on ") + argv[2]);
  }
  return 0;
}
