// recovery_cause_tool: tests/regression/quad/L05/results/step_cause/step_cause_tool.cpp extended for the L5 T4 large-angle
// recovery (decision 0006 F "T4 large-angle recovery"). Additions to the sim use, each an input record:
//   init <qw> <qx> <qy> <qz> <wx> <wy> <wz>   the initial attitude (body -> NED) and body rates (FRD), binary64; default
//                                            level at rest
//   rotors <rest|hover>                      the motors' initial speed: rest (0, as marv_plant starts) or the speed of
//                                            the hover command (thrust_to_dshot of the zero-torque allocation at
//                                            l5_thrust_n, through the plant's ESC map); default rest
//   first_read_zero <n>                      ticks 0 .. n - 1 give the firmware a body rate of 0 (gyro and attitude
//                                            state) while the plant moves from the initial state: the gz step-0 read
//                                            (lockstep.cpp sets the initial rates after that read); default 0
//   set_state <tick> <qw> <qx> <qy> <qz> <wx> <wy> <wz>   the plant's attitude and body rates are set to these values at
//                                            the start of <tick>, before its read (injects a logged gz state); optional
// Modes: design, cont, quant as in step_cause_tool; the suffix _gyro adds the rigid body's -w x Jw to cont or quant.
// Rate rows add the allocation's achieved torque and saturation flags: "R k tick d1..d4 req_x req_y req_z ach_x ach_y
// ach_z flag_x flag_y flag_z".
//
// step_cause_tool (harness only, not firmware; decision 0006 F "T4 angle-mode steps"): the l5_attitude_scripted
// composition's tick (fw/compositions/l5_attitude_scripted/src/l5_attitude_scripted.cpp, tick()) mirrored step for step
// on the firmware libraries of the host-gz-l5 build, in two uses:
//
//   step_cause_tool replay <input> <output>
//     The logged run's IMU samples and TRUTH records are replayed through the composition path. One row per rate
//     execution: the rate setpoint, the torque request, the allocation's achieved torque, the DShot commands and the
//     torque those DShot commands give through the plant's static map (marv_plant: linear-in-omega ESC map, thrust
//     k omega^2, rotor geometry and yaw reaction; motors at their commanded speed), in binary64.
//
//   step_cause_tool sim <input> <mode> <substeps> <output>
//     The composition path closed around a binary64 rotational plant from level, at rest, motors stopped, over the run's
//     ticks (the same stamps floor(j num / den), the same stick segments), with <substeps> integration sub-steps a tick:
//       design       the T3 design plant (tests/regression/quad/L05/t3/t3_test.cpp Plant): per axis J w' = u_m,
//                    tau u_m' = u - u_m, u the rate loop's torque request held over the tick, exact ZOH; no w x Jw.
//       cont         the torque request allocated (mixer::allocate); each motor commanded to omega = sqrt(f / k), not
//                    quantised; motor lag d omega/dt = (omega_cmd - omega) / tau (marv_plant); thrust k omega^2 and the
//                    plant's rotor geometry give the body torque; per-axis J w' = torque, no w x Jw.
//       quant        as cont, but each motor commanded through thrust_to_dshot and the plant's DShot -> omega map
//                    (marv_plant Model::omega_cmd): the DShot quantisation inserted, nothing else changed.
//       quant_gyro   as quant, with J w' = torque - w x J w (the rigid body's gyroscopic term).
//     The quaternion is integrated as the T3 oracle does: q <- q (x) exp(d) with the sub-step's angle increment d.
//     Output: "A a tick t_us qw qx qy qz wx wy wz" per attitude execution (q the binary32 truth the firmware reads,
//     canonical w >= 0; w the plant's body rates in binary64), "R k tick d1 d2 d3 d4 req_x req_y req_z" per rate execution.
//
// Input (text, step_cause.py writes it):
//   override <name> <f32|i32> <text>        every sil_override of the logged run (the composition's scenario values)
//   plant <mass> <k> <torque_ratio> <omega_min> <omega_max> <tau>   the card's plant fields (as the world passes them)
//   rotor <i> <x> <y> <z> <yaw_sign>        rotor position FRD (m) and yaw sign, logical motor i = 1..4
//   inertia <jxx> <jyy> <jzz>               the card's diagonal inertia (kg m^2)
//   ticks <n>                               sim: number of ticks
//   tick <n> <t_us> <g0> <g1> <g2> <flags>  replay: each SIL tick, gyro as binary32 bit patterns (hex)
//   truth <n> <qw> <qx> <qy> <qz> <wx> <wy> <wz>   replay: the TRUTH record of tick n (binary32 bit patterns, hex)
#include <array>
#include <bit>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <map>
#include <optional>
#include <span>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

#include <marv/attitude/angle_mode.hpp>
#include <marv/attitude/attitude_law.hpp>
#include <marv/attitude/config.hpp>
#include <marv/l5_script.hpp>
#include <marv/mixer/mixer.hpp>
#include <marv/params/param.hpp>
#include <marv/prim/constants.hpp>
#include <marv/prim/quat.hpp>
#include <marv/prim/vec.hpp>
#include <marv/rate/rate_loop.hpp>
#include <marv/sched/rate_groups.hpp>
#include <marv/types/attitude_state.hpp>
#include <marv/types/imu_sample.hpp>

#include "param_override.hpp"

namespace {

using namespace marv;
using Vec3f = prim::Vec3<float>;
constexpr std::size_t kM = mixer::kMotors;

struct SegmentIds {
  ParamId t_us, roll, pitch, yaw;
};
// l5_attitude_scripted.cpp kSegmentIds.
constexpr std::array kSegmentIds{
    SegmentIds{ParamId::l5_seg1_t_us, ParamId::l5_seg1_roll, ParamId::l5_seg1_pitch, ParamId::l5_seg1_yaw},
    SegmentIds{ParamId::l5_seg2_t_us, ParamId::l5_seg2_roll, ParamId::l5_seg2_pitch, ParamId::l5_seg2_yaw},
    SegmentIds{ParamId::l5_seg3_t_us, ParamId::l5_seg3_roll, ParamId::l5_seg3_pitch, ParamId::l5_seg3_yaw},
    SegmentIds{ParamId::l5_seg4_t_us, ParamId::l5_seg4_roll, ParamId::l5_seg4_pitch, ParamId::l5_seg4_yaw},
    SegmentIds{ParamId::l5_seg5_t_us, ParamId::l5_seg5_roll, ParamId::l5_seg5_pitch, ParamId::l5_seg5_yaw},
    SegmentIds{ParamId::l5_seg6_t_us, ParamId::l5_seg6_roll, ParamId::l5_seg6_pitch, ParamId::l5_seg6_yaw},
    SegmentIds{ParamId::l5_seg7_t_us, ParamId::l5_seg7_roll, ParamId::l5_seg7_pitch, ParamId::l5_seg7_yaw},
    SegmentIds{ParamId::l5_seg8_t_us, ParamId::l5_seg8_roll, ParamId::l5_seg8_pitch, ParamId::l5_seg8_yaw}};
using Script = composition::StickScript<float, kSegmentIds.size()>;
enum Group : std::size_t { kAttitude, kRate, kGroupCount };

int fail(const std::string& why) {
  std::cerr << "step_cause_tool: " << why << '\n';
  return 1;
}

std::string num(double x) {
  std::array<char, 32> b{};
  const auto r = std::to_chars(b.data(), b.data() + b.size(), x);
  return std::string(b.data(), r.ptr);
}

float hexf(const std::string& s) {
  std::uint32_t bits = 0;
  std::from_chars(s.data(), s.data() + s.size(), bits, 16);
  return std::bit_cast<float>(bits);
}

std::optional<std::uint32_t> param_id_of(const std::string& name) {
  for (std::size_t i = 0; i < kParamCount; ++i) {
    if (name == param_name(static_cast<ParamId>(i))) {
      return static_cast<std::uint32_t>(i);
    }
  }
  return std::nullopt;
}

struct PlantCard {
  double mass = 0, k = 0, ratio = 0, wmin = 0, wmax = 0, tau = 0;
  std::array<std::array<double, 3>, kM> r{};
  std::array<double, kM> sign{};
  std::array<double, 3> j{};
};

// marv_plant Model::omega_cmd in binary64.
double omega_cmd(const PlantCard& p, std::uint16_t d) {
  if (d == 0) {
    return 0;
  }
  const double span = static_cast<double>(prim::kDshotThrottleMax - prim::kDshotThrottleMin);
  return p.wmin + (p.wmax - p.wmin) * (static_cast<double>(d - prim::kDshotThrottleMin) / span);
}

// marv_plant Model::wrench's body torque (FRD) from the motor speeds.
std::array<double, 3> body_torque(const PlantCard& p, const std::array<double, kM>& w) {
  std::array<double, 3> t{};
  for (std::size_t i = 0; i < kM; ++i) {
    const double f = p.k * w[i] * w[i];
    // r x (0, 0, -f) = (-r_y f, r_x f, 0), plus the reaction torque about +z.
    t[0] += -p.r[i][1] * f;
    t[1] += p.r[i][0] * f;
    t[2] += p.sign[i] * p.ratio * f;
  }
  return t;
}

struct Qd {
  double w = 1, x = 0, y = 0, z = 0;
};
Qd mul(const Qd& a, const Qd& b) {
  return Qd{a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}
Qd expq(const std::array<double, 3>& phi) {
  const double a = std::sqrt(phi[0] * phi[0] + phi[1] * phi[1] + phi[2] * phi[2]);
  if (a == 0) {
    return Qd{};
  }
  const double s = std::sin(a / 2) / a;
  return Qd{std::cos(a / 2), phi[0] * s, phi[1] * s, phi[2] * s};
}

// The T3 design plant's exact ZOH step (t3_test.cpp Plant).
struct AxisPlant {
  double j = 1, tau = 1, h = 0, w = 0, um = 0;
  double step(double u) {
    const double x = h / tau;
    const double e = std::exp(-x);
    const double em = -std::expm1(-x);
    const double g = h - tau * em;
    const double d = h * w + (tau * g * um + (h * h / 2 - tau * g) * u) / j;
    w += (tau * em * um + g * u) / j;
    um = e * um + em * u;
    return d;
  }
};

// The composition, configured from the build's table with the run's overrides (composition::init).
struct Composition {
  rate::RateLoop<float> rate;
  mixer::MixerConfig<float> mix{};
  attitude::AngleMode<float> angle;
  attitude::AttitudeLaw<float> law;
  sched::RateGroups<kGroupCount> groups;
  Script script{};
  composition::Chirp<float> chirp{};
  composition::Disturbance<float> dist{};
  float thrust = 0;
  Vec3f sp;

  bool init() {
    const rate::RateConfig<float> rc = rate::load_config();
    mix = mixer::load_config();
    rate.init(rc, mix);
    const attitude::AttitudeConfig<float> ac = attitude::load_config();
    angle.init(ac);
    law.init(ac);
    const auto div = static_cast<std::uint32_t>(param_value<ParamId::rate_loop_divisor>());
    const auto ratio = static_cast<std::uint32_t>(param_value<ParamId::att_loop_ratio>());
    if (!groups.init({div * ratio, div})) {
      return false;
    }
    thrust = param_value<ParamId::l5_thrust_n>();
    script.count = param_value<ParamId::l5_seg_count>();
    for (std::size_t k = 0; k < kSegmentIds.size(); ++k) {
      script.segment[k].t_us = param_get(kSegmentIds[k].t_us).value.i32;
      script.segment[k].stick = Vec3f(param_get(kSegmentIds[k].roll).value.f32, param_get(kSegmentIds[k].pitch).value.f32,
                                      param_get(kSegmentIds[k].yaw).value.f32);
    }
    chirp.axis = static_cast<composition::ChirpAxis>(param_value<ParamId::l5_chirp_axis>());
    chirp.amp = param_value<ParamId::l5_chirp_amp_rad_s>();
    chirp.w_lo = param_value<ParamId::l5_chirp_w_lo>();
    chirp.w_hi = param_value<ParamId::l5_chirp_w_hi>();
    chirp.t0_us = param_value<ParamId::l5_chirp_t0_us>();
    chirp.dur_us = param_value<ParamId::l5_chirp_dur_us>();
    dist.yaw_nm = param_value<ParamId::l5_dist_yaw_nm>();
    dist.t0_us = param_value<ParamId::l5_dist_t0_us>();
    return composition::validate(script) == composition::ScriptError::None &&
           composition::validate(chirp) == composition::ScriptError::None &&
           composition::validate(dist) == composition::ScriptError::None;
  }

  struct Out {
    bool rate_ran = false;
    Vec3f request;
    mixer::Allocation<float> alloc{};
    std::array<DshotValue, kM> dshot{};
    bool fault = false;
  };

  // composition::tick, the latched attitude passed in.
  Out tick(Tick n, const ImuSample& s, const AttitudeState<float>& latch) {
    Out o;
    const std::uint32_t due = groups.due(n);
    if ((due & (1U << kAttitude)) != 0) {
      const Vec3f stick = composition::sticks_at(script, s.t_us);
      const attitude::AngleOutput<float> a =
          angle.execute(attitude::AngleSticks<float>{stick[0], stick[1], stick[2]}, latch);
      sp = law.execute(latch, a.q_sp, a.yaw_rate_cmd).rate_setpoint;
    }
    if ((due & (1U << kRate)) == 0) {
      return o;
    }
    const rate::RateOutput<float> r = rate.execute_bypass(s, sp + composition::chirp_rate(chirp, s.t_us));
    o.request = r.torque + composition::disturbance_torque(dist, s.t_us);
    o.alloc = mixer::allocate(mix, mixer::Request<float>{thrust, o.request});
    rate.record_allocation(o.request, o.alloc);
    o.dshot = mixer::thrust_to_dshot(mix, o.alloc.f);
    o.fault = r.fault_active;
    o.rate_ran = true;
    return o;
  }
};

struct Input {
  std::vector<marv_sil_param_override> overrides;
  PlantCard card;
  std::uint64_t ticks = 0;
  std::vector<std::pair<Tick, ImuSample>> samples;
  std::array<double, 4> q0{1, 0, 0, 0};
  std::array<double, 3> w0{};
  bool rotors_hover = false;
  std::uint64_t first_read_zero = 0;
  std::uint64_t set_tick = 0;
  bool set_state = false;
  std::array<double, 7> set_value{};
  std::map<Tick, AttitudeState<float>> truth;
};

bool read_input(const char* path, Input& in, std::string& err) {
  std::ifstream f(path);
  if (!f) {
    err = std::string("cannot open ") + path;
    return false;
  }
  std::string line;
  while (std::getline(f, line)) {
    std::istringstream w(line);
    std::string kind;
    if (!(w >> kind)) {
      continue;
    }
    if (kind == "override") {
      std::string name, type, text;
      w >> name >> type >> text;
      const auto id = param_id_of(name);
      if (!id) {
        err = "unknown parameter " + name;
        return false;
      }
      marv_sil_param_override o{};
      o.id = *id;
      if (type == "f32") {
        o.type = MARV_PARAM_F32;
        std::from_chars(text.data(), text.data() + text.size(), o.f32);
      } else {
        o.type = MARV_PARAM_I32;
        std::from_chars(text.data(), text.data() + text.size(), o.i32);
      }
      o.sigma = 0.0F;
      in.overrides.push_back(o);
    } else if (kind == "plant") {
      w >> in.card.mass >> in.card.k >> in.card.ratio >> in.card.wmin >> in.card.wmax >> in.card.tau;
    } else if (kind == "rotor") {
      std::size_t i = 0;
      w >> i;
      w >> in.card.r[i - 1][0] >> in.card.r[i - 1][1] >> in.card.r[i - 1][2] >> in.card.sign[i - 1];
    } else if (kind == "inertia") {
      w >> in.card.j[0] >> in.card.j[1] >> in.card.j[2];
    } else if (kind == "init") {
      w >> in.q0[0] >> in.q0[1] >> in.q0[2] >> in.q0[3] >> in.w0[0] >> in.w0[1] >> in.w0[2];
    } else if (kind == "rotors") {
      std::string m;
      w >> m;
      in.rotors_hover = m == "hover";
    } else if (kind == "set_state") {
      w >> in.set_tick;
      for (double& v : in.set_value) w >> v;
      in.set_state = true;
    } else if (kind == "first_read_zero") {
      w >> in.first_read_zero;
    } else if (kind == "ticks") {
      w >> in.ticks;
    } else if (kind == "tick") {
      Tick n = 0;
      ImuSample s{};
      std::string g0, g1, g2;
      w >> n >> s.t_us >> g0 >> g1 >> g2 >> s.flags;
      s.gyro_rad_s = Vec3f(hexf(g0), hexf(g1), hexf(g2));
      in.samples.emplace_back(n, s);
    } else if (kind == "truth") {
      Tick n = 0;
      std::array<std::string, 7> h;
      w >> n;
      for (auto& x : h) {
        w >> x;
      }
      AttitudeState<float> a{};
      a.q = prim::Quat<float>(hexf(h[0]), hexf(h[1]), hexf(h[2]), hexf(h[3]));
      a.omega_frd = Vec3f(hexf(h[4]), hexf(h[5]), hexf(h[6]));
      a.valid = true;
      in.truth[n] = a;
    } else {
      err = "unknown record " + kind;
      return false;
    }
  }
  return true;
}

bool init_params(const Input& in) {
  std::array<ParamRecord, kParamCount> merged{};
  const sil::OverrideStatus st = sil::apply_overrides(
      param_defaults(), std::span<const marv_sil_param_override>{in.overrides}, std::span<ParamRecord, kParamCount>{merged});
  return st.ok && params_init(std::span<const ParamRecord, kParamCount>{merged});
}

std::array<double, 3> static_torque(const PlantCard& p, const std::array<DshotValue, kM>& d) {
  std::array<double, kM> w{};
  for (std::size_t i = 0; i < kM; ++i) {
    w[i] = omega_cmd(p, d[i].raw());
  }
  return body_torque(p, w);
}

int replay(const Input& in, const char* out_path) {
  Composition c;
  if (!c.init()) {
    return fail("composition init");
  }
  std::ofstream out(out_path);
  out << "k tick t_us sp_x sp_y sp_z req_x req_y req_z ach_x ach_y ach_z flag_x flag_y flag_z d1 d2 d3 d4 qt_x qt_y qt_z "
         "fault\n";
  AttitudeState<float> latch{};
  std::uint64_t k = 0;
  for (const auto& [n, s] : in.samples) {
    const auto t = in.truth.find(n);
    if (t != in.truth.end()) {
      latch = t->second;
      latch.t_us = s.t_us;
    }
    const Composition::Out o = c.tick(n, s, latch);
    if (!o.rate_ran) {
      continue;
    }
    const auto qt = static_torque(in.card, o.dshot);
    out << k++ << ' ' << n << ' ' << s.t_us;
    for (std::size_t a = 0; a < 3; ++a) out << ' ' << num(static_cast<double>(c.sp[a]));
    for (std::size_t a = 0; a < 3; ++a) out << ' ' << num(static_cast<double>(o.request[a]));
    for (std::size_t a = 0; a < 3; ++a) out << ' ' << num(static_cast<double>(o.alloc.achieved_torque[a]));
    out << ' ' << int{o.alloc.flags.roll} << ' ' << int{o.alloc.flags.pitch} << ' ' << int{o.alloc.flags.yaw};
    for (const DshotValue d : o.dshot) out << ' ' << d.raw();
    for (std::size_t a = 0; a < 3; ++a) out << ' ' << num(qt[a]);
    out << ' ' << int{o.fault} << '\n';
  }
  return out ? 0 : fail("write error");
}

int sim(const Input& in, const std::string& mode, unsigned substeps, const char* out_path) {
  Composition c;
  if (!c.init()) {
    return fail("composition init");
  }
  const bool design = mode == "design";
  const bool quant = mode == "quant" || mode == "quant_gyro";
  const bool gyro = mode == "quant_gyro" || mode == "cont_gyro";
  if (!design && !quant && mode != "cont" && mode != "cont_gyro") {
    return fail("unknown mode " + mode);
  }
  const auto num_us = static_cast<std::uint64_t>(param_value<ParamId::tick_period_num_us>());
  const auto den = static_cast<std::uint64_t>(param_value<ParamId::tick_period_den>());
  const double tick_s = static_cast<double>(num_us) / static_cast<double>(den) / prim::kMicrosecondsPerSecond;
  const double hs = tick_s / substeps;
  const auto att_div = static_cast<std::uint64_t>(param_value<ParamId::rate_loop_divisor>()) *
                       static_cast<std::uint64_t>(param_value<ParamId::att_loop_ratio>());
  const PlantCard& p = in.card;
  std::array<AxisPlant, 3> ax;
  for (std::size_t a = 0; a < 3; ++a) {
    ax[a].j = p.j[a];
    ax[a].tau = static_cast<double>(param_value<ParamId::motor_tau>());
    ax[a].h = hs;
  }
  Qd q{in.q0[0], in.q0[1], in.q0[2], in.q0[3]};
  std::array<double, 3> wb = in.w0;
  for (std::size_t a = 0; a < 3; ++a) ax[a].w = in.w0[a];
  std::array<double, kM> wm{};
  std::array<double, kM> wcmd{};
  if (in.rotors_hover) {
    const auto hover = mixer::thrust_to_dshot(c.mix, mixer::allocate(c.mix, mixer::Request<float>{c.thrust, Vec3f()}).f);
    for (std::size_t i = 0; i < kM; ++i) wm[i] = omega_cmd(p, hover[i].raw());
  }
  std::array<double, 3> u{};
  std::ofstream out(out_path);
  std::uint64_t k = 0;
  for (Tick n = 0; n < in.ticks; ++n) {
    if (in.set_state && n == in.set_tick) {
      q = Qd{in.set_value[0], in.set_value[1], in.set_value[2], in.set_value[3]};
      for (std::size_t a = 0; a < 3; ++a) wb[a] = ax[a].w = in.set_value[4 + a];
    }
    ImuSample s{};
    s.t_us = n * num_us / den;
    s.flags = imu_flag(ImuFlag::GyroValid);
    for (std::size_t a = 0; a < 3; ++a) {
      s.gyro_rad_s[a] = n < in.first_read_zero ? 0.0F : static_cast<float>(design ? ax[a].w : wb[a]);
    }
    AttitudeState<float> latch{};
    latch.t_us = s.t_us;
    latch.q = prim::Quat<float>(static_cast<float>(q.w), static_cast<float>(q.x), static_cast<float>(q.y),
                                static_cast<float>(q.z));
    if (latch.q.w < 0.0F) {
      latch.q = prim::Quat<float>(-latch.q.w, -latch.q.x, -latch.q.y, -latch.q.z);
    }
    latch.omega_frd = s.gyro_rad_s;
    latch.valid = true;
    if (n % att_div == 0) {
      out << "A " << n / att_div << ' ' << n << ' ' << s.t_us << ' ' << num(static_cast<double>(latch.q.w)) << ' '
          << num(static_cast<double>(latch.q.x)) << ' ' << num(static_cast<double>(latch.q.y)) << ' '
          << num(static_cast<double>(latch.q.z));
      for (std::size_t a = 0; a < 3; ++a) out << ' ' << num(n < in.first_read_zero ? 0.0 : (design ? ax[a].w : wb[a]));
      out << '\n';
    }
    const Composition::Out o = c.tick(n, s, latch);
    if (o.rate_ran) {
      for (std::size_t a = 0; a < 3; ++a) u[a] = static_cast<double>(o.request[a]);
      for (std::size_t i = 0; i < kM; ++i) {
        wcmd[i] = quant ? omega_cmd(p, o.dshot[i].raw()) : std::sqrt(static_cast<double>(o.alloc.f[i]) / p.k);
      }
      out << "R " << k++ << ' ' << n;
      for (const DshotValue d : o.dshot) out << ' ' << d.raw();
      for (std::size_t a = 0; a < 3; ++a) out << ' ' << num(u[a]);
      for (std::size_t a = 0; a < 3; ++a) out << ' ' << num(static_cast<double>(o.alloc.achieved_torque[a]));
      out << ' ' << int{o.alloc.flags.roll} << ' ' << int{o.alloc.flags.pitch} << ' ' << int{o.alloc.flags.yaw} << '\n';
    }
    for (unsigned sub = 0; sub < substeps; ++sub) {
      std::array<double, 3> d{};
      if (design) {
        for (std::size_t a = 0; a < 3; ++a) d[a] = ax[a].step(u[a]);
      } else {
        const double e = std::exp(-hs / p.tau);
        std::array<double, kM> w1{};
        std::array<double, kM> wmid{};
        const double emid = std::exp(-hs / 2 / p.tau);
        for (std::size_t i = 0; i < kM; ++i) {
          w1[i] = wcmd[i] + (wm[i] - wcmd[i]) * e;
          wmid[i] = wcmd[i] + (wm[i] - wcmd[i]) * emid;
        }
        const auto t0 = body_torque(p, wm);
        const auto tm = body_torque(p, wmid);
        const auto t1 = body_torque(p, w1);
        // RK4 on J w' = t(s) - [w x J w], t(s) from the exact motor speeds at the sub-step's start, middle and end.
        auto f = [&](const std::array<double, 3>& t, const std::array<double, 3>& w) {
          std::array<double, 3> r{};
          std::array<double, 3> g{};
          if (gyro) {
            const std::array<double, 3> jw{p.j[0] * w[0], p.j[1] * w[1], p.j[2] * w[2]};
            g = {w[1] * jw[2] - w[2] * jw[1], w[2] * jw[0] - w[0] * jw[2], w[0] * jw[1] - w[1] * jw[0]};
          }
          for (std::size_t a = 0; a < 3; ++a) r[a] = (t[a] - g[a]) / p.j[a];
          return r;
        };
        auto add = [](const std::array<double, 3>& a, const std::array<double, 3>& b, double h) {
          return std::array<double, 3>{a[0] + h * b[0], a[1] + h * b[1], a[2] + h * b[2]};
        };
        const auto k1 = f(t0, wb);
        const auto k2 = f(tm, add(wb, k1, hs / 2));
        const auto k3 = f(tm, add(wb, k2, hs / 2));
        const auto k4 = f(t1, add(wb, k3, hs));
        // Angle increment: the integral of w over the sub-step (Simpson on the RK4 stages).
        const auto wmid_b = add(wb, k2, hs / 2);
        std::array<double, 3> wn{};
        for (std::size_t a = 0; a < 3; ++a) {
          wn[a] = wb[a] + hs / 6 * (k1[a] + 2 * k2[a] + 2 * k3[a] + k4[a]);
          d[a] = hs / 6 * (wb[a] + 4 * wmid_b[a] + wn[a]);
        }
        wb = wn;
        wm = w1;
      }
      q = mul(q, expq(d));
    }
    const double nq = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    q = Qd{q.w / nq, q.x / nq, q.y / nq, q.z / nq};
  }
  return out ? 0 : fail("write error");
}

}  // namespace

int main(int argc, char** argv) {
  const std::string use = argc > 1 ? argv[1] : "";
  if (!((use == "replay" && argc == 4) || (use == "sim" && argc == 6))) {
    return fail("usage: step_cause_tool replay <input> <output> | sim <input> <mode> <substeps> <output>");
  }
  Input in;
  std::string err;
  if (!read_input(argv[2], in, err)) {
    return fail(err);
  }
  if (!init_params(in)) {
    return fail("apply_overrides or params_init rejected the table");
  }
  if (use == "replay") {
    return replay(in, argv[3]);
  }
  return sim(in, argv[3], static_cast<unsigned>(std::stoul(argv[4])), argv[5]);
}
