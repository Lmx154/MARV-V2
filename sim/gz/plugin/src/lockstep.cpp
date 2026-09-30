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
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <marv/params/param_ids.hpp>

#include "constants.hpp"
#include "lockstep_log.hpp"
#include "marv/gz/adapter.hpp"
#include "marv/gz/frames.hpp"
#include "marv_plant.h"
#include "marv_sil.h"

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

Vec3 vec3_of(const sdf::ElementPtr& e) {
  Vec3 v{};
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

// A command source that keeps the SIL's stamp of every tick of the current host step.
class RecordingSource final : public CommandSource {
 public:
  bool dshot(std::uint64_t tick, Dshot& out) override {
    const bool ok = inner_.dshot(tick, out);
    if (ok) {
      stamps_.push_back(inner_.last_stamp_us());
    }
    return ok;
  }
  std::int32_t last_status() const { return inner_.last_status(); }
  void begin_step() { stamps_.clear(); }
  const std::vector<std::uint64_t>& stamps() const { return stamps_; }

 private:
  SilCommandSource inner_;
  std::vector<std::uint64_t> stamps_;
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

    adapter_ = std::make_unique<Adapter>(plant, source_, t_tick_s_);
    link_.EnableVelocityChecks(ecm, true);
    // Exists only for the negative control of tests/regression/quad/L02/gz/test_plugin_smoke.py: with it set, the
    // velocity command components are not removed, which is the defect the removal fixes. Never set outside that test.
    keep_vel_cmd_ = std::getenv("MARV_GZ_TEST_KEEP_VEL_CMD") != nullptr;
    v_ned_ = p.v_ned;
    w_frd_ = p.w_frd;

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
    const std::chrono::nanoseconds host_dt{m_ * tick_ns_};
    if (info.dt != host_dt) {
      refuse("UpdateInfo dt " + std::to_string(std::chrono::nanoseconds(info.dt).count()) + " ns is not m*tick " +
             std::to_string(host_dt.count()) + " ns");
    }
    const std::chrono::nanoseconds expected_sim{(steps_ + 1) * m_ * tick_ns_};
    if (info.simTime != expected_sim) {
      refuse("simTime " + std::to_string(std::chrono::nanoseconds(info.simTime).count()) + " ns is not ticks*tick " +
             std::to_string(expected_sim.count()) + " ns");
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
    const marv_plant_body body = to_plant_body(gs);

    const std::uint64_t first_tick = steps_ * m_;
    source_.begin_step();
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
      const marv_imu_meas imu{};  // the sample SilCommandSource passes: zeroed, every valid bit clear
      b.bytes(&imu, sizeof(imu));
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
          refuse(std::string("marv_sil_tick: ") + marv_sil_status_str(source_.last_status()));
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
    const double want_step = static_cast<double>(m_ * tick_ns_) / kNanosecondsPerSecond;
    if (!same_bits(phys->Data().MaxStepSize(), want_step)) {
      refuse("physics max_step_size is not m*tick");
    }
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
};

}  // namespace marv::gz

GZ_ADD_PLUGIN(marv::gz::Lockstep, ::gz::sim::System, ::gz::sim::ISystemConfigure, ::gz::sim::ISystemPreUpdate,
              ::gz::sim::ISystemReset)
GZ_ADD_PLUGIN_ALIAS(marv::gz::Lockstep, "marv::gz::Lockstep")
