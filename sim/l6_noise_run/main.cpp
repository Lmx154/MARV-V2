// l6_noise_run: the host-free run driver of the L6 stage (a) T1 SIM-2-with-noise check. It closes the Gazebo adapter,
// marv_plant with its generic IMU, and the l4_rate_scripted SIL (a flight composition built without a truth-state entry:
// the firmware sees only the plant's noisy IMU bytes, ImuSilCommandSource), and a minimal rigid body that stands in for
// Gazebo: the body state is held over each tick, the plant's wrench torque turns it, nothing else moves it.
//
// Usage: l6_noise_run <seed> <corner> <ticks> <out>
//   seed    u64, decimal or 0x hex: marv_plant_config::rng_seed
//   corner  -1, 0 or +1: the ODR clock corner of the adapter (decision 0012)
//   ticks   number of SIL ticks, one per host step
//   out     path of the binary log (log_format.hpp)
// Prints "sha256 <hex>" of the log file's bytes. Exit 0 on success, 1 on a usage or run failure (message on stderr).
#include <marv_sil.h>

#include <array>
#include <bit>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include MARV_PLANT_CARD_HEADER
#include "marv/gz/adapter.hpp"
#include "marv/params/param_ids.hpp"
#include "marv/prim/constants.hpp"
#include "marv/sim/imu_profile_config.hpp"
#include "log_format.hpp"
#include "marv_plant.h"
#include "marv/sim/sha256.hpp"

namespace {

using namespace marv::gz;
namespace run = marv::l6run;

// ---- labelled scenario values ------------------------------------------------------------------------------------

// The SIL period 625 / 4 us = 156.25 us (design/scenario_values.yaml tick_period_num_us and tick_period_den; the SIL
// refuses a period that differs from its parameter set).
constexpr std::uint32_t kPeriodNumUs = 625;
constexpr std::uint32_t kPeriodDen = 4;
// Site of the plant, as the L2 adapter test: scenario values, not vehicle data.
constexpr double kSiteLatRad = 0.82;
constexpr double kSiteHeightM = 500.0;
// Roll-rate setpoint step at stamp 0, rad/s: a labelled scenario value that makes the rate loop act on the noisy gyro.
constexpr float kRollStepRad = 0.5F;
// Inertia diagonal, kg m^2: restated from vehicles/uzh_neurobem_5in.yaml inertia_diag (sigma UNKNOWN there); a scenario
// value of this driver, not generated from the card.
constexpr std::array<double, 3> kInertiaDiag = {0.0025, 0.0021, 0.0043};

// ---- body ---------------------------------------------------------------------------------------------------------

struct BodyState {
  std::array<double, 4> q{1.0, 0.0, 0.0, 0.0};  // Hamilton, body -> NED
  std::array<double, 3> w{0.0, 0.0, 0.0};       // FRD, rad/s
};

using V3 = std::array<double, 3>;

V3 cross(const V3& a, const V3& b) {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
double dot(const V3& a, const V3& b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

// v rotated by the unit quaternion q (w, u): (w^2 - u.u) v + 2 u (u.v) + 2 w (u x v).
V3 rotate(const std::array<double, 4>& q, const V3& v) {
  const double w = q[0];
  const V3 u = {q[1], q[2], q[3]};
  const double d = w * w - dot(u, u);
  const double uv = dot(u, v);
  const V3 c = cross(u, v);
  return {d * v[0] + 2.0 * u[0] * uv + 2.0 * w * c[0], d * v[1] + 2.0 * u[1] * uv + 2.0 * w * c[1],
          d * v[2] + 2.0 * u[2] * uv + 2.0 * w * c[2]};
}

// Semi-implicit Euler step of dt seconds under the body torque of the plant (NED -> FRD by the inverse rotation):
// Euler's equation with a diagonal inertia, then q <- q (x) [1, w dt / 2] normalized, canonical sign w >= 0. A scenario
// stand-in for Gazebo's integrator; it only needs to be deterministic and bounded.
void advance(BodyState& s, const double torque_ned[3], double dt) {
  const std::array<double, 4> qinv = {s.q[0], -s.q[1], -s.q[2], -s.q[3]};
  const V3 tb = rotate(qinv, {torque_ned[0], torque_ned[1], torque_ned[2]});
  const V3 iw = {kInertiaDiag[0] * s.w[0], kInertiaDiag[1] * s.w[1], kInertiaDiag[2] * s.w[2]};
  const V3 gyro = cross(s.w, iw);
  for (std::size_t i = 0; i < 3; ++i) {
    s.w[i] += dt * (tb[i] - gyro[i]) / kInertiaDiag[i];
  }
  const double h = 0.5 * dt;
  const V3 dv = {s.w[0] * h, s.w[1] * h, s.w[2] * h};
  const V3 qv = {s.q[1], s.q[2], s.q[3]};
  const V3 cr = cross(qv, dv);
  std::array<double, 4> n = {s.q[0] - dot(qv, dv), s.q[1] + s.q[0] * dv[0] + cr[0], s.q[2] + s.q[0] * dv[1] + cr[1],
                             s.q[3] + s.q[0] * dv[2] + cr[2]};
  const double norm = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2] + n[3] * n[3]);
  const double sign = n[0] < 0.0 ? -1.0 : 1.0;
  for (std::size_t i = 0; i < 4; ++i) {
    s.q[i] = sign * n[i] / norm;
  }
}

marv_plant_body to_plant_body(const BodyState& s) {
  marv_plant_body b{};
  b.struct_size = sizeof(b);
  for (std::size_t i = 0; i < 4; ++i) {
    b.q_wxyz[i] = s.q[i];
  }
  for (std::size_t i = 0; i < 3; ++i) {
    b.omega_frd_rad_s[i] = s.w[i];
  }
  return b;
}

// ---- log ----------------------------------------------------------------------------------------------------------

class Log {
 public:
  void u16(std::uint16_t v) { le(v, 2); }
  void u32(std::uint32_t v) { le(v, 4); }
  void u64(std::uint64_t v) { le(v, 8); }
  void f32(float v) { u32(std::bit_cast<std::uint32_t>(v)); }
  void f64(double v) { u64(std::bit_cast<std::uint64_t>(v)); }
  void raw(const char* p, std::size_t n) { bytes_.insert(bytes_.end(), p, p + n); }
  const std::vector<std::uint8_t>& bytes() const { return bytes_; }

 private:
  void le(std::uint64_t v, unsigned n) {
    for (unsigned i = 0; i < n; ++i) {
      bytes_.push_back(static_cast<std::uint8_t>(v >> (8U * i)));
    }
  }
  std::vector<std::uint8_t> bytes_;
};

int fail(const char* what) {
  std::fprintf(stderr, "l6_noise_run: %s\n", what);
  return 1;
}

bool parse_u64(const char* s, std::uint64_t& out) {
  char* end = nullptr;
  errno = 0;
  out = std::strtoull(s, &end, 0);
  return errno == 0 && end != s && *end == '\0' && s[0] != '-';
}

marv_sil_status init_sil(float hover_thrust_n) {
  std::vector<marv_sil_param_override> ov;
  const auto add_f32 = [&ov](marv::ParamId id, float x) {
    marv_sil_param_override o{};
    o.id = static_cast<std::uint32_t>(id);
    o.type = MARV_PARAM_F32;
    o.f32 = x;
    ov.push_back(o);
  };
  const auto add_i32 = [&ov](marv::ParamId id, std::int32_t x) {
    marv_sil_param_override o{};
    o.id = static_cast<std::uint32_t>(id);
    o.type = MARV_PARAM_I32;
    o.i32 = x;
    ov.push_back(o);
  };
  add_f32(marv::ParamId::l4_thrust_n, hover_thrust_n);
  add_i32(marv::ParamId::l4_seg_count, 1);
  add_i32(marv::ParamId::l4_seg1_t_us, 0);
  add_f32(marv::ParamId::l4_seg1_roll, kRollStepRad);
  marv_sil_config c{};
  c.struct_size = sizeof(marv_sil_config);
  c.imu_meas_size = sizeof(marv_imu_meas);
  c.override_size = sizeof(marv_sil_param_override);
  c.tick_period_num_us = kPeriodNumUs;
  c.tick_period_den = kPeriodDen;
  c.n_overrides = static_cast<std::uint32_t>(ov.size());
  c.overrides = ov.data();
  c.param_schema_hash = marv::kParamSchemaHash;
  return marv_sil_init(&c);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 5) {
    return fail("usage: l6_noise_run <seed> <corner -1|0|1> <ticks> <out>");
  }
  std::uint64_t seed = 0;
  std::uint64_t ticks = 0;
  if (!parse_u64(argv[1], seed) || !parse_u64(argv[3], ticks) || ticks == 0) {
    return fail("seed and ticks must be unsigned integers, ticks >= 1");
  }
  const std::string corner_text = argv[2];
  int corner = 0;
  if (corner_text == "-1") {
    corner = -1;
  } else if (corner_text == "0" || corner_text == "+0") {
    corner = 0;
  } else if (corner_text == "1" || corner_text == "+1") {
    corner = 1;
  } else {
    return fail("corner must be -1, 0 or 1");
  }

  const double t_nom = tick_period_s(kPeriodNumUs, kPeriodDen);
  // The IMU is the sensor profile's (sensors/profiles/marv_v2_board_default.yaml, generated at build time): turn-on bias
  // nominal (0); the ODR error is the adapter's clock map's, drawn once per run at the corner.
  const marv_plant_imu_config imu_cfg = marv::sim::imu_profile_config();

  marv_plant_config pc{};
  MARV_PLANT_CARD_FILL(&pc);
  pc.site_lat_rad = kSiteLatRad;
  pc.site_height_m = kSiteHeightM;
  pc.motor_substep_s = t_nom;  // exact zero-order hold over the tick (adapter.hpp)
  pc.rng_seed = seed;
  // The rotors start at the hover speed: omega = sqrt(m g / (4 k)), g = WGS 84 equatorial normal gravity (the hover
  // thrust the composition is given, below).
  const double hover_n = pc.mass_kg * marv::prim::kWgs84GammaE;
  const double omega_hover = std::sqrt(hover_n / (static_cast<double>(MARV_PLANT_N_MOTORS) * pc.thrust_coeff));
  for (double& w : pc.initial_omega_rad_s) {
    w = omega_hover;
  }
  marv_plant* plant = nullptr;
  if (marv_plant_create(&pc, &plant) != MARV_PLANT_OK) {
    return fail("marv_plant_create failed");
  }

  const marv_sil_status st = init_sil(static_cast<float>(hover_n));
  if (st != MARV_SIL_OK) {
    std::fprintf(stderr, "l6_noise_run: marv_sil_init: %s\n", marv_sil_status_str(st));
    marv_plant_destroy(plant);
    return 1;
  }

  ImuSilCommandSource source;
  AdapterConfig ac;
  ac.t_tick_nominal_s = t_nom;
  ac.clock_corner = corner;
  ac.odr_error = marv::sim::kImuOdrError;
  ac.imu = &imu_cfg;
  Adapter adapter(plant, source, ac);

  Log log;
  log.raw(run::kMagic, sizeof(run::kMagic));
  log.u64(seed);
  log.u64(static_cast<std::uint64_t>(static_cast<std::int64_t>(corner)));
  log.u64(ticks);

  BodyState body;
  for (std::uint64_t n = 0; n < ticks; ++n) {
    const StepResult r = adapter.step(to_plant_body(body), n, 1);
    if (r.status != Status::kOk || r.ticks.size() != 1) {
      std::fprintf(stderr, "l6_noise_run: tick %llu failed: adapter status %d, plant status %d, sil status %d\n",
                   static_cast<unsigned long long>(n), static_cast<int>(r.status), r.plant_status,
                   source.last_status());
      return 1;
    }
    const TickOutput& t = r.ticks[0];
    log.u64(n);
    log.u64(source.last_stamp_us());
    log.f32(t.imu.gyro_rad_s.x);
    log.f32(t.imu.gyro_rad_s.y);
    log.f32(t.imu.gyro_rad_s.z);
    log.f32(t.imu.accel_m_s2.x);
    log.f32(t.imu.accel_m_s2.y);
    log.f32(t.imu.accel_m_s2.z);
    log.f32(t.imu.temp_k);
    log.u32(t.imu.flags);
    for (const std::uint16_t d : t.dshot) {
      log.u16(d);
    }
    for (const double x : body.q) {
      log.f64(x);
    }
    for (const double x : body.w) {
      log.f64(x);
    }
    advance(body, t.out.torque_ned_nm, adapter.t_tick_s());
  }

  std::FILE* f = std::fopen(argv[4], "wb");
  if (f == nullptr) {
    return fail("cannot open the output path");
  }
  const std::vector<std::uint8_t>& bytes = log.bytes();
  const bool ok = std::fwrite(bytes.data(), 1, bytes.size(), f) == bytes.size();
  if (std::fclose(f) != 0 || !ok) {
    return fail("cannot write the output file");
  }
  marv::sim::Sha256 sha;
  sha.update(bytes.data(), bytes.size());
  std::printf("sha256 %s\n", sha.hex().c_str());
  return 0;
}
