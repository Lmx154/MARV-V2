#include "marv_plant.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <new>

#include "marv/prim/constants.hpp"
#include "marv/prim/quat.hpp"
#include "marv/types/actuator.hpp"
#include "imu_model.hpp"
#include "plant_model.hpp"
#include "rotor_speed_model.hpp"
#include "vibration_model.hpp"

static_assert(MARV_PLANT_N_MOTORS == marv::kQuadXMotors);

struct marv_plant {
  explicit marv_plant(const marv::plant::Params<double>& p, std::uint32_t poles, std::uint64_t seed_in)
      : model(p), pole_count(poles), seed(seed_in) {}
  marv::plant::Model<double> model;
  std::uint32_t pole_count;
  std::uint64_t seed;
  marv::plant::ImuModel imu;
  marv::plant::VibrationModel vibration;
  marv::plant::RotorSpeedModel rotor_speed;
};

namespace marv::plant {
namespace {

// Quaternion normalization tolerance, on | |q|^2 - 1 |. A unit quaternion stored in float has each component off by at
// most eps_f/2 relative, so |q|^2 is off by at most eps_f (sum of 2 q_i^2 (eps_f/2)); a float host that also
// normalizes it in float adds about 3 eps_f more (four products and three sums, sqrt, divide, taken together).
// 4 eps_f = 2 * 2 * eps_f covers both. Anything worse is a host error and is refused; anything accepted is
// renormalized in Model::wrench, so it does not bias the force.
constexpr double kQuatNormTol = 2.0 * 2.0 * static_cast<double>(std::numeric_limits<float>::epsilon());

bool finite(double v) { return std::isfinite(v); }
bool positive(double v) { return std::isfinite(v) && v > 0.0; }

template <std::size_t N>
bool all_finite(const double (&a)[N]) {
  for (double v : a) {
    if (!finite(v)) {
      return false;
    }
  }
  return true;
}

bool config_valid(const marv_plant_config& c) {
  if (c.esc_map != MARV_PLANT_ESC_LINEAR_IN_OMEGA || c.pole_count % 2 != 0) {
    return false;
  }
  for (std::size_t i = 0; i < kMotors; ++i) {
    if ((c.yaw_sign[i] != 1 && c.yaw_sign[i] != -1) || !all_finite(c.rotor_position_frd_m[i])) {
      return false;
    }
  }
  for (std::size_t i = 0; i < kMotors; ++i) {
    if (!finite(c.initial_omega_rad_s[i]) || c.initial_omega_rad_s[i] < 0.0 ||
        c.initial_omega_rad_s[i] > c.omega_max_rad_s) {
      return false;
    }
  }
  return positive(c.mass_kg) && positive(c.thrust_coeff) && positive(c.torque_ratio_m) &&
         positive(c.omega_min_rad_s) && positive(c.omega_max_rad_s) && c.omega_min_rad_s < c.omega_max_rad_s &&
         positive(c.motor_tau_s) && positive(c.motor_substep_s) && finite(c.site_lat_rad) &&
         std::fabs(c.site_lat_rad) <= prim::kPi * 0.5 && finite(c.site_height_m);
}

bool body_valid(const marv_plant_body& b) {
  if (!all_finite(b.pos_ned_m) || !all_finite(b.vel_ned_m_s) || !all_finite(b.q_wxyz) ||
      !all_finite(b.omega_frd_rad_s)) {
    return false;
  }
  const prim::Quat<double> q(b.q_wxyz[0], b.q_wxyz[1], b.q_wxyz[2], b.q_wxyz[3]);
  return std::fabs(q.norm2() - 1.0) <= kQuatNormTol;
}

bool imu_axis_valid(const marv_plant_imu_axis_config& a) {
  return finite(a.noise_density) && a.noise_density >= 0.0 && finite(a.bias_instability) && a.bias_instability >= 0.0 &&
         positive(a.lsb) && positive(a.full_scale) &&
         a.full_scale <= static_cast<double>(std::numeric_limits<float>::max()) * 0.5 && all_finite(a.turn_on_bias) &&
         finite(imu_detail::rw_gain(ImuAxisParams{a.noise_density, a.bias_instability, a.lsb, a.full_scale, {}}));
}

static_assert(kVibrationHarmonics == MARV_PLANT_VIBRATION_HARMONICS);
static_assert(kVibrationMaxExponent == MARV_PLANT_VIBRATION_MAX_EXPONENT);

bool vibration_valid(const marv_plant_vibration_config& c) {
  for (double a : c.amplitude_rad_s) {
    if (!finite(a) || a < 0.0) {
      return false;
    }
  }
  // An integer value in 0..max: the range test first, so the conversion below is defined.
  return positive(c.omega_hover_rad_s) && finite(c.speed_exponent) && c.speed_exponent >= 0.0 &&
         c.speed_exponent <= static_cast<double>(kVibrationMaxExponent) &&
         c.speed_exponent == std::floor(c.speed_exponent);
}

static_assert(kRotorSpeedMotors == MARV_PLANT_N_MOTORS);
static_assert(kRotorSpeedMaxLatency == MARV_PLANT_ROTOR_SPEED_MAX_LATENCY_TICKS);
static_assert(kRotorSpeedMaxExponentBits == MARV_PLANT_ROTOR_SPEED_MAX_EXPONENT_BITS);
static_assert(kRotorSpeedMaxMantissaBits == MARV_PLANT_ROTOR_SPEED_MAX_MANTISSA_BITS);

bool rotor_speed_valid(const marv_plant_rotor_speed_config& c) {
  return c.pole_count != 0 && c.pole_count % 2 == 0 && c.latency_ticks <= kRotorSpeedMaxLatency &&
         c.exponent_bits >= 1 && c.exponent_bits <= kRotorSpeedMaxExponentBits && c.mantissa_bits >= 1 &&
         c.mantissa_bits <= kRotorSpeedMaxMantissaBits && positive(c.period_unit_s);
}

ImuAxisParams to_imu_axis(const marv_plant_imu_axis_config& a) {
  return ImuAxisParams{a.noise_density, a.bias_instability, a.lsb, a.full_scale,
                       {a.turn_on_bias[0], a.turn_on_bias[1], a.turn_on_bias[2]}};
}

Params<double> to_params(const marv_plant_config& c) {
  Params<double> p;
  p.mass = c.mass_kg;
  for (std::size_t i = 0; i < kMotors; ++i) {
    p.rotor_position[i] = prim::Vec3<double>(c.rotor_position_frd_m[i][0], c.rotor_position_frd_m[i][1],
                                             c.rotor_position_frd_m[i][2]);
    p.yaw_sign[i] = static_cast<double>(c.yaw_sign[i]);
  }
  p.thrust_coeff = c.thrust_coeff;
  p.torque_ratio = c.torque_ratio_m;
  p.omega_min = c.omega_min_rad_s;
  p.omega_max = c.omega_max_rad_s;
  p.tau = c.motor_tau_s;
  p.substep = c.motor_substep_s;
  p.site_lat = c.site_lat_rad;
  p.site_height = c.site_height_m;
  for (std::size_t i = 0; i < kMotors; ++i) {
    p.omega0[i] = c.initial_omega_rad_s[i];
  }
  return p;
}

}  // namespace
}  // namespace marv::plant

extern "C" {

marv_plant_status marv_plant_create(const marv_plant_config* cfg, marv_plant** out) {
  if (out == nullptr) {
    return MARV_PLANT_E_NULL;
  }
  *out = nullptr;
  if (cfg == nullptr) {
    return MARV_PLANT_E_NULL;
  }
  if (cfg->struct_size != sizeof(marv_plant_config)) {
    return MARV_PLANT_E_ABI;
  }
  if (!marv::plant::config_valid(*cfg)) {
    return MARV_PLANT_E_CONFIG;
  }
  marv_plant* plant = new (std::nothrow) marv_plant(marv::plant::to_params(*cfg), cfg->pole_count, cfg->rng_seed);
  if (plant == nullptr) {
    return MARV_PLANT_E_ALLOC;
  }
  *out = plant;
  return MARV_PLANT_OK;
}

void marv_plant_destroy(marv_plant* plant) { delete plant; }

marv_plant_status marv_plant_step(marv_plant* plant, const marv_plant_body* body, const marv_plant_cmd* cmd,
                                  double dt_s, marv_plant_out* out) {
  using namespace marv::plant;
  if (plant == nullptr || body == nullptr || cmd == nullptr || out == nullptr) {
    return MARV_PLANT_E_NULL;
  }
  if (body->struct_size != sizeof(marv_plant_body) || cmd->struct_size != sizeof(marv_plant_cmd) ||
      out->struct_size != sizeof(marv_plant_out)) {
    return MARV_PLANT_E_ABI;
  }
  if (!body_valid(*body)) {
    return MARV_PLANT_E_BODY;
  }
  std::array<std::uint16_t, kMotors> dshot{};
  for (std::size_t i = 0; i < kMotors; ++i) {
    if (!marv::DshotValue::is_valid_raw(cmd->dshot[i])) {
      return MARV_PLANT_E_CMD;
    }
    dshot[i] = cmd->dshot[i];
  }
  if (!(std::isfinite(dt_s) && dt_s > 0.0)) {
    return MARV_PLANT_E_DT;
  }

  plant->model.advance(dshot, dt_s);
  const marv::prim::Quat<double> q(body->q_wxyz[0], body->q_wxyz[1], body->q_wxyz[2], body->q_wxyz[3]);
  const Wrench<double> w = plant->model.wrench(q, body->pos_ned_m[2]);

  out->erpm_valid = plant->pole_count != 0 ? 1U : 0U;
  const double pole_pairs = static_cast<double>(plant->pole_count / 2);
  const double rev_per_min_per_rad_s = marv::prim::kSecondsPerMinute / (2.0 * marv::prim::kPi);
  for (std::size_t i = 0; i < kMotors; ++i) {
    const double omega = plant->model.omega()[i];
    out->rotor_speed_rad_s[i] = omega;
    out->erpm[i] = omega * rev_per_min_per_rad_s * pole_pairs;
  }
  for (std::size_t i = 0; i < marv::prim::kSpatialDim; ++i) {
    out->force_ned_n[i] = w.force_ned[i];
    out->torque_ned_nm[i] = w.torque_ned[i];
  }
  return MARV_PLANT_OK;
}

marv_plant_status marv_plant_imu_attach(marv_plant* plant, const marv_plant_imu_config* cfg) {
  using namespace marv::plant;
  if (plant == nullptr || cfg == nullptr) {
    return MARV_PLANT_E_NULL;
  }
  if (cfg->struct_size != sizeof(marv_plant_imu_config)) {
    return MARV_PLANT_E_ABI;
  }
  if (cfg->latency_samples > kImuMaxLatency || !imu_axis_valid(cfg->gyro) || !imu_axis_valid(cfg->accel)) {
    return MARV_PLANT_E_CONFIG;
  }
  if (plant->imu.attached()) {
    return MARV_PLANT_E_STATE;
  }
  ImuParams p;
  p.gyro = to_imu_axis(cfg->gyro);
  p.accel = to_imu_axis(cfg->accel);
  p.latency = cfg->latency_samples;
  plant->imu.attach(p, plant->seed);
  return MARV_PLANT_OK;
}

marv_plant_status marv_plant_imu_sample(marv_plant* plant, const marv_plant_body* body, double dt_s,
                                        marv_plant_imu_out* out) {
  using namespace marv::plant;
  if (plant == nullptr || body == nullptr || out == nullptr) {
    return MARV_PLANT_E_NULL;
  }
  if (body->struct_size != sizeof(marv_plant_body)) {
    return MARV_PLANT_E_ABI;
  }
  if (!body_valid(*body)) {
    return MARV_PLANT_E_BODY;
  }
  if (!(std::isfinite(dt_s) && dt_s > 0.0)) {
    return MARV_PLANT_E_DT;
  }
  if (!plant->imu.attached()) {
    return MARV_PLANT_E_STATE;
  }
  // Specific force: the rotor thrust after the last step over the mass, along -z_FRD (no gravity, no drag at v0).
  const Params<double>& mp = plant->model.params();
  double thrust = 0.0;
  for (std::size_t i = 0; i < kMotors; ++i) {
    const double omega = plant->model.omega()[i];
    thrust += mp.thrust_coeff * omega * omega;
  }
  std::array<double, 6> truth = {body->omega_frd_rad_s[0], body->omega_frd_rad_s[1], body->omega_frd_rad_s[2],
                                 0.0, 0.0, -(thrust / mp.mass)};
  // Vibration, when attached with some nonzero amplitude: added to the gyro truth, before the delay line. Otherwise
  // the truth is not touched (not even by + 0), so the stage (a) bytes are unchanged by construction.
  if (plant->vibration.active()) {
    const std::array<double, 3> vib = plant->vibration.value(plant->model.omega(), plant->model.theta());
    for (std::size_t a = 0; a < 3; ++a) {
      truth[a] += vib[a];
    }
  }
  const ImuOut m = plant->imu.sample(truth, dt_s);
  out->gyro_rad_s = {m.gyro[0], m.gyro[1], m.gyro[2]};
  out->accel_m_s2 = {m.accel[0], m.accel[1], m.accel[2]};
  out->temp_k = 0.0F;
  out->flags = m.flags;
  return MARV_PLANT_OK;
}

marv_plant_status marv_plant_vibration_attach(marv_plant* plant, const marv_plant_vibration_config* cfg) {
  using namespace marv::plant;
  if (plant == nullptr || cfg == nullptr) {
    return MARV_PLANT_E_NULL;
  }
  if (cfg->struct_size != sizeof(marv_plant_vibration_config)) {
    return MARV_PLANT_E_ABI;
  }
  if (!vibration_valid(*cfg)) {
    return MARV_PLANT_E_CONFIG;
  }
  if (plant->vibration.attached() || plant->imu.samples_taken() != 0) {
    return MARV_PLANT_E_STATE;
  }
  VibrationParams p;
  for (std::size_t h = 0; h < kVibrationHarmonics; ++h) {
    p.amplitude[h] = cfg->amplitude_rad_s[h];
  }
  p.omega_hover = cfg->omega_hover_rad_s;
  p.exponent = static_cast<std::uint32_t>(cfg->speed_exponent);
  plant->vibration.attach(p, plant->seed);
  return MARV_PLANT_OK;
}

marv_plant_status marv_plant_rotor_speed_attach(marv_plant* plant, const marv_plant_rotor_speed_config* cfg) {
  using namespace marv::plant;
  if (plant == nullptr || cfg == nullptr) {
    return MARV_PLANT_E_NULL;
  }
  if (cfg->struct_size != sizeof(marv_plant_rotor_speed_config)) {
    return MARV_PLANT_E_ABI;
  }
  if (!rotor_speed_valid(*cfg)) {
    return MARV_PLANT_E_CONFIG;
  }
  if (plant->rotor_speed.attached()) {
    return MARV_PLANT_E_STATE;
  }
  RotorSpeedParams p;
  p.pole_count = cfg->pole_count;
  p.latency = cfg->latency_ticks;
  p.exponent_bits = cfg->exponent_bits;
  p.mantissa_bits = cfg->mantissa_bits;
  p.period_unit_s = cfg->period_unit_s;
  plant->rotor_speed.attach(p);
  return MARV_PLANT_OK;
}

marv_plant_status marv_plant_rotor_speed_sample(marv_plant* plant, marv_plant_rotor_speed_out* out) {
  using namespace marv::plant;
  if (plant == nullptr || out == nullptr) {
    return MARV_PLANT_E_NULL;
  }
  if (!plant->rotor_speed.attached()) {
    return MARV_PLANT_E_STATE;
  }
  const RotorSpeedOut m = plant->rotor_speed.sample(plant->model.omega());
  for (std::size_t i = 0; i < kMotors; ++i) {
    out->omega_rad_s[i] = m.omega[i];
  }
  out->flags = m.flags;
  return MARV_PLANT_OK;
}

const char* marv_plant_status_str(marv_plant_status s) {
  switch (s) {
    case MARV_PLANT_OK: return "ok";
    case MARV_PLANT_E_NULL: return "null pointer";
    case MARV_PLANT_E_ABI: return "struct_size mismatch";
    case MARV_PLANT_E_CONFIG: return "invalid config";
    case MARV_PLANT_E_BODY: return "invalid body state";
    case MARV_PLANT_E_CMD: return "invalid DShot command";
    case MARV_PLANT_E_DT: return "invalid dt";
    case MARV_PLANT_E_ALLOC: return "allocation failed";
    case MARV_PLANT_E_STATE: return "IMU entry out of order";
    default: return "unknown status";
  }
}

}  // extern "C"
