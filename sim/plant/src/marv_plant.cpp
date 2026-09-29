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
#include "plant_model.hpp"

static_assert(MARV_PLANT_N_MOTORS == marv::kQuadXMotors);

struct marv_plant {
  explicit marv_plant(const marv::plant::Params<double>& p, std::uint32_t poles) : model(p), pole_count(poles) {}
  marv::plant::Model<double> model;
  std::uint32_t pole_count;
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
  marv_plant* plant = new (std::nothrow) marv_plant(marv::plant::to_params(*cfg), cfg->pole_count);
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
    default: return "unknown status";
  }
}

}  // extern "C"
