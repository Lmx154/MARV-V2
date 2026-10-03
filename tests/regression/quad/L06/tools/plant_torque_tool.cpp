// One marv_plant_step through the C ABI (sim/plant/include/marv_plant.h) for test_steady_tumble.py (decision 0015, the
// T1 check of R2's steady-tumble rotor speeds on the plant itself). Input on stdin, whitespace-separated: the
// marv_plant_config fields esc_map pole_count yaw_sign[4] (integers), mass_kg rotor_position_frd_m[4][3] thrust_coeff
// torque_ratio_m omega_min_rad_s omega_max_rad_s motor_tau_s motor_substep_s site_lat_rad site_height_m (C hexadecimal
// doubles), rng_seed (integer), initial_omega_rad_s[4]; the body pos_ned_m[3] vel_ned_m_s[3] q_wxyz[4]
// omega_frd_rad_s[3]; dshot[4] (integers) and dt_s. Output, one line: torque_ned_nm[3] and rotor_speed_rad_s[4] as
// hexadecimal doubles. A status other than MARV_PLANT_OK is printed on stderr with exit status 1.
//
// The plant's IMU and vibration models are not attached, so the step draws no noise. The three vendored musl entries the
// noise code references are defined here as aborts instead of linking the vendored C: a call would end the tool with a
// signal and fail the test.
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include "marv_plant.h"
#include "noise/musl_math.h"

extern "C" double marv_musl_log(double /*x*/) { std::abort(); }
extern "C" double marv_musl_sin(double /*x*/) { std::abort(); }
extern "C" double marv_musl_cos(double /*x*/) { std::abort(); }

namespace {

bool read_doubles(double* out, int n) {
  for (int i = 0; i < n; ++i) {
    if (std::scanf("%la", &out[i]) != 1) {
      return false;
    }
  }
  return true;
}

}  // namespace

int main() {
  marv_plant_config cfg{};
  cfg.struct_size = sizeof cfg;
  unsigned long long seed = 0;
  if (std::scanf("%u %u", &cfg.esc_map, &cfg.pole_count) != 2) {
    return 1;
  }
  for (std::int32_t& s : cfg.yaw_sign) {
    if (std::scanf("%d", &s) != 1) {
      return 1;
    }
  }
  double scalars[9] = {};
  if (!read_doubles(&cfg.mass_kg, 1)) {
    return 1;
  }
  for (double(&r)[3] : cfg.rotor_position_frd_m) {
    if (!read_doubles(r, 3)) {
      return 1;
    }
  }
  if (!read_doubles(scalars, 9) || std::scanf("%llu", &seed) != 1 ||
      !read_doubles(cfg.initial_omega_rad_s, MARV_PLANT_N_MOTORS)) {
    return 1;
  }
  cfg.thrust_coeff = scalars[0];
  cfg.torque_ratio_m = scalars[1];
  cfg.omega_min_rad_s = scalars[2];
  cfg.omega_max_rad_s = scalars[3];
  cfg.motor_tau_s = scalars[4];
  cfg.motor_substep_s = scalars[5];
  cfg.site_lat_rad = scalars[6];
  cfg.site_height_m = scalars[7];
  cfg.rng_seed = seed;
  const double dt = scalars[8];

  marv_plant_body body{};
  body.struct_size = sizeof body;
  marv_plant_cmd cmd{};
  cmd.struct_size = sizeof cmd;
  if (!read_doubles(body.pos_ned_m, 3) || !read_doubles(body.vel_ned_m_s, 3) || !read_doubles(body.q_wxyz, 4) ||
      !read_doubles(body.omega_frd_rad_s, 3)) {
    return 1;
  }
  for (std::uint16_t& d : cmd.dshot) {
    unsigned v = 0;
    if (std::scanf("%u", &v) != 1) {
      return 1;
    }
    d = static_cast<std::uint16_t>(v);
  }

  marv_plant* plant = nullptr;
  marv_plant_status st = marv_plant_create(&cfg, &plant);
  if (st != MARV_PLANT_OK) {
    std::fprintf(stderr, "marv_plant_create: %s\n", marv_plant_status_str(st));
    return 1;
  }
  marv_plant_out out{};
  out.struct_size = sizeof out;
  st = marv_plant_step(plant, &body, &cmd, dt, &out);
  marv_plant_destroy(plant);
  if (st != MARV_PLANT_OK) {
    std::fprintf(stderr, "marv_plant_step: %s\n", marv_plant_status_str(st));
    return 1;
  }
  std::printf("%a %a %a %a %a %a %a\n", out.torque_ned_nm[0], out.torque_ned_nm[1], out.torque_ned_nm[2],
              out.rotor_speed_rad_s[0], out.rotor_speed_rad_s[1], out.rotor_speed_rad_s[2], out.rotor_speed_rad_s[3]);
  return 0;
}
