/* sim/plant/include/marv_plant.h - marv_plant v0: rotor thrust and torque, motor and ESC first-order lag, gravity.
 * Pure C, <stdint.h> only. Host only. Conventions follow marv_sil.h: struct_size-versioned structs (each must equal
 * the sizeof the header the caller compiled against), status return codes, validate-before-act (a call that returns
 * anything but MARV_PLANT_OK leaves the plant, and every output struct, exactly as it was).
 *
 * Units SI, doubles at the ABI. World NED, body FRD with the origin at the centre of mass, attitude a Hamilton
 * quaternion body -> NED stored [w, x, y, z] (core contracts section 3). Motors are indexed by logical number - 1.
 *
 * The plant owns the motor states (core section 6); the host never reads or writes them. Motor state starts at
 * 0 rad/s. Battery, drag, ground effect, noise: not at v0.
 */
#ifndef MARV_PLANT_H
#define MARV_PLANT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum { MARV_PLANT_N_MOTORS = 4 };

typedef int32_t marv_plant_status;
enum {
  MARV_PLANT_OK = 0,
  MARV_PLANT_E_NULL,    /* a required pointer is NULL */
  MARV_PLANT_E_ABI,     /* struct_size does not match */
  MARV_PLANT_E_CONFIG,  /* a config field is non-finite, non-positive where physical, or out of its set */
  MARV_PLANT_E_BODY,    /* a body field is non-finite, or q_wxyz is not normalized within the tolerance below */
  MARV_PLANT_E_CMD,     /* a DShot value is neither 0 nor in 48..2047 */
  MARV_PLANT_E_DT,      /* dt_s is non-finite or <= 0 */
  MARV_PLANT_E_ALLOC,   /* marv_plant_create could not allocate */
  MARV_PLANT_E_STATE    /* an IMU, vibration or rotor-speed entry was called out of order: attached twice, sampled before its
                           attach, or the vibration attached after the first IMU sample */
};

/* ESC map model. DShot 0 -> commanded rotor speed 0 (stop). DShot 48..2047 -> commanded rotor speed linear in the
 * DShot value, omega_min_rad_s at 48 to omega_max_rad_s at 2047. A labelled scenario value, not a measurement. */
enum { MARV_PLANT_ESC_LINEAR_IN_OMEGA = 1 };

typedef struct marv_plant_config {
  uint32_t struct_size;
  uint32_t esc_map;                                   /* MARV_PLANT_ESC_* */
  uint32_t pole_count;                                /* motor magnet poles, even; 0 = unknown -> erpm_valid 0 */
  int32_t yaw_sign[MARV_PLANT_N_MOTORS];              /* +1 = ccw viewed from above: the reaction torque on the
                                                         body is positive about +z_FRD; -1 = cw */
  double mass_kg;
  double rotor_position_frd_m[MARV_PLANT_N_MOTORS][3];
  double thrust_coeff;                                /* k, N / (rad/s)^2 */
  double torque_ratio_m;                              /* yaw torque / thrust, m */
  double omega_min_rad_s;                             /* > 0, ESC map at DShot 48 */
  double omega_max_rad_s;                             /* > omega_min, ESC map at DShot 2047 */
  double motor_tau_s;                                 /* first-order motor time constant */
  double motor_substep_s;                             /* fixed motor sub-step h */
  double site_lat_rad;                                /* geodetic latitude, |.| <= pi/2 (scenario value) */
  double site_height_m;                               /* geodetic height of the NED origin above the WGS 84
                                                         ellipsoid (scenario value) */
  uint64_t rng_seed;                                  /* the seed of the plant's noise streams (the IMU draws from
                                                         it; marv_plant_step ignores it) */
  double initial_omega_rad_s[MARV_PLANT_N_MOTORS];    /* rotor speed at the first step, index = logical motor - 1;
                                                         each finite and in [0, omega_max_rad_s], else
                                                         MARV_PLANT_E_CONFIG. All 0 = the rotors at rest. Appended
                                                         last: struct_size (== sizeof, checked exactly) grows with it,
                                                         so a caller built against the older header gets
                                                         MARV_PLANT_E_ABI, never a read past its struct. */
} marv_plant_config;

typedef struct marv_plant_body {
  uint32_t struct_size;
  double pos_ned_m[3];
  double vel_ned_m_s[3];                              /* validated, unused at v0 (drag comes later) */
  double q_wxyz[4];                                   /* Hamilton, body FRD -> NED; | |q|^2 - 1 | <= 4 eps_float */
  double omega_frd_rad_s[3];                          /* validated, unused at v0 */
} marv_plant_body;

typedef struct marv_plant_cmd {
  uint32_t struct_size;
  uint16_t dshot[MARV_PLANT_N_MOTORS];                /* index = logical motor - 1; each 0 or 48..2047; held over dt */
} marv_plant_cmd;

/* The wrench is evaluated from the motor state at the END of the step (after the motors have advanced by dt), so a
 * step's output is the load the body feels over the next interval, and the dshot given at step k already acts in the
 * output of step k. Force is the total force on the body in NED including gravity m*g(h)*z_NED with g from WGS 84
 * normal gravity at h = site_height_m - pos_ned_m[2]; torque is about the centre of mass in NED. */
typedef struct marv_plant_out {
  uint32_t struct_size;
  uint32_t erpm_valid;                                /* 0 iff pole_count is 0; erpm[] is then 0 */
  double force_ned_n[3];
  double torque_ned_nm[3];
  double rotor_speed_rad_s[MARV_PLANT_N_MOTORS];
  double erpm[MARV_PLANT_N_MOTORS];                   /* omega * (60 / 2 pi) * (pole_count / 2) */
} marv_plant_out;

typedef struct marv_plant marv_plant;                 /* opaque; owns the motor states and a copy of the config */

marv_plant_status marv_plant_create(const marv_plant_config* cfg, marv_plant** out);  /* *out = NULL on failure */
void              marv_plant_destroy(marv_plant* plant);                             /* NULL is a no-op */
marv_plant_status marv_plant_step(marv_plant* plant, const marv_plant_body* body, const marv_plant_cmd* cmd,
                                  double dt_s, marv_plant_out* out);
const char*       marv_plant_status_str(marv_plant_status s);

/* Generic IMU (L6 stage (a), decision 0012). Opt-in: a plant that never calls marv_plant_imu_attach gives
 * marv_plant_step outputs bit-identical to a plant built without the model, and the IMU never changes them. The model
 * keeps its own state in the plant (bias, delay line, sample counter) and draws from the plant's seeded noise stream 0
 * (the primary IMU) keyed by marv_plant_config.rng_seed. SI units; FRD axes.
 *
 * One sample: the truth is the body's angular rate (marv_plant_body.omega_frd_rad_s) and the specific force of the rotor
 * thrust of the motor state after the last marv_plant_step (before any step, the initial rotor speeds), thrust / mass
 * along -z_FRD, no gravity (thrust is the only non-gravity force at v0). The measurement of sample k is
 * truth(k - latency_samples) (sample 0's truth before the start) + bias(k) + white noise, rounded to the LSB and clamped
 * to the tighter of the 20-bit word and the full scale. Details and the noise convention: sim/plant/src/imu_model.hpp. */
enum { MARV_PLANT_IMU_MAX_LATENCY_SAMPLES = 64 }; /* delay-line capacity (a labelled bound, not a sensor figure) */

/* Flag bit numbers; identical to MARV_IMU_* in marv_sil.h (a test pins it). */
enum { MARV_PLANT_IMU_GYRO_SAT_X = 0, MARV_PLANT_IMU_GYRO_SAT_Y, MARV_PLANT_IMU_GYRO_SAT_Z,
       MARV_PLANT_IMU_ACCEL_SAT_X, MARV_PLANT_IMU_ACCEL_SAT_Y, MARV_PLANT_IMU_ACCEL_SAT_Z,
       MARV_PLANT_IMU_GYRO_VALID, MARV_PLANT_IMU_ACCEL_VALID, MARV_PLANT_IMU_TEMP_VALID };

typedef struct marv_plant_imu_axis_config {
  double noise_density;                               /* N >= 0, one-sided, unit / sqrt(Hz) (rad/s or m/s^2) */
  double bias_instability;                            /* B >= 0, the minimum of the Allan deviation, unit */
  double lsb;                                         /* > 0, unit per count */
  double full_scale;                                  /* > 0, +- unit; at most half the largest float */
  double turn_on_bias[3];                             /* the bias before the first sample, FRD, unit */
} marv_plant_imu_axis_config;

typedef struct marv_plant_imu_config {
  uint32_t struct_size;                               /* sizeof(marv_plant_imu_config), checked exactly */
  uint32_t latency_samples;                           /* <= MARV_PLANT_IMU_MAX_LATENCY_SAMPLES */
  marv_plant_imu_axis_config gyro;
  marv_plant_imu_axis_config accel;
} marv_plant_imu_config;

typedef struct marv_plant_vec3f { float x, y, z; } marv_plant_vec3f;

/* Same layout as marv_imu_meas (marv_sil.h): flags are 1 << MARV_PLANT_IMU_*. TempValid is never set (temp_k 0). */
typedef struct marv_plant_imu_out {
  marv_plant_vec3f gyro_rad_s;
  marv_plant_vec3f accel_m_s2;
  float temp_k;
  uint32_t flags;
} marv_plant_imu_out;

/* Once, before the first sample. Errors: E_NULL, E_ABI, E_CONFIG (a non-finite or out-of-range field, latency above
 * the maximum, or a random-walk gain that overflows), E_STATE (already attached). A refused call changes nothing. */
marv_plant_status marv_plant_imu_attach(marv_plant* plant, const marv_plant_imu_config* cfg);

/* One IMU sample of the body state. Errors: E_NULL, E_ABI (body), E_BODY, E_DT, E_STATE (not attached). A refused call
 * changes no state, draws nothing and leaves *out as it was. */
marv_plant_status marv_plant_imu_sample(marv_plant* plant, const marv_plant_body* body, double dt_s,
                                        marv_plant_imu_out* out);

/* Gyro vibration (L6 stage (b), decision 0013). Opt-in, like the IMU: a plant that never calls
 * marv_plant_vibration_attach, or attaches it with every amplitude exactly 0, gives IMU bytes bit-identical to a plant
 * without the entry, and marv_plant_step is never affected. Motion at the IMU, gyro only: marv_plant_imu_sample adds
 *     v_axis = sum over rotors i, harmonics h = 1..3 of
 *              amplitude_rad_s[h-1] * (omega_i / omega_hover_rad_s)^speed_exponent * sin(h * theta_i + phi_{i,h,axis})
 * to the truth body rate of each gyro axis (FRD, the same amplitude on all three; INFERRED) before the delay line, the
 * noise, the quantiser and the saturation. omega_i and the mechanical angle theta_i (rad, in [0, 2 pi), 0 at the start,
 * the exact integral of the motor model) are those of the rotor state after the last marv_plant_step, so the vibration
 * has the freshness of the specific force. The 36 phases phi are drawn once at attach, uniform on [0, 2 pi), from the
 * seeded noise stream id 1 of rng_seed (order and the draw rule: sim/plant/src/vibration_model.hpp). The accelerometer
 * is not affected. The speed exponent is an integer (a non-integer one is refused at v0). */
enum { MARV_PLANT_VIBRATION_HARMONICS = 3, MARV_PLANT_VIBRATION_MAX_EXPONENT = 8 }; /* the exponent bound is a labelled capacity bound */

typedef struct marv_plant_vibration_config {
  uint32_t struct_size;                               /* sizeof(marv_plant_vibration_config), checked exactly */
  double amplitude_rad_s[MARV_PLANT_VIBRATION_HARMONICS]; /* A_h at omega_hover, each finite and >= 0; index h - 1 */
  double omega_hover_rad_s;                           /* finite, > 0 */
  double speed_exponent;                              /* p: an integer value in 0..MARV_PLANT_VIBRATION_MAX_EXPONENT */
} marv_plant_vibration_config;

/* Once, before the first IMU sample. Errors: E_NULL, E_ABI, E_CONFIG (a non-finite or negative amplitude, omega_hover
 * not finite and positive, an exponent that is not an integer in range), E_STATE (already attached, or the IMU has
 * already taken a sample). A refused call changes nothing. */
marv_plant_status marv_plant_vibration_attach(marv_plant* plant, const marv_plant_vibration_config* cfg);

/* Rotor-speed sensor (L6 stage (b), decision 0013, owner decision 7). Opt-in, like the IMU: a plant that never calls
 * marv_plant_rotor_speed_attach is untouched, and the model never changes marv_plant_step or the IMU bytes (it only reads
 * the rotor speeds). One sample per motor of omega after the last marv_plant_step (before any step, the initial rotor
 * speeds): the electrical period 2 pi / (omega * pole_count / 2) is encoded on the bidirectional-DShot telemetry grid
 * (exponent_bits + mantissa_bits word, period = m << e in units of period_unit_s), decoded again to SI rad/s, and delayed
 * by latency_ticks samples (sample 0's value before the start). A rotor too slow for the grid (including omega = 0) is a
 * VALID sample of speed exactly 0 (the all-ones word, which the protocol reads as zero eRPM); a non-finite or negative
 * omega is an INVALID sample (valid bit clear, value 0). The grid, the rule and the citation: sim/plant/src/
 * rotor_speed_model.hpp. The pole count, the grid parameters and the latency are the caller's (the profile, the
 * scenario value); the plant has no default. */
enum { MARV_PLANT_ROTOR_SPEED_MAX_LATENCY_TICKS = 64 }; /* delay-line capacity (a labelled bound, not a sensor figure) */
enum { MARV_PLANT_ROTOR_SPEED_MAX_EXPONENT_BITS = 4, MARV_PLANT_ROTOR_SPEED_MAX_MANTISSA_BITS = 16 }; /* labelled capacity bounds */

/* Flag bit numbers: bit i is motor i + 1 valid; identical to the firmware's RotorSpeedFlag (a test pins it). */
enum { MARV_PLANT_ROTOR_SPEED_M1_VALID = 0, MARV_PLANT_ROTOR_SPEED_M2_VALID, MARV_PLANT_ROTOR_SPEED_M3_VALID,
       MARV_PLANT_ROTOR_SPEED_M4_VALID };

typedef struct marv_plant_rotor_speed_config {
  uint32_t struct_size;                               /* sizeof(marv_plant_rotor_speed_config), checked exactly */
  uint32_t pole_count;                                /* motor magnet poles: even and > 0 */
  uint32_t latency_ticks;                             /* <= MARV_PLANT_ROTOR_SPEED_MAX_LATENCY_TICKS */
  uint32_t exponent_bits;                             /* telemetry word exponent field, 1..MAX_EXPONENT_BITS (3) */
  uint32_t mantissa_bits;                             /* telemetry word mantissa field, 1..MAX_MANTISSA_BITS (9) */
  double period_unit_s;                               /* the unit of the telemetry period, finite and > 0 (1 us) */
} marv_plant_rotor_speed_config;

/* The layout of the firmware's RotorSpeedSample without t_us (a test pins it): speeds in rad/s, a clear valid bit means
 * a speed of exactly 0, bits above the four valid bits are 0. */
typedef struct marv_plant_rotor_speed_out {
  float omega_rad_s[MARV_PLANT_N_MOTORS];
  uint32_t flags;
} marv_plant_rotor_speed_out;

/* Once. Errors: E_NULL, E_ABI, E_CONFIG (pole count odd or 0, latency or a grid field out of range, a period unit that is
 * not finite and positive), E_STATE (already attached). A refused call changes nothing. */
marv_plant_status marv_plant_rotor_speed_attach(marv_plant* plant, const marv_plant_rotor_speed_config* cfg);

/* One sample of the rotor speeds, once per tick, in the order of the host (the adapter: after the IMU sample of the tick,
 * before the tick's marv_plant_step, so it sees the rotor state after step j - 1). Errors: E_NULL, E_STATE (not attached).
 * A refused call changes no state and leaves *out as it was. */
marv_plant_status marv_plant_rotor_speed_sample(marv_plant* plant, marv_plant_rotor_speed_out* out);

#ifdef __cplusplus
}
#endif

#endif /* MARV_PLANT_H */
