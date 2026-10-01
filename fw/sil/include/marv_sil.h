/* fw/sil/include/marv_sil.h - the software-in-the-loop entry. Pure C, <stdint.h> only. */
#ifndef MARV_SIL_H
#define MARV_SIL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int32_t marv_sil_status;
enum { MARV_SIL_OK = 0, MARV_SIL_E_NULL, MARV_SIL_E_ABI, MARV_SIL_E_STATE, MARV_SIL_E_PERIOD,
       MARV_SIL_E_SCHEMA, MARV_SIL_E_PARAM, MARV_SIL_E_TICK, MARV_SIL_E_COUNT, MARV_SIL_E_INPUT };
typedef struct marv_vec3f { float x, y, z; } marv_vec3f;
enum { MARV_IMU_GYRO_SAT_X = 0, MARV_IMU_GYRO_SAT_Y, MARV_IMU_GYRO_SAT_Z,
       MARV_IMU_ACCEL_SAT_X, MARV_IMU_ACCEL_SAT_Y, MARV_IMU_ACCEL_SAT_Z,
       MARV_IMU_GYRO_VALID, MARV_IMU_ACCEL_VALID, MARV_IMU_TEMP_VALID, MARV_IMU_FLAG_COUNT };
typedef struct marv_imu_meas { marv_vec3f gyro_rad_s; marv_vec3f accel_m_s2; float temp_k; uint32_t flags; } marv_imu_meas;
/* Rotor speed (L6 stage (b), decision 0013): SI rad/s per motor, index = logical motor - 1; bit i of flags is motor i + 1
 * valid. Same layout as RotorSpeedSample (fw/types) without t_us. A clear valid bit means a speed of exactly 0. */
enum { MARV_ROTOR_SPEED_M1_VALID = 0, MARV_ROTOR_SPEED_M2_VALID, MARV_ROTOR_SPEED_M3_VALID, MARV_ROTOR_SPEED_M4_VALID,
       MARV_ROTOR_SPEED_FLAG_COUNT };
enum { MARV_ROTOR_SPEED_MOTORS = MARV_ROTOR_SPEED_FLAG_COUNT };
typedef struct marv_rotor_speed_meas { float omega_rad_s[MARV_ROTOR_SPEED_MOTORS]; uint32_t flags; } marv_rotor_speed_meas;
enum { MARV_PARAM_F32 = 0, MARV_PARAM_I32 };
typedef struct marv_sil_param_override { uint32_t id; uint32_t type; float f32; int32_t i32; float sigma; } marv_sil_param_override;
typedef struct marv_sil_config {
  uint32_t struct_size, imu_meas_size, override_size;
  uint32_t tick_period_num_us, tick_period_den;
  uint32_t n_overrides; const marv_sil_param_override* overrides;   /* NULL iff 0; read during init only */
  uint64_t param_schema_hash;
} marv_sil_config;
typedef struct marv_sil_info { uint32_t struct_size, n_motors, n_servos, n_params; uint64_t param_schema_hash; const char* composition; } marv_sil_info;
typedef struct marv_sil_out { uint32_t struct_size; uint32_t capacity_ticks; uint64_t* t_us; uint16_t* dshot; uint16_t* servo_us; } marv_sil_out;
marv_sil_status marv_sil_info_get(marv_sil_info* info);           /* any state */
marv_sil_status marv_sil_init(const marv_sil_config* cfg);        /* UNINIT -> READY, once per process */
marv_sil_status marv_sil_tick(uint64_t first_tick, uint32_t k, const marv_imu_meas* imu, marv_sil_out* out); /* READY */
/* As marv_sil_tick, with one rotor-speed sample per tick: rotor[0..k-1], read through hal_rotor_speed() in the tick. The
 * sample is validated like the IMU's (finite, no reserved bit, a clear valid bit means a speed of 0, a valid speed is not
 * negative); a bad one is E_INPUT with marv_sil_error_index() = its tick index. marv_sil_tick supplies none: the HAL read
 * then reports every motor invalid. READY. */
marv_sil_status marv_sil_tick_with_rotor_speed(uint64_t first_tick, uint32_t k, const marv_imu_meas* imu,
                                               const marv_rotor_speed_meas* rotor, marv_sil_out* out);
marv_sil_status marv_sil_shutdown(void);                          /* READY -> DONE (terminal) */
const char*     marv_sil_status_str(marv_sil_status s);
uint32_t        marv_sil_error_index(void);

#ifdef __cplusplus
}
#endif

#endif /* MARV_SIL_H */
