/* fw/sil/include/marv_sil.h - the software-in-the-loop entry. Pure C, <stdint.h> only. */
#ifndef MARV_SIL_H
#define MARV_SIL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int32_t marv_sil_status;
enum { MARV_SIL_OK = 0, MARV_SIL_E_NULL, MARV_SIL_E_ABI, MARV_SIL_E_STATE, MARV_SIL_E_PERIOD,
       MARV_SIL_E_SCHEMA, MARV_SIL_E_PARAM, MARV_SIL_E_TICK, MARV_SIL_E_COUNT, MARV_SIL_E_INPUT, MARV_SIL_E_STORAGE,
       MARV_SIL_E_LINK };
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
/* Manual control (HIL S2c, decision 0046 item 4): the pilot's input as hal_manual_control() returns it, same fields as
 * ManualControl (fw/types). struct_size is sizeof(marv_manual_control), checked (E_ABI). t_us is the frame stamp on the
 * board clock (hal_time_us() at the frame's completion; in SIL hal_time_us() is the tick stamp, so a replay keeps each tick's
 * frame age = tick stamp - t_us). frames counts valid RC frames (0 = none yet; then t_us, the sticks, throttle and channels
 * are 0). channels are [-1, 1], channel_01 at index 0 (the arm switch is channel 5, index 4). */
enum { MARV_RC_CH_01 = 0, MARV_RC_CH_02, MARV_RC_CH_03, MARV_RC_CH_04, MARV_RC_CH_05, MARV_RC_CH_06, MARV_RC_CH_07,
       MARV_RC_CH_08, MARV_RC_CH_09, MARV_RC_CH_10, MARV_RC_CH_11, MARV_RC_CH_12, MARV_RC_CH_13, MARV_RC_CH_14,
       MARV_RC_CH_15, MARV_RC_CH_16, MARV_RC_CHANNELS };   /* index = channel - 1; the count is checked against the CRSF 16 */
typedef struct marv_manual_control {
  uint32_t struct_size;
  uint32_t frames;
  uint64_t t_us;
  float roll, pitch, yaw, throttle;
  float channels[MARV_RC_CHANNELS];
  float link_quality;
  uint64_t link_t_us;
} marv_manual_control;
enum { MARV_PARAM_F32 = 0, MARV_PARAM_I32 };
typedef struct marv_sil_param_override { uint32_t id; uint32_t type; float f32; int32_t i32; float sigma; } marv_sil_param_override;
/* image_sha256 (decision 0053; ground-segment unit 6): the 32-byte SHA-256 of the loaded SIL library, the image identity the
 * config store binds stored configs to and the link reports; NULL = unknown (no stored config is bound to it). Read during
 * init only; only a library built with STORAGE uses it. */
typedef struct marv_sil_config {
  uint32_t struct_size, imu_meas_size, override_size;
  uint32_t tick_period_num_us, tick_period_den;
  uint32_t n_overrides; const marv_sil_param_override* overrides;   /* NULL iff 0; read during init only */
  uint64_t param_schema_hash;
  const uint8_t* image_sha256;
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
/* Stages the manual control that hal_manual_control() returns from the next tick on, verbatim; it holds for every following
 * tick until the next call replaces it (the HAL's sticky stage). Never staged: no frame (frames 0, every field 0), so the
 * link is down (arming state NoLink). READY, between tick calls. Validated: struct_size (E_ABI); every float finite, sticks
 * and channels in [-1, 1], throttle and link_quality in [0, 1], and frames == 0 with t_us, sticks, throttle and channels
 * all 0 (E_INPUT). The ticks' IMU deadline-miss count is always 0 and the rotor-speed read reports no sample
 * (marv_sil_tick) or the tick's (marv_sil_tick_with_rotor_speed). */
marv_sil_status marv_sil_stage_manual_control(const marv_manual_control* mc);
marv_sil_status marv_sil_shutdown(void);                          /* READY -> DONE (terminal) */
/* Config storage (decision 0053, ground-segment unit 4b; only in a library built with STORAGE, the flight composition's):
 * opens the config storage in the existing directory run_dir (fw/hal/sim/include/marv/hal_sim/storage.hpp: the two slot
 * files, the pending file, the boot request), which marv_sil_init's boot step then reads before params_init: a requested
 * activation is applied and the table is chosen by the boot policy (fw/store/include/marv/store/store.hpp); a stored
 * config that cannot be used refuses arming (ConfigRefused). Without this call there is no storage and the run is the run
 * of the compiled table. UNINIT only (E_STATE after marv_sil_init); E_NULL for NULL; E_STORAGE if run_dir is not a usable
 * directory or holds a slot file larger than a slot. */
marv_sil_status marv_sil_storage_open(const char* run_dir);
/* The ground link (decision 0053 Q4; ground-segment unit 6; only in a library built with STORAGE). marv_sil_link_open binds
 * the link's non-blocking UDP socket to 127.0.0.1:udp_port (fw/hal/sim/include/marv/hal_sim/link_udp.hpp); the sender of the
 * latest datagram is the peer the link answers. UNINIT only (E_STATE after marv_sil_init or a second call); E_LINK if the
 * port is 0 or above 65535 or the socket cannot be bound. Without it marv_sil_link_service does nothing.
 * marv_sil_link_service serves the link once (fw/link/include/marv/link/service.hpp: the waiting frames, the heartbeat when
 * due, the next parameter of a listing); call it between tick calls, once per host step, never from inside a tick. READY;
 * E_NULL for NULL. *reboot_requested is 1 once a config command reboots the vehicle (the store's maintenance grant was given
 * and the reboot requested): the host stops the run and relaunches it on the same run directory, whose storage the next
 * boot step reads; else 0. */
marv_sil_status marv_sil_link_open(uint32_t udp_port);
marv_sil_status marv_sil_link_service(uint32_t* reboot_requested);
const char*     marv_sil_status_str(marv_sil_status s);
uint32_t        marv_sil_error_index(void);

#ifdef __cplusplus
}
#endif

#endif /* MARV_SIL_H */
