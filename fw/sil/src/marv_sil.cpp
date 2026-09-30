#include "marv_sil.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>

#include <marv/composition.hpp>
#include <marv/hal/hal.hpp>
#include <marv/hal/tick.hpp>
#include <marv/hal_sim/hal_sim.hpp>
#include <marv/params/param.hpp>
#include <marv/types/actuator.hpp>
#include <marv/types/imu_sample.hpp>

#include "param_override.hpp"
#include "sil_session.hpp"

namespace marv::sil {
namespace {

// The C structs and enums mirror the C++ boundary types; a mismatch is a compile error.
static_assert(sizeof(marv_vec3f) == sizeof(prim::Vec3<float>) && alignof(marv_vec3f) == alignof(prim::Vec3<float>));
static_assert(offsetof(marv_vec3f, x) == 0 && offsetof(marv_vec3f, y) == sizeof(float) &&
              offsetof(marv_vec3f, z) == sizeof(float) + sizeof(float));
static_assert(sizeof(marv_imu_meas) == sizeof(ImuSample) - sizeof(TimeUs));
static_assert(offsetof(marv_imu_meas, gyro_rad_s) + sizeof(TimeUs) == offsetof(ImuSample, gyro_rad_s));
static_assert(offsetof(marv_imu_meas, accel_m_s2) + sizeof(TimeUs) == offsetof(ImuSample, accel_m_s2));
static_assert(offsetof(marv_imu_meas, temp_k) + sizeof(TimeUs) == offsetof(ImuSample, temp_k));
static_assert(offsetof(marv_imu_meas, flags) + sizeof(TimeUs) == offsetof(ImuSample, flags));
static_assert(sizeof(marv_imu_meas::flags) == sizeof(ImuSample::flags));
static_assert(static_cast<std::uint32_t>(MARV_IMU_GYRO_SAT_X) == static_cast<std::uint32_t>(ImuFlag::GyroSatX));
static_assert(static_cast<std::uint32_t>(MARV_IMU_GYRO_SAT_Y) == static_cast<std::uint32_t>(ImuFlag::GyroSatY));
static_assert(static_cast<std::uint32_t>(MARV_IMU_GYRO_SAT_Z) == static_cast<std::uint32_t>(ImuFlag::GyroSatZ));
static_assert(static_cast<std::uint32_t>(MARV_IMU_ACCEL_SAT_X) == static_cast<std::uint32_t>(ImuFlag::AccelSatX));
static_assert(static_cast<std::uint32_t>(MARV_IMU_ACCEL_SAT_Y) == static_cast<std::uint32_t>(ImuFlag::AccelSatY));
static_assert(static_cast<std::uint32_t>(MARV_IMU_ACCEL_SAT_Z) == static_cast<std::uint32_t>(ImuFlag::AccelSatZ));
static_assert(static_cast<std::uint32_t>(MARV_IMU_GYRO_VALID) == static_cast<std::uint32_t>(ImuFlag::GyroValid));
static_assert(static_cast<std::uint32_t>(MARV_IMU_ACCEL_VALID) == static_cast<std::uint32_t>(ImuFlag::AccelValid));
static_assert(static_cast<std::uint32_t>(MARV_IMU_TEMP_VALID) == static_cast<std::uint32_t>(ImuFlag::TempValid));
static_assert(static_cast<std::uint32_t>(MARV_IMU_FLAG_COUNT) == static_cast<std::uint32_t>(ImuFlag::Count));
static_assert(static_cast<std::uint32_t>(MARV_PARAM_F32) == static_cast<std::uint32_t>(ParamType::F32));
static_assert(static_cast<std::uint32_t>(MARV_PARAM_I32) == static_cast<std::uint32_t>(ParamType::I32));

constexpr std::size_t kMotors = composition::kMotors;
constexpr std::size_t kServos = composition::kServos;

enum class Lifecycle : std::uint8_t { Uninit, Ready, Done };

struct Session {
  Lifecycle state = Lifecycle::Uninit;
  hal_sim::TickPeriod period{};
  Tick ticks_run = 0;
  std::uint32_t error_index = 0;
  std::array<DshotValue, kMotors> motor_latch{};
  std::array<ServoUs, kServos> servo_latch{};
  std::array<ParamRecord, kParamCount> merged{};
};

Session g_session;

[[nodiscard]] marv_sil_status fail_at(marv_sil_status status, std::uint32_t index) noexcept {
  g_session.error_index = index;
  return status;
}

[[nodiscard]] bool has(const marv_imu_meas& m, ImuFlag f) noexcept { return (m.flags & imu_flag(f)) != 0; }

[[nodiscard]] bool any_set(const marv_imu_meas& m, ImuFlag a, ImuFlag b, ImuFlag c) noexcept {
  return has(m, a) || has(m, b) || has(m, c);
}

[[nodiscard]] bool all_finite(const marv_imu_meas& m) noexcept {
  return std::isfinite(m.gyro_rad_s.x) && std::isfinite(m.gyro_rad_s.y) && std::isfinite(m.gyro_rad_s.z) &&
         std::isfinite(m.accel_m_s2.x) && std::isfinite(m.accel_m_s2.y) && std::isfinite(m.accel_m_s2.z) &&
         std::isfinite(m.temp_k);
}

[[nodiscard]] bool vec_is_zero(const marv_vec3f& v) noexcept { return v.x == 0.0f && v.y == 0.0f && v.z == 0.0f; }

// core 3 IMU semantics at the boundary: finite fields, no reserved bits, a clear valid bit means a zero field and
// clear saturation bits, TempValid means temp_k > 0.
[[nodiscard]] bool sample_valid(const marv_imu_meas& m) noexcept {
  if (!all_finite(m) || (m.flags & ~kImuFlagsDefined) != 0) {
    return false;
  }
  if (!has(m, ImuFlag::GyroValid) &&
      (!vec_is_zero(m.gyro_rad_s) || any_set(m, ImuFlag::GyroSatX, ImuFlag::GyroSatY, ImuFlag::GyroSatZ))) {
    return false;
  }
  if (!has(m, ImuFlag::AccelValid) &&
      (!vec_is_zero(m.accel_m_s2) || any_set(m, ImuFlag::AccelSatX, ImuFlag::AccelSatY, ImuFlag::AccelSatZ))) {
    return false;
  }
  if (has(m, ImuFlag::TempValid)) {
    return m.temp_k > 0.0f;
  }
  return m.temp_k == 0.0f;
}

[[nodiscard]] ImuSample to_sample(const marv_imu_meas& m, TimeUs t_us) noexcept {
  ImuSample s{};
  s.t_us = t_us;
  s.gyro_rad_s = prim::Vec3<float>{m.gyro_rad_s.x, m.gyro_rad_s.y, m.gyro_rad_s.z};
  s.accel_m_s2 = prim::Vec3<float>{m.accel_m_s2.x, m.accel_m_s2.y, m.accel_m_s2.z};
  s.temp_k = m.temp_k;
  s.flags = m.flags;
  return s;
}

[[nodiscard]] bool present(const void* p, std::size_t count) noexcept { return count == 0 || p != nullptr; }

}  // namespace

SessionView session_view() noexcept {
  return SessionView{g_session.state == Lifecycle::Ready, g_session.ticks_run, g_session.period};
}

}  // namespace marv::sil

extern "C" {

[[gnu::visibility("default")]] marv_sil_status marv_sil_info_get(marv_sil_info* info) {
  using namespace marv;
  if (info == nullptr) {
    return MARV_SIL_E_NULL;
  }
  if (info->struct_size != sizeof(marv_sil_info)) {
    return MARV_SIL_E_ABI;
  }
  info->n_motors = static_cast<std::uint32_t>(composition::kMotors);
  info->n_servos = static_cast<std::uint32_t>(composition::kServos);
  info->n_params = static_cast<std::uint32_t>(kParamCount);
  info->param_schema_hash = kParamSchemaHash;
  info->composition = composition::kName;
  return MARV_SIL_OK;
}

[[gnu::visibility("default")]] marv_sil_status marv_sil_init(const marv_sil_config* cfg) {
  using namespace marv;
  using namespace marv::sil;
  if (g_session.state != Lifecycle::Uninit) {
    return MARV_SIL_E_STATE;
  }
  if (cfg == nullptr) {
    return MARV_SIL_E_NULL;
  }
  if (cfg->struct_size != sizeof(marv_sil_config) || cfg->imu_meas_size != sizeof(marv_imu_meas) ||
      cfg->override_size != sizeof(marv_sil_param_override)) {
    return MARV_SIL_E_ABI;
  }
  if ((cfg->n_overrides == 0) != (cfg->overrides == nullptr)) {
    return MARV_SIL_E_NULL;
  }
  const hal_sim::TickPeriod period{cfg->tick_period_num_us, cfg->tick_period_den};
  if (!hal_sim::period_valid(period)) {
    return MARV_SIL_E_PERIOD;
  }
  if (cfg->param_schema_hash != kParamSchemaHash) {
    return MARV_SIL_E_SCHEMA;
  }
  const std::span<const marv_sil_param_override> overrides{cfg->overrides, cfg->n_overrides};
  const OverrideStatus checked = validate_overrides(param_defaults(), overrides);
  if (!checked.ok) {
    return fail_at(MARV_SIL_E_PARAM, checked.index);
  }

  g_session.period = period;
  hal_sim::setup(period, std::span<DshotValue>{g_session.motor_latch}, std::span<ServoUs>{g_session.servo_latch});
  const OverrideStatus applied =
      apply_overrides(param_defaults(), overrides, std::span<ParamRecord, kParamCount>{g_session.merged});
  if (!applied.ok) {
    hal_panic("marv_sil_init: validated overrides were rejected");
  }
  if (!params_init(std::span<const ParamRecord, kParamCount>{g_session.merged})) {
    hal_panic("marv_sil_init: params_init rejected the merged table");
  }
  composition::init();
  g_session.ticks_run = 0;
  g_session.state = Lifecycle::Ready;
  return MARV_SIL_OK;
}

[[gnu::visibility("default")]] marv_sil_status marv_sil_tick(std::uint64_t first_tick, std::uint32_t k,
                                                             const marv_imu_meas* imu, marv_sil_out* out) {
  using namespace marv;
  using namespace marv::sil;
  if (g_session.state != Lifecycle::Ready) {
    return MARV_SIL_E_STATE;
  }
  if (imu == nullptr || out == nullptr) {
    return MARV_SIL_E_NULL;
  }
  if (out->struct_size != sizeof(marv_sil_out)) {
    return MARV_SIL_E_ABI;
  }
  if (out->t_us == nullptr || !present(out->dshot, kMotors) || !present(out->servo_us, kServos)) {
    return MARV_SIL_E_NULL;
  }
  if (first_tick != g_session.ticks_run) {
    return MARV_SIL_E_TICK;
  }
  if (k == 0 || k > out->capacity_ticks) {
    return MARV_SIL_E_COUNT;
  }
  for (std::uint32_t i = 0; i < k; ++i) {
    if (!sample_valid(imu[i])) {
      return fail_at(MARV_SIL_E_INPUT, i);
    }
  }

  for (std::uint32_t i = 0; i < k; ++i) {
    const Tick n = first_tick + i;
    const TimeUs t_us = hal_sim::stamp_us(g_session.period, n);
    hal_sim::begin_tick(n);
    composition::tick(to_sample(imu[i], t_us));
    hal_sim::end_tick();
    g_session.ticks_run = n + 1;

    out->t_us[i] = t_us;
    for (std::size_t m = 0; m < kMotors; ++m) {
      out->dshot[(i * kMotors) + m] = g_session.motor_latch[m].raw();
    }
    for (std::size_t s = 0; s < kServos; ++s) {
      out->servo_us[(i * kServos) + s] = g_session.servo_latch[s].us;
    }
  }
  return MARV_SIL_OK;
}

[[gnu::visibility("default")]] marv_sil_status marv_sil_shutdown(void) {
  using namespace marv::sil;
  if (g_session.state != Lifecycle::Ready) {
    return MARV_SIL_E_STATE;
  }
  g_session.state = Lifecycle::Done;
  return MARV_SIL_OK;
}

[[gnu::visibility("default")]] const char* marv_sil_status_str(marv_sil_status s) {
  switch (s) {
    case MARV_SIL_OK:
      return "ok";
    case MARV_SIL_E_NULL:
      return "E_NULL: a required pointer is NULL";
    case MARV_SIL_E_ABI:
      return "E_ABI: a struct size differs from the library's";
    case MARV_SIL_E_STATE:
      return "E_STATE: the call is not allowed in the current lifecycle state";
    case MARV_SIL_E_PERIOD:
      return "E_PERIOD: the tick period is invalid (den >= 1 and num_us >= den required)";
    case MARV_SIL_E_SCHEMA:
      return "E_SCHEMA: the parameter schema hash differs from the library's";
    case MARV_SIL_E_PARAM:
      return "E_PARAM: a parameter override is invalid (see marv_sil_error_index)";
    case MARV_SIL_E_TICK:
      return "E_TICK: first_tick is not the number of ticks already run";
    case MARV_SIL_E_COUNT:
      return "E_COUNT: k is 0 or exceeds the output capacity";
    case MARV_SIL_E_INPUT:
      return "E_INPUT: an IMU sample is invalid (see marv_sil_error_index)";
    default:
      return "unknown status";
  }
}

[[gnu::visibility("default")]] std::uint32_t marv_sil_error_index(void) {
  return marv::sil::g_session.error_index;
}

}  // extern "C"
