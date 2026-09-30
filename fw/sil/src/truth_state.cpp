#include "marv_truth.h"

#include <cmath>
#include <cstddef>
#include <cstdint>

#include <marv/composition.hpp>
#include <marv/hal_sim/hal_sim.hpp>
#include <marv/prim/quat.hpp>
#include <marv/prim/vec.hpp>
#include <marv/types/attitude_state.hpp>

#include "sil_session.hpp"

namespace marv::sil {
namespace {

static_assert(sizeof(marv_vec3f) == sizeof(prim::Vec3<float>));
static_assert(static_cast<std::size_t>(MARV_TRUTH_Q_COUNT) == sizeof(prim::Quat<float>) / sizeof(float));

constexpr std::uint32_t kValidBit = std::uint32_t{1} << static_cast<unsigned>(MARV_TRUTH_ATTITUDE_VALID);
constexpr std::uint32_t kDefinedFlags = (std::uint32_t{1} << static_cast<unsigned>(MARV_TRUTH_FLAG_COUNT)) - 1;

[[nodiscard]] bool q_is_zero(const marv_truth_state& s) noexcept {
  return s.q_wxyz[MARV_TRUTH_Q_W] == 0.0f && s.q_wxyz[MARV_TRUTH_Q_X] == 0.0f && s.q_wxyz[MARV_TRUTH_Q_Y] == 0.0f &&
         s.q_wxyz[MARV_TRUTH_Q_Z] == 0.0f;
}

[[nodiscard]] bool omega_is_zero(const marv_truth_state& s) noexcept {
  return s.omega_frd_rad_s.x == 0.0f && s.omega_frd_rad_s.y == 0.0f && s.omega_frd_rad_s.z == 0.0f;
}

[[nodiscard]] bool all_finite(const marv_truth_state& s) noexcept {
  return std::isfinite(s.q_wxyz[MARV_TRUTH_Q_W]) && std::isfinite(s.q_wxyz[MARV_TRUTH_Q_X]) &&
         std::isfinite(s.q_wxyz[MARV_TRUTH_Q_Y]) && std::isfinite(s.q_wxyz[MARV_TRUTH_Q_Z]) &&
         std::isfinite(s.omega_frd_rad_s.x) && std::isfinite(s.omega_frd_rad_s.y) &&
         std::isfinite(s.omega_frd_rad_s.z);
}

// Reserved bits clear; a clear valid bit means every field is zero (the IMU rule); a set valid bit means finite
// fields and a quaternion that is not zero.
[[nodiscard]] bool state_valid(const marv_truth_state& s) noexcept {
  if ((s.flags & ~kDefinedFlags) != 0) {
    return false;
  }
  if ((s.flags & kValidBit) == 0) {
    return q_is_zero(s) && omega_is_zero(s);
  }
  return all_finite(s) && !q_is_zero(s);
}

}  // namespace
}  // namespace marv::sil

extern "C" {

[[gnu::visibility("default")]] marv_sil_status marv_truth_state_set(const marv_truth_state* s) {
  using namespace marv;
  using namespace marv::sil;
  const SessionView session = session_view();
  if (!session.ready) {
    return MARV_SIL_E_STATE;
  }
  if (s == nullptr) {
    return MARV_SIL_E_NULL;
  }
  if (s->struct_size != sizeof(marv_truth_state)) {
    return MARV_SIL_E_ABI;
  }
  if (s->tick != session.ticks_run) {
    return MARV_SIL_E_TICK;
  }
  if (!state_valid(*s)) {
    return MARV_SIL_E_INPUT;
  }
  const AttitudeState<float> a{
      hal_sim::stamp_us(session.period, s->tick),
      prim::Quat<float>{s->q_wxyz[MARV_TRUTH_Q_W], s->q_wxyz[MARV_TRUTH_Q_X], s->q_wxyz[MARV_TRUTH_Q_Y],
                        s->q_wxyz[MARV_TRUTH_Q_Z]},
      prim::Vec3<float>{s->omega_frd_rad_s.x, s->omega_frd_rad_s.y, s->omega_frd_rad_s.z},
      (s->flags & kValidBit) != 0};
  composition::attitude_input(a);
  return MARV_SIL_OK;
}

}  // extern "C"
