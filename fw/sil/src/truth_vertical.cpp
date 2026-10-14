#include "marv_truth_vertical.h"

#include <cmath>
#include <cstdint>

#include <marv/composition.hpp>
#include <marv/hal_sim/hal_sim.hpp>
#include <marv/types/vertical_state.hpp>

#include "sil_session.hpp"

namespace marv::sil {
namespace {

constexpr std::uint32_t kValidBit = std::uint32_t{1} << static_cast<unsigned>(MARV_TRUTH_VERTICAL_VALID);
constexpr std::uint32_t kDefinedFlags = (std::uint32_t{1} << static_cast<unsigned>(MARV_TRUTH_VERTICAL_FLAG_COUNT)) - 1;

[[nodiscard]] bool fields_are_zero(const marv_truth_vertical& s) noexcept {
  return s.z_ned_m == 0.0f && s.vz_ned_m_s == 0.0f;
}

[[nodiscard]] bool all_finite(const marv_truth_vertical& s) noexcept {
  return std::isfinite(s.z_ned_m) && std::isfinite(s.vz_ned_m_s);
}

// Reserved bits clear; a clear valid bit means every field is zero (the IMU rule); a set valid bit means finite
// fields.
[[nodiscard]] bool state_valid(const marv_truth_vertical& s) noexcept {
  if ((s.flags & ~kDefinedFlags) != 0) {
    return false;
  }
  if ((s.flags & kValidBit) == 0) {
    return fields_are_zero(s);
  }
  return all_finite(s);
}

}  // namespace
}  // namespace marv::sil

extern "C" {

[[gnu::visibility("default")]] marv_sil_status marv_truth_vertical_set(const marv_truth_vertical* s) {
  using namespace marv;
  using namespace marv::sil;
  const SessionView session = session_view();
  if (!session.ready) {
    return MARV_SIL_E_STATE;
  }
  if (s == nullptr) {
    return MARV_SIL_E_NULL;
  }
  if (s->struct_size != sizeof(marv_truth_vertical)) {
    return MARV_SIL_E_ABI;
  }
  if (s->tick != session.ticks_run) {
    return MARV_SIL_E_TICK;
  }
  if (!state_valid(*s)) {
    return MARV_SIL_E_INPUT;
  }
  const VerticalState<float> v{hal_sim::stamp_us(session.period, s->tick), s->z_ned_m, s->vz_ned_m_s,
                               (s->flags & kValidBit) != 0};
  composition::vertical_input(v);
  return MARV_SIL_OK;
}

}  // extern "C"
