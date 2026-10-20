// The SIL's raw RC frame entry (ground-segment unit GX-9, the SIL pilot bridge; a library built with RC_FRAME, fw/sil/CMakeLists.txt).
// A frame is decoded by the function the board's receiver calls (rc_input::crsf_to_controls), with the table's rc_channel_* map,
// so that a channel remap acts alike in SIL and on the board (invariant R-2); the ManualControl it yields is staged as
// marv_sil_stage_manual_control stages one.
#include <cstddef>
#include <cstdint>

#include <marv/hal_sim/hal_sim.hpp>
#include <marv/hal_target/crsf.hpp>
#include <marv/hal_target/rc_input.hpp>
#include <marv/params/param.hpp>
#include <marv/params/param_ids.hpp>
#include <marv/prim/constants.hpp>
#include <marv/types/manual_control.hpp>

#include "marv_sil.h"
#include "sil_session.hpp"

static_assert(static_cast<std::size_t>(MARV_RC_CHANNELS) == marv::prim::kCrsfRcChannels);

namespace {

// The manual control as the receiver last built it (receiver.cpp g_current): frames and the link fields persist across frames.
marv::ManualControl g_current{};

constexpr std::uint16_t kRawMax = static_cast<std::uint16_t>((1U << marv::prim::kCrsfRcChannelBits) - 1U);

}  // namespace

extern "C" {

[[gnu::visibility("default")]] marv_sil_status marv_sil_stage_rc_frame(const marv_rc_frame* frame) {
  using namespace marv;
  using namespace marv::hal_target;
  if (!sil::session_view().ready) {
    return MARV_SIL_E_STATE;
  }
  if (frame == nullptr) {
    return MARV_SIL_E_NULL;
  }
  if (frame->struct_size != sizeof(marv_rc_frame) || frame->reserved != 0) {
    return MARV_SIL_E_ABI;
  }
  CrsfRcChannels rc;
  for (std::size_t i = 0; i < prim::kCrsfRcChannels; ++i) {
    if (frame->raw[i] > kRawMax) {
      return MARV_SIL_E_INPUT;
    }
    rc.raw[i] = frame->raw[i];
  }
  const RcChannelMap map{param_value<ParamId::rc_channel_roll>(), param_value<ParamId::rc_channel_pitch>(),
                         param_value<ParamId::rc_channel_throttle>(), param_value<ParamId::rc_channel_yaw>()};
  ManualControl next = g_current;
  if (!crsf_to_controls(rc, map, next)) {
    return MARV_SIL_E_PARAM;
  }
  next.t_us = frame->t_us;
  ++next.frames;
  if (next.frames == 0) {
    next.frames = 1;
  }
  hal_sim::stage_manual_control(next);
  g_current = next;
  return MARV_SIL_OK;
}

}  // extern "C"
