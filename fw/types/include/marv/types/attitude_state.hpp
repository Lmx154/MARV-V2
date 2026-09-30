#pragma once

#include <type_traits>

#include "marv/prim/quat.hpp"
#include "marv/prim/vec.hpp"
#include "marv/types/time.hpp"

namespace marv {

// The attitude estimate as the attitude module takes it: body FRD -> NED Hamilton quaternion, body rates in FRD, and a
// validity flag. t_us is the stamp of the sample the state belongs to, in the hal_time_us() timebase. If valid is
// false the other fields carry no information.
template <class T>
struct AttitudeState {
  TimeUs t_us;
  prim::Quat<T> q;
  prim::Vec3<T> omega_frd;  // rad/s
  bool valid;
};

static_assert(std::is_trivially_copyable_v<AttitudeState<float>> && std::is_standard_layout_v<AttitudeState<float>>);

}  // namespace marv
