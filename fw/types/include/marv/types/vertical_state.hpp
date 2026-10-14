#pragma once

#include <type_traits>

#include "marv/types/time.hpp"

namespace marv {

// The vertical state as the vertical loop takes it: down-position and down-velocity in NED (z down), and a validity
// flag. t_us is the stamp of the sample the state belongs to, in the hal_time_us() timebase. If valid is false the
// other fields carry no information.
template <class T>
struct VerticalState {
  TimeUs t_us;
  T z_ned_m;     // m
  T vz_ned_m_s;  // m/s
  bool valid;
};

static_assert(std::is_trivially_copyable_v<VerticalState<float>> && std::is_standard_layout_v<VerticalState<float>>);

}  // namespace marv
