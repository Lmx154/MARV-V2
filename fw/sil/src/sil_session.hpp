// The part of the SIL session that the truth-state entry reads. Defined in marv_sil.cpp.
#pragma once

#include <marv/hal_sim/hal_sim.hpp>
#include <marv/types/time.hpp>

namespace marv::sil {

struct SessionView {
  bool ready;
  Tick ticks_run;
  hal_sim::TickPeriod period;
};

[[nodiscard]] SessionView session_view() noexcept;

}  // namespace marv::sil
