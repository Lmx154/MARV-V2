// The bench motor-step profile's status (unit U3 BENCH-PROFILE): declared for the bench image's console and the L09 tests.
// Only the composition variant marv_composition_quad_fc_bench_profile (quad_fc.cpp built with MARV_QUAD_FC_BENCH_PROFILE)
// defines it; the flight composition does not, and no flight image may contain it (invariant L-6).
#pragma once

#include <cstdint>

#include "marv/arming/arming.hpp"

namespace marv::composition {

// Where the profile of the latest arm is. Idle: no arm edge since init. LeadIn and Running: live (it may still write). The
// others are final until the next arm edge: Done (the last level ended), Capped (the total-duration cap reached first),
// SwitchOut (the bench switch was not in position on a tick since the arm edge, that edge's tick included), NotArmed (the
// arming state was not Armed on a tick: the arm switch off, a link-loss hold or centring, any disarm), StampBack (a sample
// stamp before the arm edge's).
enum class BenchProfilePhase : std::uint8_t { Idle, LeadIn, Running, Done, Capped, SwitchOut, NotArmed, StampBack };

// As of the latest rate tick. arms: arm edges since init. level: the 0-based level of the latest Running tick. motor and
// dshot: the logical motor (1..4) and the value of the latest tick's profile write, 0 and 0 when it wrote stop. kill_state:
// the arming state on the tick the profile became final (meaningful for NotArmed). Relaxed atomic reads of values the rate
// tick publishes one after the other, callable from any context; a read that overlaps a rate tick can mix two ticks.
struct BenchProfileStatus {
  BenchProfilePhase phase = BenchProfilePhase::Idle;
  arming::ArmState kill_state = arming::ArmState::Boot;
  std::uint32_t arms = 0;
  std::uint32_t level = 0;
  std::uint32_t motor = 0;
  std::uint32_t dshot = 0;
};
[[nodiscard]] BenchProfileStatus bench_profile_status() noexcept;

}  // namespace marv::composition
