// The disarmed arena (onboard logger unit U4-2; serena memory quad-m4/architect-logger-u4-2026-10-20, decision 1b and
// invariants L-8, L-8b): one contiguous static block of RAM. Its first bytes are the config store's working memory
// (BootInputs::scratch, marv/store/store.hpp), used by the boot step and by an activation; while the store is idle one holder
// (the onboard logger, marv/log/arena_logger.hpp) may lease the whole block for its staging and its ring. The lease carries the
// SD card's FatFs volume with it: whoever holds the arena is the volume's only user (the store's pending file, the logger's
// files).
//
// One owner at a time:
//   - Nothing can be leased before store::boot has registered the arena (arena_open), so the boot step has it to itself.
//   - arena_lease hands the whole arena to one holder; while it is leased every other lease is refused.
//   - After its boot step the store touches the arena or the volume only after arena_reclaim: an idle arena is the store's at
//     once; a leased one is asked back from its holder (ArenaHolder::yield). A holder that refuses keeps the arena and the
//     volume, and the store refuses the operation (store::activate).
//   - A holder yields only when it can (the logger never while armed, Logging or PostRoll; a Ready logger yields). When yield
//     returns true it has stopped every use of the arena and the volume: the tick no longer reaches the ring, the file is
//     closed and the volume unmounted.
// Background loop and boot only: the store's commands and the holder's service both run in the background loop, so a lease
// never changes hands inside either. The tick never calls these; it reaches the arena only through the ring pointer the holder
// publishes, which the holder can withdraw safely only because the tick (the IMU interrupt) and the background loop run on one
// core, core 0 (invariant L-8b; the flight image's post-link check tools/ci/check_single_core.py forbids
// multicore_launch_core1). Arming is never gated on the lease (decision 0053): nothing here reads or changes the arming state.
#pragma once

#include <cstdint>
#include <span>

namespace marv::store {

struct ArenaHolder {
  // Asked from the background loop to give the arena and the volume back. True: done, the lease is over; false: not now.
  bool (*yield)(void* ctx) noexcept = nullptr;
  void* ctx = nullptr;
};

// store::boot only, once, when its boot step is done with the arena: from then on `arena` (empty: nothing to lend) can be leased.
void arena_open(std::span<std::uint8_t> arena) noexcept;

// The whole arena for `holder`, or an empty span (nothing leased): before arena_open, for an empty arena, for a holder that
// cannot be asked to yield, or while the arena is leased.
[[nodiscard]] std::span<std::uint8_t> arena_lease(const ArenaHolder& holder) noexcept;

[[nodiscard]] bool arena_leased() noexcept;

// The store, before it touches the arena or the volume after its boot step. True: both are the store's until it returns to the
// background loop (the arena was idle, or its holder yielded and the lease is over). False: the holder refused and keeps both.
[[nodiscard]] bool arena_reclaim() noexcept;

}  // namespace marv::store
