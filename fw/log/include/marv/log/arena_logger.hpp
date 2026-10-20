// The onboard logger on the disarmed arena (onboard logger unit U4-2; serena memory quad-m4/architect-logger-u4-2026-10-20,
// invariants L-8 and L-8b): the logger's side of the arena lease (marv/store/arena.hpp). From the background loop it leases the
// whole arena while the store is idle, makes its staging (the first staging_bytes) and its ring (the rest, from the first cell
// boundary) there, publishes the ring to the tick and services the logger on them. Asked to yield it refuses while armed, Logging
// or PostRoll (Logger::yield); otherwise the logger closes its file and gives the card back, the ring is withdrawn from the tick,
// and the lease ends. The logger and the ring are made afresh on the next lease (the store has used the arena in between), so
// the logger's counters start again at zero.
//
// The tick reaches the arena only through the published ring pointer (the push_* functions below); after a yield no push
// touches the arena. Withdrawing the pointer is safe only because the tick (the IMU interrupt) and the background loop share
// core 0: a push that loaded the pointer runs to its end before the background loop resumes, so no push is still writing when
// the store takes the arena (L-8b). Moving the tick or the background loop to core 1 needs a quiescence handshake first; the
// flight image's post-link check (tools/ci/check_single_core.py) forbids multicore_launch_core1.
//
// Arming is never gated on any of this (decision 0053). Never hal_panic (invariant L-3): an arena that cannot hold the staging
// and a ring is held unused (and given back at once when asked), and nothing is logged.
#pragma once

#include <atomic>
#include <cstddef>
#include <optional>

#include "marv/log/header.hpp"
#include "marv/log/logger.hpp"
#include "marv/log/records.hpp"
#include "marv/log/ring.hpp"
#include "marv/log/storage.hpp"
#include "marv/types/imu_sample.hpp"
#include "marv/types/manual_control.hpp"
#include "marv/types/time.hpp"

namespace marv::log {

// The logger's configuration for a ring of `ring_cells` cells (the pre-roll is derived from the ring: params.hpp
// logger_config_from_params).
using ConfigForRing = LoggerConfig (*)(std::size_t ring_cells, void* ctx) noexcept;

class ArenaLogger {
 public:
  // `staging_bytes`: whole sectors, at least two (logger.hpp).
  ArenaLogger(LogStorage& storage, std::size_t staging_bytes, ConfigForRing config, void* config_ctx,
              const HeaderInfo& header) noexcept
      : storage_(storage), staging_bytes_(staging_bytes), config_(config), config_ctx_(config_ctx), header_(header) {}

  ArenaLogger(const ArenaLogger&) = delete;
  ArenaLogger& operator=(const ArenaLogger&) = delete;

  // ---- the background loop ----
  // Without a lease, asks for one (marv::store::arena_lease); then the logger's service. `armed` as Logger::service.
  void service(TimeUs now, bool armed) noexcept;

  [[nodiscard]] bool holds_lease() const noexcept { return held_; }
  // The logger on the leased arena; null without one.
  [[nodiscard]] const Logger* logger() const noexcept { return logger_ ? &*logger_ : nullptr; }

  // ---- the tick context (ring.hpp's producer calls through the published ring pointer; false: no ring, or dropped) ----
  bool push_imu(const ImuSample& s) noexcept;
  bool push_imu_raw(const ImuSample& s, const ImuRawScales& sc) noexcept;
  bool push_rate_tick(const RateTickRecord& t) noexcept;
  bool push_manual(const ManualControl& m) noexcept;

 private:
  static bool yield_thunk(void* ctx) noexcept;
  [[nodiscard]] bool yield() noexcept;
  void take_lease() noexcept;
  [[nodiscard]] Ring* published() noexcept { return published_.load(std::memory_order_acquire); }

  LogStorage& storage_;
  std::size_t staging_bytes_;
  ConfigForRing config_;
  void* config_ctx_;
  HeaderInfo header_;
  std::optional<Ring> ring_;
  std::optional<Logger> logger_;
  std::atomic<Ring*> published_{nullptr};  // stored by the background loop only; loaded by the tick
  bool held_ = false;
  TimeUs now_ = 0;      // of the last service pass
  bool armed_ = false;  // of the last service pass
};

}  // namespace marv::log
