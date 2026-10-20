#pragma once

// The onboard SD logger in the flight image (MARV_FC3V2_FLIGHT_LOG, default OFF; onboard logger unit U4-4; serena memories
// quad-m4/architect-onboard-logger-2026-10-18 items 1, 4, 5 and 7, and quad-m4/architect-logger-u4-2026-10-20): the logger on the
// disarmed arena (marv/log/arena_logger.hpp) with the SD card's log storage, serviced from the background loop beside the link
// service, and the tick hooks of log_tick.hpp, which reach the arena only through the arena logger's published ring pointer:
//   the sample sink       every IMU sample, as the compact raw-count record (log format version 2, Ring::push_imu_raw, with the
//                         board's scales below); a sample that is not a whole number of counts goes as the 32-byte record;
//   hal_actuators_write   the rate-tick record;
//   hal_manual_control    the ManualControl record, once per new frame.
// Before flight_log_start, and while the logger holds no lease, a hook pushes nothing. The logger logs from the arm edge (with the
// pre-roll the ring holds beyond the worst-stall budget, marv/log/budget.hpp) to the disarm edge plus the post-roll
// (log_post_roll_s); while it logs it writes the slow stream, the composition's Telemetry snapshot, every log_slow_period_s. It
// leases the arena when the store is idle and gives it back to the store only while Ready or not logging (never armed, Logging
// or PostRoll; invariant L-8). Background loop only except the hooks; arming is never gated on any of it (decision 0053), the
// card is touched only from flight_log_service, and nothing here calls hal_panic (invariant L-3). No Pico SDK header: the host
// test of the wiring (tests/regression/quad/L09/unit/log_flight_test.cpp) compiles this file's source.
//
// MARV_FC3V2_FLIGHT_LOG_EMB6_OPEN (bench only, needs MARV_FC3V2_FLIGHT_LOG; invariant L-5a): the measurement build, whose ring may
// hold less than the fast stream for the worst-stall budget (core EMB-6 is open there; its post-link L-5 check only warns). Its
// banner prints flight_log_emb6_open_banner, its log header carries log::kBuildFlagEmb6Open, and its console has the trigger
//   log start   the logger is serviced as if armed (Ready -> Logging; the hooks push exactly as in flight) without arming
//               anything; refused while armed or unless the logger is Ready;
//   log stop    ends the trigger: the logger sees the disarm edge (PostRoll, then the End record and the close).
// Every symbol of that code names emb6_open; every other build of the image forbids such a symbol after the link
// (tools/ci/check_emb6_open.py).
#include <span>

#include "marv/log/arena_logger.hpp"
#include "marv/log/header.hpp"
#include "marv/log/records.hpp"
#include "marv/log/storage.hpp"
#include "marv/types/telemetry.hpp"
#include "marv/types/time.hpp"

namespace marv::hal_target {

struct FlightLogStart {
  log::LogStorage* storage = nullptr;  // the card (sd_log_storage)
  log::ConfigForRing config = nullptr;  // the logger's configuration for the ring the lease gives it (marv/log/params.hpp)
  void* config_ctx = nullptr;
  log::HeaderInfo header{};             // telemetry, imu_raw and build_flags are set by flight_log_start
  log::ImuRawScales imu_raw{};          // the IMU's SI per count and the board's mounting signs
  bool (*telemetry)(Telemetry& out) noexcept = nullptr;  // the composition's snapshot (composition::telemetry)
};

// Once, from the background loop, after imu_start (the header carries the IMU's ODR): makes the arena logger, then lets the hooks
// reach it. Its first service pass asks the store for the arena.
void flight_log_start(const FlightLogStart& start) noexcept;

// One background-loop pass. `armed`: the arming machine's Armed state as the loop reads it.
void flight_log_service(TimeUs now, bool armed) noexcept;

// The arena logger; null before flight_log_start.
[[nodiscard]] const log::ArenaLogger* flight_log() noexcept;

// The fields of the slow stream's records as the header describes them: every member of Telemetry in order, an array's
// elements one by one (x y z, w x y z, or the logical index from 1), named after the member.
[[nodiscard]] std::span<const log::FieldDesc> flight_log_telemetry_fields() noexcept;

#if defined(MARV_FC3V2_FLIGHT_LOG_EMB6_OPEN)
extern const char flight_log_emb6_open_banner[];

// One console byte. True when it ended a line that was a `log` command (answered here).
[[nodiscard]] bool flight_log_emb6_open_console_byte(char c, bool armed) noexcept;
#endif

}  // namespace marv::hal_target
