#pragma once

// The bench log image's logger runtime (MARV_FC3V2_LOG_BENCH, bench only, default OFF; unit U2 LOG-TARGET; serena memory
// quad-m4/architect-onboard-logger-2026-10-18): the U1 logger (marv/log/logger.hpp) on the ring of log_tick.hpp and the SD
// storage of log_storage_sd.hpp, run from the background loop with a MARV_FC3V2_LOG_STAGING_BYTES staging chunk, plus the
// bench's own console commands. Not compiled into any other image.
//
// Console commands (one per line on the console CDC; every other line goes to the motor-test parser as before):
//   log start         the BENCH TRIGGER: the logger is serviced as if armed (Ready -> Logging; the tick pushes exactly as in
//                     flight) without arming anything. Accepted only while the vehicle is disarmed and the logger is Ready.
//   log stop          ends the bench trigger: the logger sees the disarm edge (PostRoll, then End, truncate and close).
//   log card off|on   off: the storage answers as if no card were in (prepare NoCard, write and close fail) without
//                     touching the card; on: back to the card. The bench's card-absent check.
//   log dump <index> [<bytes>]
//                     streams /marv/log/<index>.mlog (at most <bytes>) to the link CDC (interface 2; no link service runs in
//                     this image), refused while the logger logs or the trigger is on.
//   log hist          prints the chunk write-time histogram since the last `log start`.
// Once per second a `log ...` status line follows the console's status line.
#include "marv/log/header.hpp"
#include "marv/log/logger.hpp"
#include "marv/types/time.hpp"

namespace marv::hal_target {

// After imu_start: makes the logger (state Absent; its first service pass probes the card).
void log_bench_start(const log::LoggerConfig& config, const log::HeaderInfo& header) noexcept;

// One background-loop pass: the logger's service with `armed` or the bench trigger, then a step of a running dump.
void log_bench_service(TimeUs now, bool armed) noexcept;

// One console byte. True when it ended a line that was a `log` command (answered here).
[[nodiscard]] bool log_bench_console_byte(char c, bool armed) noexcept;

// The once-per-second status line.
void log_bench_print(TimeUs now) noexcept;

}  // namespace marv::hal_target
