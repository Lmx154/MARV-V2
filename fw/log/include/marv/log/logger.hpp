// The logger: the background-loop consumer of the ring, the writer of sectors to the card (through LogStorage) and the state
// machine of the log file's life. U1 LOG-CORE: host-testable, no HAL, no FatFs, no clock of its own (the caller passes `now`),
// never hal_panic (invariant L-3): every failure is a state and a counter, and nothing here reaches the tick.
//
//   Absent   no file is prepared: no card, or not probed yet. Re-probed (prepare) only while disarmed, every reprobe period.
//   Ready    a file is prepared and its header is written; the ring is kept at most pre_roll_cells deep (older records are
//            discarded and counted), so the arm edge finds the last moments before it.
//   Logging  armed: the ring is drained into sectors and written in whole chunks (data writes only: invariant L-9).
//   PostRoll disarmed, still logging until post_roll has passed; a re-arm in it continues the same file.
//   Full     the file's region is used up, or the card has no room: nothing more is written until disarm, when the file is
//            closed (truncated) and the logger returns to Absent.
//   Fault    a write or the header failed (or the card answered with an error): as Full.
// Arming is never gated on any of this (decision 0053: the SD card is never in the boot or arming path).
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "marv/log/format.hpp"
#include "marv/log/framer.hpp"
#include "marv/log/header.hpp"
#include "marv/log/ring.hpp"
#include "marv/log/storage.hpp"
#include "marv/types/time.hpp"

namespace marv::log {

enum class LoggerState : std::uint8_t { Absent, Ready, Logging, PostRoll, Full, Fault };

struct LoggerConfig {
  std::uint64_t file_bytes = 0;     // the region asked of the card per file
  TimeUs post_roll_us = 0;
  TimeUs status_period_us = 0;      // 0: no periodic Status record (one is still written at the start and the end)
  TimeUs slow_period_us = 0;        // 0: no Telemetry stream
  TimeUs reprobe_period_us = 0;
  std::size_t pre_roll_cells = 0;   // budget.hpp pre_roll_cells
  std::uint64_t (*session_id)(void* ctx) noexcept = nullptr;  // a fresh 64-bit id per file (the TRNG on the board)
  void* session_ctx = nullptr;
};

class Logger final : private ChunkSink {
 public:
  // `staging`: whole sectors, at least two (static storage on the board). `header.telemetry` may be null.
  Logger(Ring& ring, LogStorage& storage, std::span<std::uint8_t> staging, const LoggerConfig& config,
         const HeaderInfo& header) noexcept
      : ring_(ring), storage_(storage), framer_(staging), config_(config), header_(header) {}

  // One pass of the background loop. `armed` is the arming machine's Armed state as the loop last read it.
  void service(TimeUs now, bool armed) noexcept;

  // The arena lease (invariant L-8, marv/store/arena.hpp; marv/log/arena_logger.hpp), from the background loop. False, changing
  // nothing, while `armed`, Logging or PostRoll: the logger never gives up its ring, its staging or the card then. Otherwise it
  // closes the file it holds (Ready's, or a Full or Fault file not closed yet), gives the card back (LogStorage::release), is
  // Absent, and returns true: it no longer uses the ring, the staging or the card.
  [[nodiscard]] bool yield(TimeUs now, bool armed) noexcept;

  [[nodiscard]] LoggerState state() const noexcept { return state_; }
  [[nodiscard]] const std::array<std::uint32_t, kStatusFields>& counters() const noexcept { return counters_; }
  [[nodiscard]] std::uint64_t session_id() const noexcept { return session_; }
  [[nodiscard]] std::uint64_t bytes_written() const noexcept { return bytes_written_; }
  [[nodiscard]] std::uint64_t capacity_bytes() const noexcept { return capacity_; }

 private:
  bool write_chunk(std::span<const std::uint8_t> chunk) noexcept override;

  [[nodiscard]] bool append(std::span<const std::uint8_t> record) noexcept;
  void try_open(TimeUs now) noexcept;
  void start_logging(TimeUs now) noexcept;
  void pump(TimeUs now) noexcept;
  void on_sink_failure() noexcept;
  void write_end(TimeUs now, EndReason reason) noexcept;
  void close_file(TimeUs now, TimeUs next_probe) noexcept;
  void write_status(TimeUs now) noexcept;
  void write_telemetry(TimeUs now) noexcept;
  void bump(StatusField f) noexcept { ++counters_[static_cast<std::size_t>(f)]; }

  Ring& ring_;
  LogStorage& storage_;
  SectorFramer framer_;
  LoggerConfig config_;
  HeaderInfo header_;

  LoggerState state_ = LoggerState::Absent;
  std::array<std::uint32_t, kStatusFields> counters_{};
  std::uint64_t session_ = 0;
  std::uint64_t capacity_ = 0;
  std::uint64_t bytes_written_ = 0;
  bool file_open_ = false;
  bool data_started_ = false;  // the session entered Logging: the file may hold data sectors (invariant L-10)
  bool full_ = false;        // the sink refused a chunk for want of room
  bool finalizing_ = false;  // the reserve may be used
  TimeUs next_probe_ = 0;
  TimeUs next_status_ = 0;
  TimeUs next_slow_ = 0;
  TimeUs disarmed_at_ = 0;
  TimeUs now_ = 0;
};

}  // namespace marv::log
