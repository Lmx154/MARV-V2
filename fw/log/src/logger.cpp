#include "marv/log/logger.hpp"

#include <algorithm>

namespace marv::log {

bool Logger::append(std::span<const std::uint8_t> record) noexcept {
  // Stop at a record boundary while two chunks of the region are still free: one for the chunk being filled, one for the End
  // record and the padding of the last sector. So a full file ends on whole records and the End record always fits.
  if (!finalizing_ && bytes_written_ + 2 * framer_.staging_bytes() + record.size() > capacity_) {
    full_ = true;
    return false;
  }
  return framer_.append(record, *this);
}

bool Logger::write_chunk(std::span<const std::uint8_t> chunk) noexcept {
  if (bytes_written_ + chunk.size() > capacity_) {
    full_ = true;
    return false;
  }
  if (storage_.write(chunk) != StorageStatus::Ok) {
    bump(StatusField::WriteFailures);
    return false;
  }
  bytes_written_ += chunk.size();
  bump(StatusField::ChunksWritten);
  return true;
}

void Logger::try_open(TimeUs now) noexcept {
  next_probe_ = now + config_.reprobe_period_us;
  const Prepared p = storage_.prepare(config_.file_bytes);
  if (p.status != StorageStatus::Ok) {
    switch (p.status) {
      case StorageStatus::NoCard:
        bump(StatusField::PrepareAbsent);
        state_ = LoggerState::Absent;
        break;
      case StorageStatus::Full:
        bump(StatusField::PrepareFull);
        state_ = LoggerState::Full;
        break;
      default:
        bump(StatusField::PrepareError);
        state_ = LoggerState::Fault;
        break;
    }
    return;
  }
  capacity_ = p.capacity_bytes;
  bytes_written_ = 0;
  full_ = false;
  finalizing_ = false;
  file_open_ = true;
  session_ = config_.session_id != nullptr ? config_.session_id(config_.session_ctx) : 0;
  framer_.begin(session_);
  header_.session_start_us = now;
  if (!write_header(header_, framer_, *this)) {
    bump(StatusField::PrepareError);
    state_ = full_ ? LoggerState::Full : LoggerState::Fault;
    return;
  }
  state_ = LoggerState::Ready;
}

void Logger::start_logging(TimeUs now) noexcept {
  bump(StatusField::Sessions);
  state_ = LoggerState::Logging;
  next_status_ = now + config_.status_period_us;
  next_slow_ = now + config_.slow_period_us;
  write_status(now);
}

void Logger::write_status(TimeUs now) noexcept {
  const RingStats rs = ring_.stats();
  for (std::size_t i = 0; i < kRingKinds; ++i) {
    counters_[static_cast<std::size_t>(StatusField::DropsImu) + i] = rs.dropped[i];
    counters_[static_cast<std::size_t>(StatusField::PushedImu) + i] = rs.pushed[i];
  }
  counters_[static_cast<std::size_t>(StatusField::RingHighWaterCells)] = rs.high_water_cells;
  counters_[static_cast<std::size_t>(StatusField::RingBacklogCells)] = static_cast<std::uint32_t>(ring_.backlog_cells());
  counters_[static_cast<std::size_t>(StatusField::BytesWrittenLo)] = static_cast<std::uint32_t>(bytes_written_);
  std::array<std::uint8_t, status::kBytes> r{};
  r[kLeadOffKind] = static_cast<std::uint8_t>(Kind::Status);
  r[kLeadOffSub] = static_cast<std::uint8_t>(state_);
  put<std::uint16_t>(r, kLeadOffLength, static_cast<std::uint16_t>(r.size()));
  put<std::uint64_t>(r, status::kOffTime, now);
  for (std::size_t i = 0; i < kStatusFields; ++i) {
    put<std::uint32_t>(r, status::kOffCounters + i * sizeof(std::uint32_t), counters_[i]);
  }
  if (!append(r)) {
    on_sink_failure();
  }
}

void Logger::write_telemetry(TimeUs now) noexcept {
  const TelemetryHook* t = header_.telemetry;
  if (t == nullptr || t->fill == nullptr) {
    return;
  }
  std::array<std::uint8_t, kSectorBytes> r{};
  const std::size_t n = t->fill(t->ctx, std::span<std::uint8_t>{r}.subspan(telemetry::kOffBody));
  if (n == 0 || n > r.size() - telemetry::kOffBody) {
    return;
  }
  const std::size_t len = telemetry::kOffBody + n;
  r[kLeadOffKind] = static_cast<std::uint8_t>(Kind::Telemetry);
  put<std::uint16_t>(r, kLeadOffLength, static_cast<std::uint16_t>(len));
  put<std::uint32_t>(r, telemetry::kOffTime, low32(now));
  bump(StatusField::TelemetryRecords);
  if (!append(std::span<const std::uint8_t>{r}.first(len))) {
    on_sink_failure();
  }
}

void Logger::pump(TimeUs now) noexcept {
  std::array<std::uint8_t, kMaxRingRecordBytes> rec{};
  for (;;) {
    const std::size_t n = ring_.pop(rec);
    if (n == 0) {
      break;
    }
    if (!append(std::span<const std::uint8_t>{rec}.first(n))) {
      on_sink_failure();
      return;
    }
  }
  if (config_.slow_period_us != 0 && now >= next_slow_) {
    next_slow_ = now + config_.slow_period_us;
    write_telemetry(now);
    if (state_ != LoggerState::Logging && state_ != LoggerState::PostRoll) {
      return;
    }
  }
  if (config_.status_period_us != 0 && now >= next_status_) {
    next_status_ = now + config_.status_period_us;
    write_status(now);
  }
}

void Logger::on_sink_failure() noexcept {
  if (!full_) {
    state_ = LoggerState::Fault;
    return;
  }
  state_ = LoggerState::Full;
  if (framer_.failed()) {
    framer_.discard_staged();  // a chunk was refused for want of room: the sectors it held are gone
  }
  finalizing_ = true;
  write_end(now_, EndReason::Full);
}

void Logger::write_end(TimeUs now, EndReason reason) noexcept {
  std::array<std::uint8_t, end::kBytes> r{};
  r[kLeadOffKind] = static_cast<std::uint8_t>(Kind::End);
  r[kLeadOffSub] = static_cast<std::uint8_t>(reason);
  put<std::uint16_t>(r, kLeadOffLength, static_cast<std::uint16_t>(r.size()));
  put<std::uint64_t>(r, end::kOffTime, now);
  put<std::uint32_t>(r, end::kOffSectors, framer_.next_sequence() + 1);
  if (!framer_.append(r, *this) || !framer_.flush(*this)) {
    if (state_ != LoggerState::Full) {
      state_ = LoggerState::Fault;
    }
  }
}

void Logger::close_file(TimeUs now, TimeUs next_probe) noexcept {
  if (file_open_) {
    if (storage_.close(bytes_written_) != StorageStatus::Ok) {
      bump(StatusField::CloseFailures);
    }
    file_open_ = false;
  }
  state_ = LoggerState::Absent;
  next_probe_ = std::max(now, next_probe);
}

bool Logger::yield(TimeUs now, bool armed) noexcept {
  if (armed || state_ == LoggerState::Logging || state_ == LoggerState::PostRoll) {
    return false;
  }
  close_file(now, now);
  storage_.release();
  return true;
}

void Logger::service(TimeUs now, bool armed) noexcept {
  now_ = now;
  switch (state_) {
    case LoggerState::Absent:
      counters_[static_cast<std::size_t>(StatusField::DiscardedCells)] +=
          static_cast<std::uint32_t>(ring_.discard_oldest(config_.pre_roll_cells));
      if (!armed && now >= next_probe_) {
        try_open(now);
      }
      break;
    case LoggerState::Ready:
      if (armed) {
        start_logging(now);  // the ring holds the pre-roll and what arrived since the last pass: all of it is logged
        pump(now);
      } else {
        counters_[static_cast<std::size_t>(StatusField::DiscardedCells)] +=
            static_cast<std::uint32_t>(ring_.discard_oldest(config_.pre_roll_cells));
      }
      break;
    case LoggerState::Logging:
      pump(now);
      if (state_ == LoggerState::Logging && !armed) {
        state_ = LoggerState::PostRoll;
        disarmed_at_ = now;
      }
      break;
    case LoggerState::PostRoll:
      pump(now);
      if (state_ != LoggerState::PostRoll) {
        break;
      }
      if (armed) {
        state_ = LoggerState::Logging;
      } else if (now - disarmed_at_ >= config_.post_roll_us) {
        write_status(now);
        if (state_ == LoggerState::PostRoll) {
          write_end(now, EndReason::Disarm);
        }
        close_file(now, now);
      }
      break;
    case LoggerState::Full:
    case LoggerState::Fault:
      counters_[static_cast<std::size_t>(StatusField::DiscardedCells)] +=
          static_cast<std::uint32_t>(ring_.discard_oldest(config_.pre_roll_cells));
      if (!armed) {
        if (file_open_) {
          close_file(now, now + config_.reprobe_period_us);
        } else if (now >= next_probe_) {
          try_open(now);
        }
      }
      break;
  }
}

}  // namespace marv::log
