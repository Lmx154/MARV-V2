#include "marv/log/arena_logger.hpp"

#include <cstdint>
#include <memory>
#include <new>
#include <span>

#include "marv/log/format.hpp"
#include "marv/store/arena.hpp"

namespace marv::log {

void ArenaLogger::service(TimeUs now, bool armed) noexcept {
  now_ = now;
  armed_ = armed;
  if (!held_) {
    take_lease();
  }
  if (logger_) {
    logger_->service(now, armed);
  }
}

void ArenaLogger::take_lease() noexcept {
  if (config_ == nullptr) {
    return;
  }
  const std::span<std::uint8_t> arena = store::arena_lease(store::ArenaHolder{&ArenaLogger::yield_thunk, this});
  if (arena.empty()) {
    return;
  }
  held_ = true;
  if (staging_bytes_ % kSectorBytes != 0 || staging_bytes_ / kSectorBytes < 2 || staging_bytes_ > arena.size()) {
    return;
  }
  void* start = arena.data() + staging_bytes_;
  std::size_t space = arena.size() - staging_bytes_;
  if (std::align(alignof(Cell), sizeof(Cell), start, space) == nullptr) {
    return;
  }
  const std::size_t cells = space / sizeof(Cell);
  if (cells <= cells_for(kMaxRingRecordBytes)) {
    return;
  }
  ring_.emplace(std::span<Cell>{std::launder(static_cast<Cell*>(start)), cells});
  logger_.emplace(*ring_, storage_, arena.first(staging_bytes_), config_(cells, config_ctx_), header_);
  published_.store(&*ring_, std::memory_order_release);
}

bool ArenaLogger::yield_thunk(void* ctx) noexcept { return static_cast<ArenaLogger*>(ctx)->yield(); }

bool ArenaLogger::yield() noexcept {
  if (logger_ && !logger_->yield(now_, armed_)) {
    return false;
  }
  published_.store(nullptr, std::memory_order_release);
  logger_.reset();
  ring_.reset();
  held_ = false;
  return true;
}

bool ArenaLogger::push_imu(const ImuSample& s) noexcept {
  Ring* r = published();
  return r != nullptr && r->push_imu(s);
}

bool ArenaLogger::push_imu_raw(const ImuSample& s, const ImuRawScales& sc) noexcept {
  Ring* r = published();
  return r != nullptr && r->push_imu_raw(s, sc);
}

bool ArenaLogger::push_rate_tick(const RateTickRecord& t) noexcept {
  Ring* r = published();
  return r != nullptr && r->push_rate_tick(t);
}

bool ArenaLogger::push_manual(const ManualControl& m) noexcept {
  Ring* r = published();
  return r != nullptr && r->push_manual(m);
}

}  // namespace marv::log
