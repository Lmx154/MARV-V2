#include "marv/log/ring.hpp"

#include <algorithm>

namespace marv::log {

void Ring::write(std::size_t index, std::span<const std::uint8_t> record) noexcept {
  for (std::size_t off = 0; off < record.size(); off += kCellBytes) {
    std::copy_n(record.begin() + static_cast<std::ptrdiff_t>(off), kCellBytes, cells_[index].bytes.begin());
    index = advance(index, 1);
  }
}

void Ring::read(std::size_t index, std::span<std::uint8_t> out) const noexcept {
  for (std::size_t off = 0; off < out.size(); off += kCellBytes) {
    std::copy(cells_[index].bytes.begin(), cells_[index].bytes.end(), out.begin() + static_cast<std::ptrdiff_t>(off));
    index = advance(index, 1);
  }
}

bool Ring::push(Kind kind, std::span<const std::uint8_t> record, std::uint32_t t_lo) noexcept {
  const std::size_t k = ring_kind_index(kind);
  const std::size_t n = record.size() / kCellBytes;
  const std::size_t head = head_.load(std::memory_order_relaxed);
  const std::size_t tail = tail_.load(std::memory_order_acquire);
  const std::size_t in_use = used(head, tail);
  const std::size_t room = cells_.size() - 1 - in_use;
  const std::size_t gap_cells = gap_owed_ ? gap::kBytes / kCellBytes : 0;

  if (room < n + gap_cells) {
    dropped_[k].store(dropped_[k].load(std::memory_order_relaxed) + 1, std::memory_order_relaxed);
    ++gap_pending_[k];
    gap_owed_ = true;
    return false;
  }

  std::size_t at = head;
  if (gap_owed_) {
    const auto g = encode_gap(t_lo, gap_pending_);
    write(at, g);
    at = advance(at, gap_cells);
    gap_pending_.fill(0);
    gap_owed_ = false;
    gaps_.store(gaps_.load(std::memory_order_relaxed) + 1, std::memory_order_relaxed);
  }
  write(at, record.first(n * kCellBytes));
  at = advance(at, n);
  head_.store(at, std::memory_order_release);

  pushed_[k].store(pushed_[k].load(std::memory_order_relaxed) + 1, std::memory_order_relaxed);
  const auto held = static_cast<std::uint32_t>(in_use + gap_cells + n);
  if (held > high_water_.load(std::memory_order_relaxed)) {
    high_water_.store(held, std::memory_order_relaxed);
  }
  return true;
}

std::size_t Ring::backlog_cells() const noexcept {
  const std::size_t tail = tail_.load(std::memory_order_relaxed);
  const std::size_t head = head_.load(std::memory_order_acquire);
  return used(head, tail);
}

std::size_t Ring::pop(std::span<std::uint8_t> out) noexcept {
  const std::size_t tail = tail_.load(std::memory_order_relaxed);
  const std::size_t head = head_.load(std::memory_order_acquire);
  if (head == tail) {
    return 0;
  }
  const auto kind = static_cast<Kind>(cells_[tail].bytes[kLeadOffKind]);
  const std::size_t bytes = ring_record_bytes(kind);
  if (bytes == 0 || bytes / kCellBytes > used(head, tail)) {
    corrupt_.store(corrupt_.load(std::memory_order_relaxed) + 1, std::memory_order_relaxed);
    tail_.store(head, std::memory_order_release);
    return 0;
  }
  if (out.size() < bytes) {
    return 0;
  }
  read(tail, out.first(bytes));
  tail_.store(advance(tail, bytes / kCellBytes), std::memory_order_release);
  return bytes;
}

std::size_t Ring::discard_oldest(std::size_t keep_cells) noexcept {
  std::size_t removed = 0;
  for (;;) {
    const std::size_t tail = tail_.load(std::memory_order_relaxed);
    const std::size_t head = head_.load(std::memory_order_acquire);
    const std::size_t in_use = used(head, tail);
    if (in_use <= keep_cells) {
      return removed;
    }
    const auto kind = static_cast<Kind>(cells_[tail].bytes[kLeadOffKind]);
    const std::size_t bytes = ring_record_bytes(kind);
    if (bytes == 0 || bytes / kCellBytes > in_use) {
      corrupt_.store(corrupt_.load(std::memory_order_relaxed) + 1, std::memory_order_relaxed);
      tail_.store(head, std::memory_order_release);
      return removed + in_use;
    }
    tail_.store(advance(tail, bytes / kCellBytes), std::memory_order_release);
    removed += bytes / kCellBytes;
  }
}

RingStats Ring::stats() const noexcept {
  RingStats s;
  for (std::size_t i = 0; i < kRingKinds; ++i) {
    s.pushed[i] = pushed_[i].load(std::memory_order_relaxed);
    s.dropped[i] = dropped_[i].load(std::memory_order_relaxed);
  }
  s.gaps = gaps_.load(std::memory_order_relaxed);
  s.high_water_cells = high_water_.load(std::memory_order_relaxed);
  s.corrupt = corrupt_.load(std::memory_order_relaxed);
  return s;
}

}  // namespace marv::log
