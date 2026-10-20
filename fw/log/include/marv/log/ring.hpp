// The log ring: a single-producer single-consumer ring of 16-byte cells (format.hpp kCellBytes) between the tick context
// (the writer; the IMU interrupt on the board) and the background loop (the consumer, the logger).
//
// Invariant L-1: the writer never waits. push* is wait-free and bounded: one load of each index, a copy of at most
// manual::kBytes plus gap::kBytes bytes, one release store; it takes no lock, calls no HAL function, and does the same work
// whatever the consumer is doing (running, stopped mid-pop, absent). When the ring has no room the NEWEST whole record is
// dropped, counted per kind, and the writer remembers the drop; before the next record that fits it pushes a Gap record
// (kind Gap, the counts since the last one), so the loss sits where it happened in the stream. A drop never leaves half a
// record in the ring: a record is published by the single store of the head index after all its cells are written.
//
// The ring keeps one cell free (head == tail means empty). The caller supplies the storage (static, on the board).
#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <span>

#include "marv/log/format.hpp"
#include "marv/log/records.hpp"
#include "marv/types/imu_sample.hpp"
#include "marv/types/manual_control.hpp"

namespace marv::log {

struct alignas(kCellBytes) Cell {
  std::array<std::uint8_t, kCellBytes> bytes;
};

struct RingStats {
  std::array<std::uint32_t, kRingKinds> pushed{};   // records published, per ring kind (Gap records are not counted)
  std::array<std::uint32_t, kRingKinds> dropped{};  // records dropped for want of room, per ring kind
  std::uint32_t gaps = 0;                           // Gap records published
  std::uint32_t high_water_cells = 0;               // most cells held at once
  std::uint32_t corrupt = 0;                        // times the consumer found a cell that starts no record (it then empties the ring)
};

class Ring {
 public:
  explicit Ring(std::span<Cell> storage) noexcept : cells_(storage) {}

  Ring(const Ring&) = delete;
  Ring& operator=(const Ring&) = delete;

  // ---- producer (the tick context only) ----
  bool push_imu(const ImuSample& s) noexcept {
    return push(Kind::Imu, encode_imu(s), low32(s.t_us));
  }
  // The compact record when the sample is exactly counts of `sc`, the Imu record when it is not (the stream stays lossless).
  bool push_imu_raw(const ImuSample& s, const ImuRawScales& sc) noexcept {
    if (const auto r = encode_imu_raw(s, sc)) {
      return push(Kind::ImuRaw, *r, low32(s.t_us));
    }
    return push_imu(s);
  }
  bool push_rate_tick(const RateTickRecord& t) noexcept {
    return push(Kind::RateTick, encode_rate_tick(t), low32(t.now_us));
  }
  bool push_manual(const ManualControl& m) noexcept {
    return push(Kind::ManualControl, encode_manual(m), low32(m.t_us));
  }
  // `record` is exactly ring_record_bytes(kind) bytes starting with the kind. False: dropped (counted).
  bool push(Kind kind, std::span<const std::uint8_t> record, std::uint32_t t_lo) noexcept;

  // ---- consumer (the background loop only) ----
  [[nodiscard]] std::size_t capacity_cells() const noexcept { return cells_.size(); }
  [[nodiscard]] std::size_t backlog_cells() const noexcept;
  // Copies the oldest record into `out` (kMaxRingRecordBytes is always enough) and removes it; returns its bytes, 0 if the
  // ring is empty or `out` is too small (the record stays).
  std::size_t pop(std::span<std::uint8_t> out) noexcept;
  // Removes the oldest records until at most `keep_cells` cells remain; returns the cells removed.
  std::size_t discard_oldest(std::size_t keep_cells) noexcept;

  [[nodiscard]] RingStats stats() const noexcept;

 private:
  [[nodiscard]] std::size_t advance(std::size_t index, std::size_t cells) const noexcept {
    const std::size_t next = index + cells;
    return next >= cells_.size() ? next - cells_.size() : next;
  }
  [[nodiscard]] std::size_t used(std::size_t head, std::size_t tail) const noexcept {
    return head >= tail ? head - tail : cells_.size() - tail + head;
  }
  void write(std::size_t index, std::span<const std::uint8_t> record) noexcept;
  void read(std::size_t index, std::span<std::uint8_t> out) const noexcept;

  std::span<Cell> cells_;
  std::atomic<std::size_t> head_{0};  // next cell the producer writes; stored (release) only by the producer
  std::atomic<std::size_t> tail_{0};  // next cell the consumer reads; stored (release) only by the consumer
  std::array<std::atomic<std::uint32_t>, kRingKinds> pushed_{};
  std::array<std::atomic<std::uint32_t>, kRingKinds> dropped_{};
  std::atomic<std::uint32_t> gaps_{0};
  std::atomic<std::uint32_t> high_water_{0};
  std::atomic<std::uint32_t> corrupt_{0};
  // Producer-only: drops since the last Gap record.
  std::array<std::uint32_t, kRingKinds> gap_pending_{};
  bool gap_owed_ = false;
};

}  // namespace marv::log
