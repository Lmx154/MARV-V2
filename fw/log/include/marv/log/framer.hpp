// The sector framer: puts records into 512-byte self-validating sectors (format.hpp) in a staging buffer and hands every
// full staging buffer ("chunk", a whole number of sectors) to a sink. Records may span sectors. No allocation, no HAL.
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "marv/log/format.hpp"

namespace marv::log {

class ChunkSink {
 public:
  ChunkSink() = default;
  ChunkSink(const ChunkSink&) = delete;
  ChunkSink& operator=(const ChunkSink&) = delete;
  virtual ~ChunkSink() = default;
  // A whole number of sectors. False: the chunk was not stored.
  [[nodiscard]] virtual bool write_chunk(std::span<const std::uint8_t> chunk) noexcept = 0;
};

class SectorFramer {
 public:
  // `staging` is a whole, non-zero number of sectors.
  explicit SectorFramer(std::span<std::uint8_t> staging) noexcept : staging_(staging) {}

  // A new file: sector sequence 0, empty staging.
  void begin(std::uint64_t session_id) noexcept;

  // Appends one record. False when a chunk the framer had to hand over was refused (the framer is then failed: later calls do
  // nothing and return false until begin).
  [[nodiscard]] bool append(std::span<const std::uint8_t> record, ChunkSink& sink) noexcept;

  // Pads the open sector with zero bytes and hands over every staged sector. Nothing staged: nothing handed over.
  [[nodiscard]] bool flush(ChunkSink& sink) noexcept;

  // Drops the staged, unhanded sectors and the open sector, and takes the sector sequence back to the first of them (the
  // file then has no hole). The framer is not failed afterwards.
  void discard_staged() noexcept;

  [[nodiscard]] bool failed() const noexcept { return failed_; }
  // Sectors completed since begin that reached the sink or are staged (a refused chunk's sectors are taken back).
  [[nodiscard]] std::uint32_t next_sequence() const noexcept { return seq_; }
  [[nodiscard]] std::size_t staging_bytes() const noexcept { return staging_.size(); }
  // Sectors currently staged and complete (not including the open one).
  [[nodiscard]] std::size_t staged_sectors() const noexcept { return staged_; }

 private:
  void finish_sector() noexcept;
  [[nodiscard]] std::span<std::uint8_t> open_sector() noexcept {
    return staging_.subspan(staged_ * kSectorBytes, kSectorBytes);
  }
  void clear_open() noexcept;

  std::span<std::uint8_t> staging_;
  std::uint64_t session_ = 0;
  std::uint32_t seq_ = 0;
  std::size_t staged_ = 0;  // complete sectors waiting in staging_
  std::size_t fill_ = 0;    // payload bytes used in the open sector
  std::uint16_t first_ = kNoRecord;
  bool failed_ = false;
};

}  // namespace marv::log
