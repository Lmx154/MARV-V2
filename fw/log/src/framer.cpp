#include "marv/log/framer.hpp"

#include <algorithm>

namespace marv::log {

void SectorFramer::clear_open() noexcept {
  std::fill(open_sector().begin(), open_sector().end(), std::uint8_t{0});
  fill_ = 0;
  first_ = kNoRecord;
}

void SectorFramer::begin(std::uint64_t session_id) noexcept {
  session_ = session_id;
  seq_ = 0;
  staged_ = 0;
  failed_ = false;
  clear_open();
}

void SectorFramer::finish_sector() noexcept {
  const std::span<std::uint8_t> s = open_sector();
  std::copy(kSectorMagic.begin(), kSectorMagic.end(), s.begin() + kSecOffMagic);
  put<std::uint64_t>(s, kSecOffSession, session_);
  put<std::uint32_t>(s, kSecOffSeq, seq_);
  put<std::uint16_t>(s, kSecOffFirst, first_);
  put<std::uint16_t>(s, kSecOffVersion, kFormatVersion);
  put<std::uint32_t>(s, kSecOffCrc, crc32(s.first(kSecOffCrc)));
  ++seq_;
  ++staged_;
}

bool SectorFramer::append(std::span<const std::uint8_t> record, ChunkSink& sink) noexcept {
  if (failed_) {
    return false;
  }
  bool begun = false;
  std::size_t done = 0;
  while (done < record.size()) {
    if (!begun && first_ == kNoRecord) {
      first_ = static_cast<std::uint16_t>(fill_);
    }
    begun = true;
    const std::size_t take = std::min(record.size() - done, kSectorPayloadBytes - fill_);
    const std::span<std::uint8_t> s = open_sector();
    std::copy_n(record.begin() + static_cast<std::ptrdiff_t>(done), take,
                s.begin() + static_cast<std::ptrdiff_t>(kSectorHeaderBytes + fill_));
    fill_ += take;
    done += take;
    if (fill_ == kSectorPayloadBytes) {
      finish_sector();
      if (staged_ * kSectorBytes >= staging_.size()) {
        const std::size_t sectors = staged_;
        staged_ = 0;
        if (!sink.write_chunk(staging_.first(sectors * kSectorBytes))) {
          seq_ -= static_cast<std::uint32_t>(sectors);  // the refused sectors never reached the card: the file has no hole
          failed_ = true;
          clear_open();
          return false;
        }
      }
      clear_open();
    }
  }
  return true;
}

bool SectorFramer::flush(ChunkSink& sink) noexcept {
  if (failed_) {
    return false;
  }
  if (fill_ > 0) {
    finish_sector();
  }
  bool ok = true;
  if (staged_ > 0) {
    ok = sink.write_chunk(staging_.first(staged_ * kSectorBytes));
    if (!ok) {
      seq_ -= static_cast<std::uint32_t>(staged_);
    }
    staged_ = 0;
  }
  clear_open();
  if (!ok) {
    failed_ = true;
  }
  return ok;
}

void SectorFramer::discard_staged() noexcept {
  seq_ -= static_cast<std::uint32_t>(staged_);
  staged_ = 0;
  failed_ = false;
  clear_open();
}

}  // namespace marv::log
