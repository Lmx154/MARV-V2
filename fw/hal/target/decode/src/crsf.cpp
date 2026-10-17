#include "marv/hal_target/crsf.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "marv/prim/constants.hpp"

namespace marv::hal_target {
namespace {

constexpr unsigned kByteBits = std::numeric_limits<std::uint8_t>::digits;
constexpr std::uint8_t kCrcTopBit = static_cast<std::uint8_t>(1U << (kByteBits - 1U));
constexpr std::uint32_t kChannelMask = (1U << prim::kCrsfRcChannelBits) - 1U;

// Bytes before the type: sync and length. After the length: type, payload and a CRC byte (type and CRC are the two
// bytes the minimum length counts).
constexpr std::size_t kHeaderBytes = 2;
constexpr std::size_t kTypeIndex = kHeaderBytes;
constexpr std::size_t kPayloadIndex = kTypeIndex + 1;

static_assert(prim::kCrsfRcChannels * prim::kCrsfRcChannelBits == prim::kCrsfRcPayloadBytes * kByteBits);
static_assert(prim::kCrsfLengthMax + kHeaderBytes == prim::kCrsfFrameBytesMax);
static_assert(prim::kCrsfLinkStatsPayloadBytes == prim::kCrsfLsDownSnr + 1);

[[nodiscard]] bool is_sync(std::uint8_t b) noexcept { return b == prim::kCrsfSyncSerial || b == prim::kCrsfAddrBroadcast; }

}  // namespace

std::uint8_t crsf_crc8(const std::uint8_t* data, std::size_t n) noexcept {
  std::uint8_t crc = 0;
  for (std::size_t i = 0; i < n; ++i) {
    crc = static_cast<std::uint8_t>(crc ^ data[i]);
    for (unsigned bit = 0; bit < kByteBits; ++bit) {
      crc = static_cast<std::uint8_t>((crc & kCrcTopBit) != 0 ? ((crc << 1U) ^ prim::kCrsfCrc8Poly) : (crc << 1U));
    }
  }
  return crc;
}

CrsfRcChannels crsf_unpack_channels(const std::uint8_t* payload) noexcept {
  CrsfRcChannels out;
  std::uint32_t acc = 0;
  unsigned have = 0;
  std::size_t next = 0;
  for (std::size_t i = 0; i < prim::kCrsfRcPayloadBytes; ++i) {
    acc |= static_cast<std::uint32_t>(payload[i]) << have;
    have += kByteBits;
    while (have >= prim::kCrsfRcChannelBits && next < out.raw.size()) {
      out.raw[next++] = static_cast<std::uint16_t>(acc & kChannelMask);
      acc >>= prim::kCrsfRcChannelBits;
      have -= prim::kCrsfRcChannelBits;
    }
  }
  return out;
}

void CrsfDecoder::drop_first() noexcept {
  std::copy(buf_.begin() + 1, buf_.begin() + static_cast<std::ptrdiff_t>(len_), buf_.begin());
  --len_;
  ++counters_.dropped_bytes;
}

CrsfEvent CrsfDecoder::accept(std::size_t length_field) noexcept {
  const std::size_t payload_bytes = length_field - prim::kCrsfLengthMin;
  const std::uint8_t* payload = buf_.data() + kPayloadIndex;
  switch (buf_[kTypeIndex]) {
    case prim::kCrsfTypeRcChannelsPacked:
      if (payload_bytes < prim::kCrsfRcPayloadBytes) {
        ++counters_.bad_length;
        return CrsfEvent::None;
      }
      rc_ = crsf_unpack_channels(payload);
      ++counters_.rc_frames;
      return CrsfEvent::RcChannels;
    case prim::kCrsfTypeLinkStatistics:
      if (payload_bytes < prim::kCrsfLinkStatsPayloadBytes) {
        ++counters_.bad_length;
        return CrsfEvent::None;
      }
      link_.up_rssi_ant1 = payload[prim::kCrsfLsUpRssiAnt1];
      link_.up_rssi_ant2 = payload[prim::kCrsfLsUpRssiAnt2];
      link_.up_link_quality = payload[prim::kCrsfLsUpLinkQuality];
      link_.up_snr = static_cast<std::int8_t>(payload[prim::kCrsfLsUpSnr]);
      link_.active_antenna = payload[prim::kCrsfLsActiveAntenna];
      link_.rf_profile = payload[prim::kCrsfLsRfProfile];
      link_.up_rf_power = payload[prim::kCrsfLsUpRfPower];
      link_.down_rssi = payload[prim::kCrsfLsDownRssi];
      link_.down_link_quality = payload[prim::kCrsfLsDownLinkQuality];
      link_.down_snr = static_cast<std::int8_t>(payload[prim::kCrsfLsDownSnr]);
      ++counters_.link_stats_frames;
      return CrsfEvent::LinkStats;
    default:
      ++counters_.skipped_frames;
      return CrsfEvent::Skipped;
  }
}

CrsfEvent CrsfDecoder::push(std::uint8_t byte) noexcept {
  buf_[len_++] = byte;  // len_ < buf_.size(): a frame is completed or dropped before the buffer can fill
  CrsfEvent event = CrsfEvent::None;
  while (len_ > 0) {
    if (!is_sync(buf_[0])) {
      drop_first();
      continue;
    }
    if (len_ < kHeaderBytes) {
      break;
    }
    const std::size_t length_field = buf_[1];
    if (length_field < prim::kCrsfLengthMin || length_field > prim::kCrsfLengthMax) {
      ++counters_.bad_length;
      drop_first();
      continue;
    }
    const std::size_t frame_bytes = kHeaderBytes + length_field;
    if (len_ < frame_bytes) {
      break;
    }
    const std::uint8_t crc = crsf_crc8(buf_.data() + kTypeIndex, length_field - 1);
    if (crc != buf_[frame_bytes - 1]) {
      ++counters_.bad_crc;
      drop_first();
      continue;
    }
    event = accept(length_field);
    std::copy(buf_.begin() + static_cast<std::ptrdiff_t>(frame_bytes), buf_.begin() + static_cast<std::ptrdiff_t>(len_), buf_.begin());
    len_ -= frame_bytes;
  }
  return event;
}

}  // namespace marv::hal_target
