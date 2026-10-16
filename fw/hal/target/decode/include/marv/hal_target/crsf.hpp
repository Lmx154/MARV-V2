#pragma once

// The CRSF frame decoder of the ELRS receiver link (M2 packet P5a), without the Pico SDK so the L08 T1 tests build it on
// the host. A streaming parser: it takes received bytes one at a time and yields the RC channels frame (0x16) as raw
// 11-bit values and the link statistics frame (0x14) as raw fields. Nothing is converted to sticks, microseconds or
// units here; that is the contract of the code that reads these structs. The protocol numbers are in constants.hpp
// (TBS CRSF spec, sections "Frame Construction", "CRC", "0x14 Link Statistics", "0x16 RC Channels Packed Payload").
//
// Frame: sync, length, type, payload, CRC8 (polynomial 0xD5, over type and payload). The sync byte is the serial sync
// byte or the broadcast address; the length counts type, payload and CRC and is valid from 2 to 62. A frame whose type
// is neither of the two is counted and skipped; a payload longer than the known fields is accepted and the extra bytes
// are ignored (spec: "Frame receiver should just ignore extra fields").
//
// Resynchronisation: the bytes since the candidate sync byte are kept (at most one frame). When a candidate fails (a
// length out of range, a CRC mismatch) its first byte is dropped and the rest are scanned again, so a real frame that
// began inside the failed candidate is found. Nothing is fatal and nothing is allocated; the buffer holds a maximum
// frame, so there is no overrun to count: a length above the maximum is a bad length.

#include <array>
#include <cstddef>
#include <cstdint>

#include "marv/prim/constants.hpp"

namespace marv::hal_target {

// The CRC8 of `n` bytes (polynomial kCrsfCrc8Poly, initial value 0, no reflection, no final xor).
[[nodiscard]] std::uint8_t crsf_crc8(const std::uint8_t* data, std::size_t n) noexcept;

// The 16 channels of a 0x16 frame as the 11-bit values on the wire (0 .. 2^11 - 1), channel_01 first.
struct CrsfRcChannels {
  std::array<std::uint16_t, prim::kCrsfRcChannels> raw{};
};

// The 0x14 link statistics fields as on the wire (RSSI as dBm * -1; link quality in percent; SNR in dB; the antenna,
// RF profile and RF power fields are the spec's enumerations).
struct CrsfLinkStats {
  std::uint8_t up_rssi_ant1 = 0;
  std::uint8_t up_rssi_ant2 = 0;
  std::uint8_t up_link_quality = 0;
  std::int8_t up_snr = 0;
  std::uint8_t active_antenna = 0;
  std::uint8_t rf_profile = 0;
  std::uint8_t up_rf_power = 0;
  std::uint8_t down_rssi = 0;
  std::uint8_t down_link_quality = 0;
  std::int8_t down_snr = 0;
};

// Unpacks the kCrsfRcPayloadBytes of a 0x16 payload, least significant bit first.
[[nodiscard]] CrsfRcChannels crsf_unpack_channels(const std::uint8_t* payload) noexcept;

// What a pushed byte completed.
enum class CrsfEvent : std::uint8_t { None, RcChannels, LinkStats, Skipped };

struct CrsfCounters {
  std::uint32_t rc_frames = 0;
  std::uint32_t link_stats_frames = 0;
  std::uint32_t skipped_frames = 0;  // valid frames of any other type
  std::uint32_t bad_crc = 0;
  std::uint32_t bad_length = 0;  // a length outside 2 .. 62, or a payload shorter than the known fields of its type
  std::uint32_t dropped_bytes = 0;  // bytes discarded while searching for a sync byte or after a failed candidate
};

class CrsfDecoder {
 public:
  // Feeds one received byte. Returns the kind of frame it completed (CrsfEvent::None if none). The latest values are
  // in rc() and link(), valid once the matching frame has been seen. A byte can complete more than one frame only after garbage that hid a
  // whole frame; the event returned is the last one.
  [[nodiscard]] CrsfEvent push(std::uint8_t byte) noexcept;

  [[nodiscard]] const CrsfRcChannels& rc() const noexcept { return rc_; }
  [[nodiscard]] const CrsfLinkStats& link() const noexcept { return link_; }
  [[nodiscard]] const CrsfCounters& counters() const noexcept { return counters_; }

 private:
  void drop_first() noexcept;
  [[nodiscard]] CrsfEvent accept(std::size_t length_field) noexcept;

  std::array<std::uint8_t, prim::kCrsfFrameBytesMax> buf_{};
  std::size_t len_ = 0;
  CrsfRcChannels rc_{};
  CrsfLinkStats link_{};
  CrsfCounters counters_{};
};

}  // namespace marv::hal_target
