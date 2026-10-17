// The MAVLink v2 codec with the MARV dialect (link/marv.xml). The only code in the repository that includes MAVLink
// headers is src/codec_mavlink.cpp (GS-3, tools/ci/check_gs3.py); this header does not.
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "marv/link/codec.hpp"

namespace marv::link {

// Writes one MAVLink v2 frame (no signature) with the given envelope into `out`. Returns the frame length, or 0 when `out`
// is too short. Trailing zero bytes of the payload are trimmed, as the protocol requires.
std::size_t encode_mavlink(const Envelope& envelope, const Message& message, std::span<std::uint8_t> out) noexcept;

// A byte-stream parser on one MAVLink channel. Its state is the library's per-channel state, so two parsers on one channel
// share it; use one parser per channel. A channel number the library does not have makes every push return BadCrc.
class MavlinkParser {
 public:
  explicit MavlinkParser(std::uint8_t channel = 0) noexcept;

  Status push(std::uint8_t byte) noexcept;
  const Frame& frame() const noexcept { return frame_; }
  void reset() noexcept;

 private:
  std::uint8_t channel_;
  Frame frame_{};
};

}  // namespace marv::link
