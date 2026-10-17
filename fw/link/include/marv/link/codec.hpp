// The link codec interface (core section 8; ground segment GS-3). Everything above the codec sees these types and no
// MAVLink header: the messages (marv/link/messages.hpp, generated from the dialect by tools/link/gen_cpp_structs.py),
// the envelope and the status a parser returns. A codec is an `encode` function and a parser with `push` and `frame`;
// the conformance suite (tests/regression/quad/L00/link) holds every codec to the same behaviour.
#pragma once

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>

#include "marv/link/messages.hpp"

namespace marv::link {

struct Envelope {
  std::uint8_t system_id{};
  std::uint8_t component_id{};
  std::uint8_t sequence{};
  bool operator==(const Envelope&) const = default;
};

struct Frame {
  Envelope envelope{};
  Message message{};
  bool operator==(const Frame&) const = default;
};

enum class Status : std::uint8_t {
  Ok,           // a frame was decoded; Parser::frame() holds it
  Pending,      // more bytes are needed
  BadCrc,       // the checksum is wrong, or the message id is not in the dialect
  BadLength,    // the payload is longer than the message allows, or bytes follow a complete frame
  Unsupported,  // a valid frame of a message this build does not carry
};

template <class P>
concept FrameParser = requires(P parser, std::uint8_t byte) {
  { parser.push(byte) } -> std::same_as<Status>;
  { parser.frame() } -> std::same_as<const Frame&>;
};

// Decodes exactly one frame from `bytes` with a fresh parser. Ok only when the last byte completes a frame and no byte
// follows it; fewer bytes than the frame needs is Pending; more is BadLength; the first parser error is returned as is.
template <FrameParser P>
Status decode_frame(P& parser, std::span<const std::uint8_t> bytes, Frame& out) {
  Status status = Status::Pending;
  for (std::size_t i = 0; i < bytes.size(); ++i) {
    if (status != Status::Pending) {
      return Status::BadLength;
    }
    status = parser.push(bytes[i]);
    if (status != Status::Pending && status != Status::Ok) {
      return status;
    }
  }
  if (status == Status::Ok) {
    out = parser.frame();
  }
  return status;
}

}  // namespace marv::link
