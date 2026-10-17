// The trivial test codec (ground segment GS-3: "a second, trivial codec (for tests) passes the same conformance suite").
// Frame: 'T', system, component, sequence, variant index, size, the raw bytes of the message struct, CRC-16/CCITT-FALSE over
// everything after 'T'. It is not a wire format: it only has to meet the codec interface and the suite.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <type_traits>
#include <variant>

#include "marv/link/codec.hpp"

namespace marv::link::test {

constexpr std::uint8_t kTrivialMagic = 'T';
constexpr std::size_t kTrivialHeaderBytes = 6;
constexpr std::size_t kTrivialMaxPayload = 255;

inline std::uint16_t crc16(std::span<const std::uint8_t> bytes, std::uint16_t crc = 0xFFFF) {
  for (const std::uint8_t byte : bytes) {
    crc = static_cast<std::uint16_t>(crc ^ (byte << 8));
    for (int bit = 0; bit < 8; ++bit) {
      crc = static_cast<std::uint16_t>((crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1);
    }
  }
  return crc;
}

inline std::size_t encode_trivial(const Envelope& envelope, const Message& message, std::span<std::uint8_t> out) {
  std::array<std::uint8_t, kTrivialHeaderBytes + kTrivialMaxPayload + 2> frame{};
  std::size_t size = 0;
  std::visit(
      [&](const auto& plain) {
        static_assert(std::is_trivially_copyable_v<std::decay_t<decltype(plain)>>);
        size = sizeof(plain);
        std::memcpy(frame.data() + kTrivialHeaderBytes, &plain, sizeof(plain));
      },
      message);
  frame[0] = kTrivialMagic;
  frame[1] = envelope.system_id;
  frame[2] = envelope.component_id;
  frame[3] = envelope.sequence;
  frame[4] = static_cast<std::uint8_t>(message.index());
  frame[5] = static_cast<std::uint8_t>(size);
  const std::uint16_t crc = crc16(std::span(frame).subspan(1, kTrivialHeaderBytes - 1 + size));
  frame[kTrivialHeaderBytes + size] = static_cast<std::uint8_t>(crc & 0xFF);
  frame[kTrivialHeaderBytes + size + 1] = static_cast<std::uint8_t>(crc >> 8);
  const std::size_t length = kTrivialHeaderBytes + size + 2;
  if (length > out.size()) {
    return 0;
  }
  std::memcpy(out.data(), frame.data(), length);
  return length;
}

class TrivialParser {
 public:
  Status push(std::uint8_t byte) {
    if (count_ == 0 && byte != kTrivialMagic) {
      return Status::Pending;
    }
    buffer_[count_++] = byte;
    if (count_ < kTrivialHeaderBytes) {
      return Status::Pending;
    }
    const std::size_t size = buffer_[5];
    if (count_ < kTrivialHeaderBytes + size + 2) {
      return Status::Pending;
    }
    count_ = 0;
    const std::uint16_t want = static_cast<std::uint16_t>(buffer_[kTrivialHeaderBytes + size] |
                                                          (buffer_[kTrivialHeaderBytes + size + 1] << 8));
    if (crc16(std::span<const std::uint8_t>(buffer_).subspan(1, kTrivialHeaderBytes - 1 + size)) != want) {
      return Status::BadCrc;
    }
    frame_.envelope = {buffer_[1], buffer_[2], buffer_[3]};
    return take(buffer_[4], size);
  }

  const Frame& frame() const { return frame_; }

 private:
  template <std::size_t I = 0>
  Status take(std::size_t index, std::size_t size) {
    if constexpr (I == std::variant_size_v<Message>) {
      return Status::Unsupported;
    } else {
      if (index != I) {
        return take<I + 1>(index, size);
      }
      using T = std::variant_alternative_t<I, Message>;
      if (size != sizeof(T)) {
        return Status::BadLength;
      }
      T plain;
      std::memcpy(&plain, buffer_.data() + kTrivialHeaderBytes, sizeof(T));
      frame_.message = plain;
      return Status::Ok;
    }
  }

  std::array<std::uint8_t, kTrivialHeaderBytes + kTrivialMaxPayload + 2> buffer_{};
  std::size_t count_ = 0;
  Frame frame_{};
};

}  // namespace marv::link::test
