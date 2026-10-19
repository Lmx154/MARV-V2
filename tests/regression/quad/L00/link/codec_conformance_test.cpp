// The link codec conformance suite (ground segment GS-3; core section 8 rows IF-1, IF-5, IF-11; decision 0053).
//
// Every codec (the MAVLink codec and the trivial test codec) is held to the same behaviour, TYPED_TEST below:
//   - every sample message survives encode then decode, with its envelope;
//   - frames in a byte stream with garbage between them all arrive;
//   - a frame with any one byte changed never decodes, a frame missing its last byte is Pending, a frame with a byte too
//     many is BadLength.
// The MAVLink codec alone is held to the protocol: its framing against hand-built frames (header, trimmed zero tail, a
// payload longer than its message, an id not in the dialect, an id the build does not carry), and to the other language:
// vectors/cpp_frames.txt is what this codec writes, vectors/ts_frames.txt is what gcs/src/lib/link writes and must decode
// here to the same messages.
#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

#include "marv/link/codec_mavlink.hpp"
#include "samples.hpp"
#include "trivial_codec.hpp"
#include "vectors.hpp"

namespace marv::link::test {
namespace {

constexpr std::size_t kBufferBytes = 512;

struct MavlinkCodec {
  static std::size_t encode(const Envelope& e, const Message& m, std::span<std::uint8_t> out) {
    return encode_mavlink(e, m, out);
  }
  static MavlinkParser parser() { return MavlinkParser(0); }
};

struct TrivialCodec {
  static std::size_t encode(const Envelope& e, const Message& m, std::span<std::uint8_t> out) {
    return encode_trivial(e, m, out);
  }
  static TrivialParser parser() { return TrivialParser(); }
};

template <class Codec>
std::vector<std::uint8_t> encoded(const Envelope& e, const Message& m) {
  std::array<std::uint8_t, kBufferBytes> buffer{};
  const std::size_t length = Codec::encode(e, m, buffer);
  return {buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(length)};
}

template <class Codec>
Status decoded(const std::vector<std::uint8_t>& bytes, Frame& out) {
  auto parser = Codec::parser();
  return decode_frame(parser, bytes, out);
}

template <class Codec>
class CodecConformance : public testing::Test {};

using Codecs = testing::Types<MavlinkCodec, TrivialCodec>;
TYPED_TEST_SUITE(CodecConformance, Codecs);

TYPED_TEST(CodecConformance, EverySampleRoundTripsWithItsEnvelope) {
  const auto all = samples();
  for (std::size_t i = 0; i < all.size(); ++i) {
    const Envelope envelope = envelope_for(i);
    const auto bytes = encoded<TypeParam>(envelope, all[i].message);
    ASSERT_FALSE(bytes.empty()) << all[i].name;
    Frame frame;
    ASSERT_EQ(decoded<TypeParam>(bytes, frame), Status::Ok) << all[i].name;
    EXPECT_EQ(frame.envelope, envelope) << all[i].name;
    EXPECT_EQ(frame.message, all[i].message) << all[i].name;
  }
}

TYPED_TEST(CodecConformance, ATooShortOutputWritesNothing) {
  std::array<std::uint8_t, 4> tiny{};
  EXPECT_EQ(TypeParam::encode(envelope_for(0), samples().front().message, tiny), 0U);
}

TYPED_TEST(CodecConformance, FramesInAStreamWithGarbageBetweenThemAllArrive) {
  const auto all = samples();
  std::vector<std::uint8_t> stream;
  for (std::size_t i = 0; i < all.size(); ++i) {
    const auto bytes = encoded<TypeParam>(envelope_for(i), all[i].message);
    stream.insert(stream.end(), bytes.begin(), bytes.end());
    stream.push_back(0x00);
    stream.push_back(0x55);
  }
  auto parser = TypeParam::parser();
  std::size_t arrived = 0;
  for (const std::uint8_t byte : stream) {
    if (parser.push(byte) == Status::Ok) {
      ASSERT_LT(arrived, all.size());
      EXPECT_EQ(parser.frame().envelope, envelope_for(arrived)) << all[arrived].name;
      EXPECT_EQ(parser.frame().message, all[arrived].message) << all[arrived].name;
      ++arrived;
    }
  }
  EXPECT_EQ(arrived, all.size());
}

TYPED_TEST(CodecConformance, AFrameWithAnyOneByteChangedDoesNotDecode) {
  const auto all = samples();
  for (std::size_t i = 0; i < all.size(); ++i) {
    const auto good = encoded<TypeParam>(envelope_for(i), all[i].message);
    for (std::size_t at = 0; at < good.size(); ++at) {
      auto bad = good;
      bad[at] = static_cast<std::uint8_t>(bad[at] ^ 0x5A);
      Frame frame;
      EXPECT_NE(decoded<TypeParam>(bad, frame), Status::Ok) << all[i].name << " byte " << at;
    }
  }
}

TYPED_TEST(CodecConformance, ATruncatedFrameIsPendingAndAnExtendedOneIsBadLength) {
  const auto all = samples();
  for (std::size_t i = 0; i < all.size(); ++i) {
    const auto good = encoded<TypeParam>(envelope_for(i), all[i].message);
    Frame frame;
    auto shorter = good;
    shorter.pop_back();
    EXPECT_EQ(decoded<TypeParam>(shorter, frame), Status::Pending) << all[i].name;
    auto longer = good;
    longer.push_back(0x00);
    EXPECT_EQ(decoded<TypeParam>(longer, frame), Status::BadLength) << all[i].name;
  }
}

constexpr std::uint8_t kStx = 0xFD;
constexpr std::size_t kHeaderBytes = 10;
constexpr std::uint8_t kHeartbeatCrcExtra = 50;
constexpr std::size_t kHeartbeatBytes = 9;
constexpr std::uint8_t kSystemTimeCrcExtra = 137;  // SYSTEM_TIME, a common.xml message outside link/scope.txt
constexpr std::uint32_t kSystemTimeId = 2;
constexpr std::size_t kSystemTimeBytes = 12;
constexpr std::uint32_t kAbsentId = 14999;

std::uint16_t x25(std::span<const std::uint8_t> bytes, std::uint16_t crc = 0xFFFF) {
  for (std::uint8_t byte : bytes) {
    byte = static_cast<std::uint8_t>(byte ^ (crc & 0xFF));
    byte = static_cast<std::uint8_t>(byte ^ (byte << 4));
    crc = static_cast<std::uint16_t>((crc >> 8) ^ (byte << 8) ^ (byte << 3) ^ (byte >> 4));
  }
  return crc;
}

std::vector<std::uint8_t> hand_frame(std::uint32_t id, std::size_t payload_length, std::uint8_t crc_extra) {
  std::vector<std::uint8_t> f = {kStx, static_cast<std::uint8_t>(payload_length), 0, 0, 7, 1, 2,
                                 static_cast<std::uint8_t>(id), static_cast<std::uint8_t>(id >> 8),
                                 static_cast<std::uint8_t>(id >> 16)};
  f.insert(f.end(), payload_length, 0x01);
  std::uint16_t crc = x25(std::span(f).subspan(1));
  crc = x25(std::span<const std::uint8_t>(&crc_extra, 1), crc);
  f.push_back(static_cast<std::uint8_t>(crc & 0xFF));
  f.push_back(static_cast<std::uint8_t>(crc >> 8));
  return f;
}

TEST(MavlinkFraming, TheHeaderCarriesTheEnvelopeAndTheMessageId) {
  const Envelope envelope{7, 190, 33};
  const auto bytes = encoded<MavlinkCodec>(envelope, MarvIdentity{});
  ASSERT_GT(bytes.size(), kHeaderBytes);
  EXPECT_EQ(bytes[0], kStx);
  EXPECT_EQ(bytes[2], 0);  // incompatibility flags: unsigned
  EXPECT_EQ(bytes[4], envelope.sequence);
  EXPECT_EQ(bytes[5], envelope.system_id);
  EXPECT_EQ(bytes[6], envelope.component_id);
  const std::uint32_t id = bytes[7] | (bytes[8] << 8) | (bytes[9] << 16);
  EXPECT_EQ(id, kMarvIdentityId);
}

TEST(MavlinkFraming, TheZeroTailOfThePayloadIsTrimmedToOneByte) {
  const auto bytes = encoded<MavlinkCodec>({1, 1, 0}, ParamRequestList{});
  EXPECT_EQ(bytes.size(), kHeaderBytes + 1 + 2);
  EXPECT_EQ(bytes[1], 1);
}

TEST(MavlinkFraming, AFrameBuiltByHandDecodesAndOneByteMoreThanTheMessageHasIsBadLength) {
  Frame frame;
  const auto exact = hand_frame(kHeartbeatId, kHeartbeatBytes, kHeartbeatCrcExtra);
  auto parser = MavlinkParser(0);
  ASSERT_EQ(decode_frame(parser, exact, frame), Status::Ok);
  EXPECT_EQ(frame.envelope, (Envelope{1, 2, 7}));
  EXPECT_EQ(std::get<Heartbeat>(frame.message).type, 1);
  const auto longer = hand_frame(kHeartbeatId, kHeartbeatBytes + 1, kHeartbeatCrcExtra);
  auto parser2 = MavlinkParser(0);
  EXPECT_EQ(decode_frame(parser2, longer, frame), Status::BadLength);
}

TEST(MavlinkFraming, AnIdNotInTheDialectIsBadCrc) {
  const auto bytes = hand_frame(kAbsentId, 4, 0);
  Frame frame;
  auto parser = MavlinkParser(0);
  EXPECT_EQ(decode_frame(parser, bytes, frame), Status::BadCrc);
}

TEST(MavlinkFraming, AValidFrameOfAMessageTheBuildDoesNotCarryIsUnsupported) {
  const auto bytes = hand_frame(kSystemTimeId, kSystemTimeBytes, kSystemTimeCrcExtra);
  Frame frame;
  auto parser = MavlinkParser(0);
  EXPECT_EQ(decode_frame(parser, bytes, frame), Status::Unsupported);
}

TEST(MavlinkFraming, ACorrectCrcWithTheWrongCrcExtraIsRejected) {
  const auto bytes = hand_frame(kHeartbeatId, kHeartbeatBytes, kHeartbeatCrcExtra + 1);
  Frame frame;
  auto parser = MavlinkParser(0);
  EXPECT_EQ(decode_frame(parser, bytes, frame), Status::BadCrc);
}

TEST(MavlinkFraming, AChannelTheLibraryDoesNotHaveNeverDecodes) {
  auto parser = MavlinkParser(255);
  EXPECT_EQ(parser.push(kStx), Status::BadCrc);
}

// Byte-wise parameter encoding (decision 0053 Q1): an INT32 parameter travels as its 4 bytes in the float field. The patterns
// that are signalling NaNs as binary32 (exponent all ones, quiet bit clear, mantissa not zero), of either sign, and a quiet NaN
// and an infinity beside them, cross the codec unchanged: on the wire at param_value's payload offset (0 in both messages: the
// only 4-byte field, sorted first) and in the decoded message.
TEST(MavlinkFraming, IntegerBitPatternsThatAreSignallingNaNsAsFloatsRoundTripBitExactly) {
  constexpr std::array<std::uint32_t, 6> kPatterns{0x7F800001U, 0x7FBFFFFFU, 0xFF800001U,
                                                   0xFFBFFFFFU, 0x7FC00000U, 0x7F800000U};
  constexpr std::uint8_t kInt32 = 6;
  for (const std::uint32_t bits : kPatterns) {
    const std::array<Message, 2> messages{ParamValue{text<16>("rc_channel_arm"), bits, kInt32, 137, 3},
                                          ParamSet{1, 1, text<16>("rc_channel_arm"), bits, kInt32}};
    for (const Message& m : messages) {
      const auto bytes = encoded<MavlinkCodec>({1, 1, 0}, m);
      ASSERT_GT(bytes.size(), kHeaderBytes + sizeof(bits));
      std::uint32_t on_wire = 0;
      for (std::size_t i = 0; i < sizeof(bits); ++i) {
        on_wire |= static_cast<std::uint32_t>(bytes[kHeaderBytes + i]) << (8 * i);
      }
      EXPECT_EQ(on_wire, bits) << std::hex << bits;
      Frame frame;
      auto parser = MavlinkParser(0);
      ASSERT_EQ(decode_frame(parser, bytes, frame), Status::Ok) << std::hex << bits;
      EXPECT_EQ(frame.message, m) << std::hex << bits;
    }
  }
}

#ifndef MARV_LINK_VECTORS_DIR
#error "MARV_LINK_VECTORS_DIR names the directory of the vector files"
#endif

TEST(MavlinkVectors, TheCommittedCppFramesAreWhatTheCodecWrites) {
  const auto lines = read_vectors(std::string(MARV_LINK_VECTORS_DIR) + "/cpp_frames.txt");
  const auto all = samples();
  ASSERT_EQ(lines.size(), all.size());
  for (std::size_t i = 0; i < all.size(); ++i) {
    EXPECT_EQ(lines[i].name, all[i].name);
    EXPECT_EQ(lines[i].envelope, envelope_for(i));
    EXPECT_EQ(lines[i].frame, encoded<MavlinkCodec>(envelope_for(i), all[i].message)) << all[i].name;
  }
}

TEST(MavlinkVectors, EveryFrameTheTypeScriptCodecWroteDecodesToTheSampleAndEncodesToTheSameBytes) {
  const auto lines = read_vectors(std::string(MARV_LINK_VECTORS_DIR) + "/ts_frames.txt");
  const auto all = samples();
  ASSERT_EQ(lines.size(), all.size());
  for (std::size_t i = 0; i < all.size(); ++i) {
    ASSERT_EQ(lines[i].name, all[i].name);
    Frame frame;
    auto parser = MavlinkParser(0);
    ASSERT_EQ(decode_frame(parser, lines[i].frame, frame), Status::Ok) << lines[i].name;
    EXPECT_EQ(frame.envelope, lines[i].envelope) << lines[i].name;
    EXPECT_EQ(frame.message, all[i].message) << lines[i].name;
    EXPECT_EQ(encoded<MavlinkCodec>(lines[i].envelope, all[i].message), lines[i].frame) << lines[i].name;
  }
}

}  // namespace
}  // namespace marv::link::test
