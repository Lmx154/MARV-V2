// The pilot input of the lockstep plugin (ground-segment unit GX-9, the SIL pilot bridge; decision 0056).
//
// The optional <pilot_udp_port> (127.0.0.1) makes the plugin the receiver of the pilot's RC frames: the ground-station helper reads
// the RadioMaster as a USB joystick (tools/gs/pilot.py) and sends each frame as one datagram to that port. The datagram is the
// raw 16-channel 11-bit CRSF frame, not a stick value, and the plugin hands it to marv_sil_stage_rc_frame, which decodes it with
// the function the board's receiver calls and the parameter table's rc_channel_*, so that a channel remap acts alike in SIL and
// on the board (invariant R-2).
//
// Datagram, little endian, kPilotDatagramBytes long:
//   0   4  magic "MPLT"
//   4   2  version, kPilotVersion
//   6   2  length, the datagram's size in bytes
//   8  32  the 16 channels, u16 each, channel_01 first, each at most 2^11 - 1
// A datagram of another size, magic, version or length, or a channel above 2^11 - 1, is dropped and counted: the socket is on
// the loopback interface, but any local process can send to it, and a bad datagram must not stop a flight.
//
// Staging. Before each tick the plugin drains the socket and stages the newest valid frame, if any, stamped with that tick's SIL
// stamp (so its age is 0 at the tick). With no new frame nothing is staged: the SIL keeps the last one, whose stamp ages, and the
// link-loss failsafe acts as it does on the board when the receiver stops (a pilot who closes the joystick, or a helper that
// stops, is a receiver that stops). Before the first frame no frame is staged: the link is down (arming state NoLink).
//
// The input is the wall clock's: the helper sends on its own period and the world must be real time (the plugin refuses the port
// in a world with real_time_factor 0, the test mode). The input is therefore not deterministic, and the plugin adds nothing of
// it to the lockstep log.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace marv::gz {

inline constexpr std::size_t kPilotChannels = 16;                            // CRSF: 16 channels (prim kCrsfRcChannels, asserted in pilot_input.cpp)
inline constexpr std::uint16_t kPilotRawMax = (1U << 11U) - 1U;              // CRSF: 11-bit channel field (prim kCrsfRcChannelBits)
inline constexpr std::uint16_t kPilotVersion = 1;
inline constexpr std::size_t kPilotHeaderBytes = 8;
inline constexpr std::size_t kPilotDatagramBytes = kPilotHeaderBytes + (kPilotChannels * sizeof(std::uint16_t));

struct PilotFrame {
  std::array<std::uint16_t, kPilotChannels> raw{};
};

// The datagram's channels, or nothing if it is not a version-1 datagram of the right size with every channel in range.
std::optional<PilotFrame> decode_pilot_datagram(const std::uint8_t* data, std::size_t size);

class PilotInput {
 public:
  PilotInput() = default;
  ~PilotInput();
  PilotInput(const PilotInput&) = delete;
  PilotInput& operator=(const PilotInput&) = delete;

  // Binds the non-blocking socket to 127.0.0.1:port. Empty on success, else the refusal.
  std::string open(std::uint32_t port);
  bool enabled() const { return fd_ >= 0; }
  // Drains the socket; returns the newest valid frame, if any.
  std::optional<PilotFrame> poll();
  std::uint64_t received() const { return received_; }
  std::uint64_t dropped() const { return dropped_; }

 private:
  int fd_ = -1;
  std::uint64_t received_ = 0;
  std::uint64_t dropped_ = 0;
};

}  // namespace marv::gz
