// The ground link service (decision 0053; ground-segment unit 6; core section 8 IF-1, IF-5, IF-11). It reads frames from a
// byte transport with the link codec and answers them from the config store (marv/store/store.hpp):
//   - the heartbeat (once per second; the armed flag from the arming state) and, on request, the version (byte-wise parameter
//     encoding) and the identity (the image, config and schema hashes, the config's source, the boot step's verdict and
//     refusal reason, the last activation, the validation report id, whether the staged config differs);
//   - the parameter list and single reads: the STAGED value of each parameter (index = ParamId, count = kParamCount, its link
//     id) followed by its provenance record (active and staged bits, origin, method, sigma, lock, the start of its source);
//     a listing sends one parameter per service step;
//   - a parameter write: staged through store::stage_set; a refusal is answered with the reason as status text, then the
//     unchanged value;
//   - source texts: read in chunks; a text set in chunks is staged with the parameter's staged value and provenance once it is
//     complete; a read or set naming another parameter schema is refused;
//   - the config commands (activate with a report id and the expected config hash, revert to the last good config, factory
//     reset), answered with an acknowledgement; an accepted command reboots only through store::reboot_when_granted.
//   - the reboot command (MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, this autopilot only): refused while armed; disarmed it asks for the
//     maintenance grant (IN_PROGRESS), then ACCEPTED once granted, or DENIED if the vehicle armed first. param1 1 then
//     reboots through store::reboot_when_granted (FAILED with the reason if the board cannot); param1 3 (bootloader) only takes
//     the grant: the host then resets the board into the bootloader through the USB reset interface the grant opens. One grant
//     waits at a time: a reboot command that arrives while one waits is answered IN_PROGRESS and replaces it (its requester and
//     its param1); the final answer goes to that latest requester only, and the earlier one gets nothing more.
//   - the telemetry streams (decision 0053 Q4; ground-segment unit 11): SYS_STATUS, ATTITUDE, ATTITUDE_QUATERNION, RC_CHANNELS,
//     ATTITUDE_TARGET, HIGHRES_IMU, ACTUATOR_OUTPUT_STATUS, MARV_FC_STATUS and MARV_ESTIMATOR, built from the flight composition's
//     telemetry snapshot (marv/types/telemetry.hpp), and MARV_IDENTITY. None is sent until the ground station sets its interval
//     with MAV_CMD_SET_MESSAGE_INTERVAL (param1 the message id, param2 the interval in us; -1 and 0, the default rate, stop it).
//     The granted interval is the longest of the one asked, the stream's cap (its tlm_hz_* parameter, a rate in Hz) and the
//     one at which the stream fits the telemetry budget (the design-budget entry telemetry_byte_rate_max, bytes per second)
//     beside the streams already running, each counted at its longest frame; a stream the budget or a cap of 0 leaves no room
//     for is DENIED with a status text. MAV_CMD_REQUEST_MESSAGE with param1 MESSAGE_INTERVAL and param2 a message id answers
//     the interval in force (-1: stopped, 0: not a stream); with param1 a stream's id it sends that message once. A stream is
//     sent at most once per step, only while the transport has room for its frame beside one parameter's frames, and every
//     stream stops while the transport reports no ground station (a station that connects later sets its streams again).
// Frames are handled in arrival order. Background loop only (the SIL's link service call; the board's background loop from
// ground-segment unit 12); never from the tick. Static state, no allocation.
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "marv/store/sha256.hpp"

namespace marv {
struct Telemetry;
}

namespace marv::link {

// The byte transport: receive copies waiting bytes into `out` and returns how many (0: nothing waits); send writes one frame
// (it may drop it); writable, if set, is how many bytes send takes now without dropping a frame (a listing waits for room for
// one parameter's frames); without it the transport counts as always having room. connected, if set, says whether a ground
// station holds the link now; while it returns false a listing in progress is dropped (a station that connects later asks
// again); without it the transport counts as always connected.
struct Transport {
  std::size_t (*receive)(std::span<std::uint8_t> out) noexcept = nullptr;
  void (*send)(std::span<const std::uint8_t> frame) noexcept = nullptr;
  std::size_t (*writable)() noexcept = nullptr;
  bool (*connected)() noexcept = nullptr;
};

struct ServiceConfig {
  Transport transport;
  store::Digest image_sha256{};        // the running image's SHA-256; all zero = unknown
  bool (*armed)() noexcept = nullptr;  // the arming state; without it the vehicle counts as armed
  // The maintenance grant (composition::request_maintenance, maintenance_granted); without both a reboot command is unsupported.
  void (*request_maintenance)() noexcept = nullptr;
  bool (*maintenance_granted)() noexcept = nullptr;
  // The flight composition's telemetry snapshot (composition::telemetry: false before the first rate tick); without it only
  // MARV_IDENTITY can be streamed.
  bool (*telemetry)(Telemetry& out) noexcept = nullptr;
};

// Starts the service. Call once, after store::boot and params_init (it reads the stream caps and the telemetry budget then; a
// value that is not a finite non-negative number counts as 0, which stops that stream or every stream); a second call, or a
// transport without both functions, is a hal_panic.
void service_init(const ServiceConfig& config) noexcept;

// Serves the link once: every frame waiting on the transport, the heartbeat when due, the next parameter of a listing, the
// telemetry streams that are due, a reboot waiting for the maintenance grant. Returns true once the store's reboot was requested (hal_reboot returned): the
// host stops this run. Does nothing before service_init.
[[nodiscard]] bool service_step() noexcept;

}  // namespace marv::link
