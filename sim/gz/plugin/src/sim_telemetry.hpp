#pragma once

// The sim channel of the lockstep plugin (ground-segment unit GX-8, decision 0056): the optional <sim_telemetry_udp_port> and
// <sim_telemetry_period_us> elements make the plugin publish one versioned record per period of SIM time as a UDP datagram to
// 127.0.0.1:<port> (the ground-station helper relays it unparsed to the GCS's SIM panels). It lives in the plugin because it
// carries simulator truth (invariant S-1: truth only on a separate sim channel, never in the firmware, never on the vehicle
// link; gate G3 is about the flight composition, which this file is not part of).
//
// The channel never perturbs the simulation (invariant SC-1): the plugin only reads state it already holds, after the step is
// done and logged; the datagram goes out on a non-blocking socket and a failure to send is counted, never acted on; the only
// clock read is the wall clock, for the real-time factor of the record, and it feeds nothing but the record. Without the two
// elements nothing here runs (the socket is never opened).
//
// Record, version 1. Little-endian, packed (no padding), one datagram. Decoders: gcs/src/lib/sim/record.ts (the GCS, the one
// decoder of the app) and tools/sim/sim_channel.py (tests, evidence). A decoder accepts a record whose magic and version it
// knows and whose length field equals the datagram's and is at least this version's length; a newer record appends fields.
//
//   offset size field
//   0      4    magic "MSIM"
//   4      2    u16 version (1)
//   6      2    u16 length: bytes of the whole record
//   8      4    u32 sequence: 0 for the first record of the run, +1 per record sent; a gap is a lost datagram
//   12     4    u32 flags (bit numbers below)
//   16     8    u64 sim_time_ns: the gz simulation time at this host step's end
//   24     8    u64 tick: the last tick of this host step (the firmware stamp is floor(tick num_us / den) us)
//   32     8    f64 rtf: simulated seconds per wall second since the previous record (0 for the first)
//   40     24   f64[3] pos_ned_m: truth, the body the step was read at (start of the host step)
//   64     24   f64[3] vel_ned_m_s: truth
//   88     32   f64[4] q_wxyz: truth, Hamilton, body FRD -> NED
//   120    24   f64[3] omega_frd_rad_s: truth
//   144    24   f64[3] specific_force_frd_m_s2: truth, rotor thrust / mass along body -z (0, 0, -sum k w^2 / m) with the rotor
//                speeds below; drag and the pad's contact force are not in it (the injected accelerometer sample has them)
//   168    32   f64[4] rotor_speed_rad_s: truth, logical motors 1..4, after the host step's last tick
//   200    12   f32[3] imu_gyro_rad_s: the sample injected into the firmware at the last tick (FRD)
//   212    12   f32[3] imu_accel_m_s2: the same, specific force FRD
//   224    4    u32 imu_flags: marv_imu_meas.flags (MARV_IMU_*)
//   228    8    u16[4] dshot_commanded: the command the firmware (SIL) or the board (HIL) gave at the last tick
//   236    8    u16[4] dshot_applied: the command the plant was stepped with (after the command delay)
//   244    4    u32 touchdowns: touchdowns so far (the opt-in ground)
//   248    4    f32 last_touchdown_down_m_s: the last touchdown's NED down speed at impact (0 before the first)
//   252    ...  HIL block, present always, zero unless flag bit 3 (hil) is set:
//   252    4    u32 arming_state: the board's arming state code (fw/arming ArmState), from its latest status record
//   256    4    u32 status_records: status records received from the board (0: arming_state is not known yet)
//   260    4    u32 receiver_frames: receiver-frame records received (0: no pilot frame yet)
//   264    4    u32 frame_counter: the receiver's own count of RC frames in the latest frame
//   268    4    u32 mode: 0 unknown, 1 acro, 2 angle (the mode the board selects from the latest frame)
//   272    4    u32 deadline_misses: the board's IMU deadline misses
//   276    4    u32 link_faults: the board's link faults (CRC failures, bad headers, sequence and tick gaps, drops, overflows)
//   280    4    u32 reports_used: board DShot reports used so far
//   284    4    u32 lag_events: ticks gz reached after their wall-clock instant
//   288    16   f32[4] sticks: roll, pitch, yaw, throttle of the latest receiver frame
//   304    64   f32[16] channels: the latest receiver frame's channels 1..16
//   368    4    f32 link_quality: the latest receiver frame's
//   372    4    f32 margin_min_us: smallest command margin (needed - arrived) so far, microseconds (0 before the first report)
//   376    4    f32 margin_max_us: largest
//   380    4    f32 lag_us: the schedule shift absorbed so far, microseconds
//   384         (end of version 1)
//
//   flag bit 0 imu_model: the injected IMU sample is the plant model's (else zero or the truth gyro's)
//        bit 1 launch_pad, bit 2 ground, bit 3 hil
//        bit 4 pad_held: the pad held the body at the last tick
//        bit 5 pad_released: the pad has released (the vehicle has lifted off) at least once
//
// Absent from the record, by construction: a late command count (a late command stops a HIL run, so a record never shows one);
// the SIL's arming state, mode and receiver frames (firmware state: they reach the GCS on the vehicle link, not here); the
// board's attitude and estimates (the HIL image has no link).

#include <netinet/in.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include "hil_source.hpp"

namespace marv::gz {

inline constexpr std::uint16_t kSimRecordVersion = 1;
inline constexpr std::size_t kSimRecordBytes = 384;
inline constexpr std::uint32_t kSimFlagImuModel = 1U << 0;
inline constexpr std::uint32_t kSimFlagLaunchPad = 1U << 1;
inline constexpr std::uint32_t kSimFlagGround = 1U << 2;
inline constexpr std::uint32_t kSimFlagHil = 1U << 3;
inline constexpr std::uint32_t kSimFlagPadHeld = 1U << 4;
inline constexpr std::uint32_t kSimFlagPadReleased = 1U << 5;

struct SimSample {
  std::uint64_t sim_time_ns = 0;
  std::uint64_t tick = 0;
  std::uint32_t flags = 0;
  std::array<double, 3> pos_ned_m{};
  std::array<double, 3> vel_ned_m_s{};
  std::array<double, 4> q_wxyz{};
  std::array<double, 3> omega_frd_rad_s{};
  std::array<double, 3> specific_force_frd_m_s2{};
  std::array<double, 4> rotor_speed_rad_s{};
  std::array<float, 3> imu_gyro_rad_s{};
  std::array<float, 3> imu_accel_m_s2{};
  std::uint32_t imu_flags = 0;
  std::array<std::uint16_t, 4> dshot_commanded{};
  std::array<std::uint16_t, 4> dshot_applied{};
  std::uint32_t touchdowns = 0;
  float last_touchdown_down_m_s = 0.0F;
  HilView hil{};
};

class SimTelemetry {
 public:
  SimTelemetry() = default;
  ~SimTelemetry();
  SimTelemetry(const SimTelemetry&) = delete;
  SimTelemetry& operator=(const SimTelemetry&) = delete;

  // Opens the non-blocking socket toward 127.0.0.1:port. Empty on success, else the refusal.
  std::string open(std::uint32_t port, std::uint64_t period_us);
  bool enabled() const { return fd_ >= 0; }
  // True at the first call and then at the first host step whose sim time reaches the next multiple of the period.
  bool due(std::uint64_t sim_time_ns) const { return fd_ >= 0 && sim_time_ns >= next_ns_; }
  void publish(const SimSample& s);
  std::uint64_t sent() const { return seq_; }
  std::uint64_t failures() const { return failures_; }

 private:
  int fd_ = -1;
  sockaddr_in dst_{};
  std::uint64_t period_ns_ = 0;
  std::uint64_t next_ns_ = 0;
  std::uint32_t seq_ = 0;
  std::uint64_t failures_ = 0;
  bool have_prev_ = false;
  std::uint64_t prev_sim_ns_ = 0;
  std::int64_t prev_wall_ns_ = 0;
};

}  // namespace marv::gz
