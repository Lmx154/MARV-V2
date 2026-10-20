#pragma once

// The lockstep plugin's HIL command source (HIL stage S4, decision 0046 items 2-4): the FC3v2 board running the
// MARV_FC3V2_HIL_RECEIVER image is the flight code. Selected by the plugin's <hil> element (lockstep.cpp); the bench only,
// never CI. The board's record formats are fw/hal/target/include/marv/hal_target/hil_format.hpp, included here, so the two
// sides share one definition.
//
// Link. open() opens the board's USB CDC port raw (DTR asserted, as tools/target/hil_link.py open_port), reads the hello,
// the receiver-boot record and the first status record, and refuses a board that is not a fresh receiver image of this
// table: parameter schema hash (the plugin's SIL library's), values CRC (<values_crc>), tick period (the world's), batch
// limit >= F, a receiver-boot record after the hello (the S4 image, decision 0046 item 4), and a first status that has run no
// tick and counted no deadline miss (a board reset since its last stream).
//
// Pacing (the wall clock; the one place of the plugin that reads it). The board's receiver and radio live on real time, so
// tick j happens at t0 + stamp(j) + lag on CLOCK_MONOTONIC (t0 = the first call, stamp(j) = j num / den us exactly, in ns):
// dshot() first waits for that instant, then queues tick j's IMU sample (the plant model's bytes, marv_imu_meas, with the SIL's
// stamp floor(j num / den) us), and sends the sample record when it holds F samples (the record goes out when its last
// sample is due, as hil_fc.py). Every wait polls the uplink. lag is the sum of the latenesses of the ticks gz reached after
// their instant: such a tick shifts the schedule instead of the run catching up, so samples never go out faster than the
// tick period (a catch-up would shorten the time a record has to come back below N - F + 1 ticks). The lag is counted and
// reported; the board sees it as one longer gap between two records (an IMU deadline miss if it reaches D_HIL).
//
// Delay (decision 0046 item 2). The source returns, at tick j, the command the board issued at tick j - N: the board's
// latch, i.e. its DShot report of rate tick k d (d = the hello's rate_loop_divisor, reports at ticks 0, d, 2 d, ... of a
// fresh board) with k = floor((j - N) / d); for j < N the stop command, all four DShot 0. This is the SIL's delay line in
// HIL: the plugin passes N to this source and 0 to the adapter, so the command is delayed once and the log's TICK dshot is
// the command the plant was stepped with. The command of tick j - N must already be here when tick j is processed: one that
// is not is the late-command fault. The source then waits at most <late_wait_us> to state how late it was, and fails
// (failure() says which tick, which report, and by how much); the plugin refuses, which stops the run. No command is ever
// reused or guessed. A report out of the schedule, a hello mid-run (the board reset) or a stalled write fail the same way.
//
// Records. Every uplink byte is written as received to <uplink_path> (decoded by tools/target/hil_fc.py parse_uplink): the
// hello, the board's DShot reports with their board times, status and receiver records (the pilot's sticks with their
// board stamps, for the SIL replay of stage S5). <frames_path> gets one CSV row per sample record (first tick, due and send
// times) and <reports_path> one per DShot report used (its tick, arrival time and the time it was needed), host
// nanoseconds since t0. Once per second of tick time a status line goes to stderr (the live view of an S4 pilot).

#include <array>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <string>
#include <vector>

#include "marv/gz/adapter.hpp"
#include "marv_plant.h"
#include "marv_sil.h"

namespace marv::gz {

struct HilConfig {
  std::string device;                 // the board's USB CDC port
  std::uint32_t samples_per_record = 0;  // F
  std::uint32_t delay_ticks = 0;      // N
  std::uint32_t num_us = 0;           // the world's tick period num_us / den us
  std::uint32_t den = 0;
  std::uint64_t schema_hash = 0;      // kParamSchemaHash of the plugin's SIL library
  std::uint32_t values_crc = 0;       // the expected HelloField::ParamValuesCrc
  std::uint64_t hello_wait_us = 0;    // open(): how long to wait for the hello, receiver-boot and first status records
  std::uint64_t late_wait_us = 0;     // a late command: how long to wait to state how late before failing
  std::uint64_t drain_us = 0;         // finish(): how long to keep reading the uplink after the last record
  std::string uplink_path;
  std::string frames_path;
  std::string reports_path;
};

// What the board has reported so far, for the sim channel (sim_telemetry.hpp): a read-only copy of the state the status line
// prints. Counts are saturated at the u32 range of the record.
struct HilView {
  std::uint32_t arming_state = 0;     // the board's ArmState code of its latest status record
  std::uint32_t status_records = 0;   // status records received (0: arming_state is not known yet)
  std::uint32_t receiver_frames = 0;  // receiver-frame records received
  std::uint32_t frame_counter = 0;    // the receiver's own RC frame count in the latest frame
  std::uint32_t mode = 0;             // 0 unknown, 1 acro, 2 angle
  std::uint32_t deadline_misses = 0;
  std::uint32_t link_faults = 0;
  std::uint32_t reports_used = 0;
  std::uint32_t lag_events = 0;
  std::array<float, 4> sticks{};      // roll, pitch, yaw, throttle of the latest receiver frame
  std::array<float, 16> channels{};   // channels 1..16 of the latest receiver frame
  float link_quality = 0.0F;
  float margin_min_us = 0.0F;
  float margin_max_us = 0.0F;
  float lag_us = 0.0F;
};

class HilCommandSource final : public CommandSource {
 public:
  HilCommandSource() = default;
  ~HilCommandSource() override;
  HilCommandSource(const HilCommandSource&) = delete;
  HilCommandSource& operator=(const HilCommandSource&) = delete;

  // Opens the port and the files and checks the board (above). Empty on success, else the refusal.
  std::string open(const HilConfig& cfg);
  // HIL needs the tick's IMU sample: this form fails.
  bool dshot(std::uint64_t tick, Dshot& out) override;
  bool dshot(std::uint64_t tick, const marv_imu_meas* imu, Dshot& out) override;
  std::uint64_t last_stamp_us() const { return stamp_us_; }
  const std::string& failure() const { return failure_; }
  // The body of the current host step, for the status line only.
  void set_body(const marv_plant_body& body) { body_ = body; }
  // With the plugin's <ground>: the touchdowns so far and the last one's NED down speed at impact, for the status line only.
  void set_touchdowns(std::uint64_t count, double last_down_m_s) {
    show_touchdowns_ = true;
    touchdowns_ = count;
    last_touchdown_m_s_ = last_down_m_s;
  }
  // The board's state so far (a copy; reads nothing from the port).
  HilView view() const;
  // Keeps reading the uplink for drain_us after the last record (the board's status after the stream ends), prints the
  // summary line and closes the port and the files. Idempotent.
  void finish();

 private:
  struct Report {
    std::uint64_t tick = 0;
    Dshot dshot{};
    std::int64_t arrival_ns = 0;  // host, since t0 (negative before the first tick)
  };

  std::int64_t now_ns() const;
  bool fail(const std::string& why);
  bool service();
  void parse();
  bool send_frame(std::uint64_t first_tick);
  void status_line(std::uint64_t tick);
  std::uint64_t due_ns(std::uint64_t tick) const;

  HilConfig cfg_;
  int fd_ = -1;
  std::FILE* uplink_ = nullptr;
  std::FILE* frames_ = nullptr;
  std::FILE* reports_ = nullptr;
  std::string failure_;
  std::int64_t t0_abs_ns_ = 0;  // CLOCK_MONOTONIC of t0; 0 before the first tick
  bool started_ = false;
  std::uint64_t stamp_us_ = 0;
  marv_plant_body body_{};
  bool show_touchdowns_ = false;
  std::uint64_t touchdowns_ = 0;
  double last_touchdown_m_s_ = 0.0;

  // Downlink.
  std::uint32_t down_seq_ = 0;
  std::vector<std::uint8_t> frame_;  // the sample record being filled
  std::uint32_t frame_count_ = 0;
  std::uint64_t frames_sent_ = 0;
  std::int64_t max_send_late_ns_ = 0;  // a record's send time after its last sample's (lag-shifted) instant
  std::int64_t lag_ns_ = 0;            // the schedule shift absorbed so far (pacing above)
  std::int64_t max_lag_step_ns_ = 0;
  std::uint64_t lag_events_ = 0;

  // Uplink.
  std::vector<std::uint8_t> read_buf_;
  std::vector<std::uint8_t> rx_;
  bool have_up_seq_ = false;
  std::uint32_t up_seq_next_ = 0;
  std::uint64_t up_seq_gaps_ = 0;
  std::uint64_t up_crc_failures_ = 0;
  std::uint64_t up_unframed_ = 0;
  bool hello_ = false;
  bool receiver_boot_ = false;
  std::uint64_t status_records_ = 0;
  std::vector<std::uint32_t> hello_fields_;
  std::vector<std::uint32_t> status_;
  std::vector<std::uint32_t> receiver_status_;
  std::uint32_t rate_divisor_ = 0;
  std::uint64_t reports_received_ = 0;  // DShot reports of the schedule received so far (report k is tick k d)
  std::deque<Report> pending_;          // received, not yet passed by the plant, oldest first
  std::uint64_t pending_base_ = 0;      // the report index of pending_.front()
  std::uint64_t receiver_frames_ = 0;   // receiver-frame records received
  std::uint32_t rx_frames_ = 0;         // and the last one's fields, for the status line
  float rx_throttle_ = 0.0F;
  float rx_lq_ = 0.0F;
  std::array<float, 4> rx_sticks_{};    // roll, pitch, yaw, throttle of the last frame (sim channel only)
  std::array<float, 16> rx_channels_{};  // its channels 1..16 (sim channel only)
  bool rx_angle_ = false;               // the last frame selects angle mode (status line only)
  std::size_t angle_channel_ = 0;       // quad_fc's rc_channel_angle (0: none), and its angle_switch_threshold (open())
  float angle_threshold_ = 0.0F;
  bool have_angle_threshold_ = false;

  // The command the plant is stepped with, and the margin statistics.
  Dshot applied_{};
  bool have_applied_ = false;
  std::uint64_t applied_k_ = 0;
  std::uint64_t reports_used_ = 0;
  std::int64_t min_margin_ns_ = 0;
  std::int64_t max_margin_ns_ = 0;
  std::uint64_t next_status_tick_ = 0;
  bool finished_ = false;
};

}  // namespace marv::gz
