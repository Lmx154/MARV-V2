#pragma once

// The per-host-step loop of the Gazebo adapter (decision 0003 items 5, 6 and 8). Host-free: no Gazebo header. The
// Gazebo plugin is a thin shell over this.
//
// Host step H = m * t_tick, m >= 1 an integer; the state is read once per host step and held over its m ticks. For
// each tick j = first_tick .. first_tick + m - 1: the command source gives dshot_j, then marv_plant_step(body,
// dshot_j, t_tick) gives W_j. The applied wrench is W_bar = (W_0 + W_1 + ... + W_(m-1)) / m, summed in tick order
// starting from W_0 (not 0.0) and divided once, so m = 1 gives W_bar = W_0 bitwise.
//
// The motor sub-step of the plant is h = t_tick (exact zero-order hold). That is the caller's configuration
// (marv_plant_config::motor_substep_s = t_tick); the adapter does not check it.

#include <array>
#include <cstdint>
#include <functional>
#include <utility>
#include <vector>

#include "marv/gz/frames.hpp"
#include "marv_plant.h"

namespace marv::gz {

using Dshot = std::array<std::uint16_t, MARV_PLANT_N_MOTORS>;

// t_tick in seconds from the SIL period rational num_us / den microseconds, one rounding: num_us / (den * 1e6).
double tick_period_s(std::uint32_t num_us, std::uint32_t den);

class CommandSource {
 public:
  virtual ~CommandSource() = default;
  // The DShot values of tick `tick`, logical motors 1..4 at index 0..3. False on failure.
  virtual bool dshot(std::uint64_t tick, Dshot& out) = 0;
};

// A function of the tick number, for tests.
class ScriptedCommandSource final : public CommandSource {
 public:
  explicit ScriptedCommandSource(std::function<Dshot(std::uint64_t)> script) : script_(std::move(script)) {}
  bool dshot(std::uint64_t tick, Dshot& out) override;

 private:
  std::function<Dshot(std::uint64_t)> script_;
};

// The real SIL: marv_sil_tick(tick, 1, &imu, out) with a zeroed IMU sample whose flags mark every channel invalid
// (marv_plant v0 has no sensor models). The caller has run marv_sil_init.
class SilCommandSource final : public CommandSource {
 public:
  bool dshot(std::uint64_t tick, Dshot& out) override;
  std::int32_t last_status() const { return status_; }        // marv_sil_status of the last call
  std::uint64_t last_stamp_us() const { return stamp_us_; }   // the SIL's t_us of the last tick

 private:
  std::int32_t status_ = 0;
  std::uint64_t stamp_us_ = 0;
};

struct TickOutput {
  std::uint64_t tick = 0;
  Dshot dshot{};
  marv_plant_out out{};  // as marv_plant_step returned it, NED
  Wrench wrench_enu{};   // W_j
};

enum class Status { kOk, kZeroTicks, kCommandSource, kPlant };

struct StepResult {
  Status status = Status::kOk;
  std::int32_t plant_status = MARV_PLANT_OK;  // of the failing marv_plant_step when status is kPlant
  Wrench wrench_enu{};                        // W_bar, valid when status is kOk
  std::vector<TickOutput> ticks;              // the ticks completed, in order
};

class Adapter {
 public:
  // Takes ownership of `plant` (destroyed with the adapter). `source` must outlive the adapter.
  Adapter(marv_plant* plant, CommandSource& source, double t_tick_s);
  ~Adapter();
  Adapter(const Adapter&) = delete;
  Adapter& operator=(const Adapter&) = delete;

  double t_tick_s() const { return t_tick_s_; }

  // m >= 1 else kZeroTicks with nothing done. On a failure part-way the plant has already advanced by the completed
  // ticks and `ticks` holds them.
  StepResult step(const marv_plant_body& body, std::uint64_t first_tick, std::uint32_t m);

 private:
  marv_plant* plant_;
  CommandSource& source_;
  double t_tick_s_;
};

}  // namespace marv::gz
