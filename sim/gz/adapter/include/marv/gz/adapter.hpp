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
//
// Clock error e (decision 0012, owner decisions 2 and 3) enters in exactly one place: the sim-side true tick time
// t_true = t_nom / (1 + e), computed by true_tick_period_s and stored as the adapter's tick time. That value is the dt
// of marv_plant_step and of marv_plant_imu_sample. e never reaches the SIL: its tick period stays the nominal rational
// and its stamps are unchanged. e = corner * odr_error, corner in {-1, 0, +1}; corner 0 gives t_true = t_nom bitwise.
//
// Opt-in IMU path (AdapterConfig::imu non-null; the default leaves every output as without it). The adapter attaches
// the plant's IMU once at construction. For each tick j, in this order: (1) marv_plant_imu_sample(body, t_true) gives the
// sample of tick j; (2) the command source takes it and gives dshot_j; (3) marv_plant_step(body, dshot_j, t_true).
// The sample of tick j therefore measures the body held over the host step and the rotor state after step j - 1 (the
// initial rotor state at the first tick): what the SIL's tick j can have seen before it issues dshot_j, so dshot_j
// cannot reach its own sample. The truth-gyro path has the same freshness (truth_gyro.hpp: the body at the host step's
// start, taken before the tick's step).
//
// Opt-in rotor-speed path (AdapterConfig::rotor_speed non-null, L6 stage (b), decision 0013; independent of the IMU path;
// the default leaves every output as without it). The adapter attaches the plant's rotor-speed sensor once at
// construction. For each tick j, in this order: (1) the IMU sample, if that path is on; (2) marv_plant_rotor_speed_sample
// gives the sample of tick j, from the rotor state after step j - 1 (the initial rotor state at the first tick), the same
// freshness as the IMU sample; (3) the command source takes both and gives dshot_j; (4) marv_plant_step. The rotor sample
// reaches the SIL as the bytes the plant produced, through RotorSilCommandSource (marv_sil_tick_with_rotor_speed). A source
// that does not override the rotor-speed form ignores the sample. No truth symbol.

#include <array>
#include <cstdint>
#include <functional>
#include <utility>
#include <vector>

#include "marv/gz/frames.hpp"
#include "marv_plant.h"
#include "marv_sil.h"

namespace marv::gz {

using Dshot = std::array<std::uint16_t, MARV_PLANT_N_MOTORS>;

// t_tick in seconds from the SIL period rational num_us / den microseconds, one rounding: num_us / (den * 1e6).
double tick_period_s(std::uint32_t num_us, std::uint32_t den);

// The fractional clock error e = corner * odr_error. corner must be -1, 0 or +1 and odr_error finite with
// |odr_error| < 1, else NaN (which marv_plant_step refuses as MARV_PLANT_E_DT).
double clock_error(int corner, double odr_error);

// The sim-side true tick time t_nom_s / (1 + e): the only place e is applied. NaN if e is non-finite or 1 + e <= 0
// (t_nom_s is not checked: marv_plant_step validates dt). e = 0 returns t_nom_s bitwise (division by exactly 1).
double true_tick_period_s(double t_nom_s, double e);

class CommandSource {
 public:
  virtual ~CommandSource() = default;
  // The DShot values of tick `tick`, logical motors 1..4 at index 0..3. False on failure.
  virtual bool dshot(std::uint64_t tick, Dshot& out) = 0;
  // The same, given the IMU sample of the tick (never null when called by an Adapter with the IMU path enabled). The
  // default ignores the sample and calls the method above.
  virtual bool dshot(std::uint64_t tick, const marv_imu_meas* imu, Dshot& out);
  // The same, given the rotor-speed sample of the tick as well (never null when called by an Adapter with the rotor-speed
  // path enabled; imu is null if the IMU path is off). The default ignores the rotor sample and calls the IMU form when
  // imu is non-null, else the first form.
  virtual bool dshot(std::uint64_t tick, const marv_imu_meas* imu, const marv_rotor_speed_meas* rotor, Dshot& out);
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

// The non-truth IMU source: marv_sil_tick(tick, 1, imu, out) with the plant's IMU bytes verbatim (marv_plant_imu_out
// and marv_imu_meas have the same layout, static_asserted in adapter.cpp). Without a sample (the two-argument call or a
// null pointer) it passes a zeroed one, as SilCommandSource. The caller has run marv_sil_init. No truth symbol.
class ImuSilCommandSource final : public CommandSource {
 public:
  bool dshot(std::uint64_t tick, Dshot& out) override;
  bool dshot(std::uint64_t tick, const marv_imu_meas* imu, Dshot& out) override;
  std::int32_t last_status() const { return status_; }        // marv_sil_status of the last call
  std::uint64_t last_stamp_us() const { return stamp_us_; }   // the SIL's t_us of the last tick

 private:
  std::int32_t status_ = 0;
  std::uint64_t stamp_us_ = 0;
};

// The SIL with the rotor-speed sample: marv_sil_tick_with_rotor_speed(tick, 1, imu, rotor, out) with the plant's rotor-speed
// bytes verbatim (marv_plant_rotor_speed_out and marv_rotor_speed_meas have the same layout, static_asserted in
// rotor_sil.cpp). A null imu or a null rotor passes a zeroed sample of that kind; with a null rotor it calls marv_sil_tick
// (the HAL then reports the rotor speed invalid). The caller has run marv_sil_init and links a SIL library. No truth
// symbol. Its code is in rotor_sil.cpp, a separate object of the adapter library, so that a program that does not use it
// does not need marv_sil_tick_with_rotor_speed.
class RotorSilCommandSource final : public CommandSource {
 public:
  bool dshot(std::uint64_t tick, Dshot& out) override;
  bool dshot(std::uint64_t tick, const marv_imu_meas* imu, Dshot& out) override;
  bool dshot(std::uint64_t tick, const marv_imu_meas* imu, const marv_rotor_speed_meas* rotor, Dshot& out) override;
  std::int32_t last_status() const { return status_; }        // marv_sil_status of the last call
  std::uint64_t last_stamp_us() const { return stamp_us_; }   // the SIL's t_us of the last tick

 private:
  std::int32_t status_ = 0;
  std::uint64_t stamp_us_ = 0;
};

struct TickOutput {
  std::uint64_t tick = 0;
  Dshot dshot{};
  marv_plant_imu_out imu{};  // the sample of this tick; zero unless the IMU path is enabled
  marv_plant_rotor_speed_out rotor_speed{};  // the rotor-speed sample of this tick; zero unless that path is enabled
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

struct AdapterConfig {
  double t_tick_nominal_s = 0.0;                // t_nom, from tick_period_s
  int clock_corner = 0;                         // -1, 0, +1
  double odr_error = 0.0;                       // magnitude of the fractional clock error, from the caller
  const marv_plant_imu_config* imu = nullptr;   // non-null enables the IMU path (read during construction only)
  const marv_plant_rotor_speed_config* rotor_speed = nullptr;  // non-null enables the rotor-speed path (read during
                                                               // construction only)
  // 0 (the default): t_true = t_nom / (1 + e) as above. Nonzero: t_true itself, as the caller realised it (decision
  // 0019: the lockstep plugin's host step n_true ns divided by m * 1e9, rounded once); clock_corner and odr_error are
  // then not used. The plant's clock only: the SIL's period and stamps stay nominal.
  double t_tick_true_s = 0.0;
};

class Adapter {
 public:
  // Takes ownership of `plant` (destroyed with the adapter). `source` must outlive the adapter. The three-argument
  // form is the config with t_tick_nominal_s = t_tick_s, corner 0 and no IMU.
  Adapter(marv_plant* plant, CommandSource& source, double t_tick_s);
  Adapter(marv_plant* plant, CommandSource& source, const AdapterConfig& cfg);
  ~Adapter();
  Adapter(const Adapter&) = delete;
  Adapter& operator=(const Adapter&) = delete;

  double t_tick_s() const { return t_tick_s_; }  // t_true
  bool imu_enabled() const { return imu_enabled_; }
  marv_plant_status imu_attach_status() const { return imu_attach_status_; }  // MARV_PLANT_OK when the IMU is off
  bool rotor_speed_enabled() const { return rotor_speed_enabled_; }
  marv_plant_status rotor_speed_attach_status() const { return rotor_speed_attach_status_; }  // OK when it is off

  // m >= 1 else kZeroTicks with nothing done. A failed IMU or rotor-speed attach is kPlant with that status
  // (the IMU's first), nothing done. On a
  // failure part-way the plant has already advanced by the completed ticks and `ticks` holds them.
  StepResult step(const marv_plant_body& body, std::uint64_t first_tick, std::uint32_t m);

 private:
  marv_plant* plant_;
  CommandSource& source_;
  double t_tick_s_;
  bool imu_enabled_ = false;
  marv_plant_status imu_attach_status_ = MARV_PLANT_OK;
  bool rotor_speed_enabled_ = false;
  marv_plant_status rotor_speed_attach_status_ = MARV_PLANT_OK;
};

}  // namespace marv::gz
