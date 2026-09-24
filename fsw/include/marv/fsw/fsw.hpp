// One flight-software tick: sensors in, telemetry and actuator commands out. Runs unchanged on the Pico
// and on the host.
//
//   mission -> guidance -> controller -> allocation -> actuators
//
// The controller flies on the navigation source the mission selects: the simulator's truth (the lab's
// truth-fed mode) or the estimate. The estimator runs every tick either way, so on a truth-fed flight it
// is in shadow: its output is reported in telemetry and compared against truth, but flies nothing.
// Until it has aligned and taken its GNSS origin (the first fix) its state is invalid, and a mission on it keeps
// the motors at zero; once valid it stays valid for the run.
//
// Every module is built once from the Setup given at construction (params.hpp): the estimator is its estimator
// kind, and each module copies its constants from the setup's typed parameters. Nothing reads the Setup afterwards.
// The vehicle kind picks the chain after the estimator:
//   uav     passthrough or trajectory guidance (the setup's guidance kind) -> cascaded PID -> quad-x allocation ->
//           rotor-speed-fraction actuators; brake zero
//   rocket  apogee-predictor guidance -> apogee PID -> rocket-brake allocation -> brake-servo actuators (the
//           deployment, unchanged); every motor zero, always
#pragma once

#include <cstdint>
#include <variant>

#include <marv/fsw/actuators.hpp>
#include <marv/fsw/allocation.hpp>
#include <marv/fsw/apogee_pid.hpp>
#include <marv/fsw/complementary.hpp>
#include <marv/fsw/contracts.hpp>
#include <marv/fsw/controller.hpp>
#include <marv/fsw/ekf.hpp>
#include <marv/fsw/eskf.hpp>
#include <marv/fsw/geo.hpp>
#include <marv/fsw/guidance.hpp>
#include <marv/fsw/mahony.hpp>
#include <marv/fsw/presets.hpp>
#include <marv/fsw/rocket_brake.hpp>

namespace marv {

// The modules this build flies besides the preset's estimator.
using ActiveController = Controller;
using ActiveAllocation = Allocation;
using ActiveActuators = Actuators;

struct Tick {
    Telemetry tlm;
    ActuatorCommand act;
};

class Fsw {
public:
    explicit Fsw(const param::Setup& setup);
    // The factory setup (presets.hpp kFactory) this one equals, else 0xFF: Telemetry::preset.
    std::uint8_t preset() const { return preset_; }
    // param::setup_crc of the setup it runs.
    std::uint32_t setup_crc() const { return crc_; }
    void on_mission(const MissionCommand& m) { mission_ = m; }
    void on_truth(const State& s) { truth_ = s; }
    Tick step(const SensorBus& bus);

private:
    MissionCommand mission_{Mode::kIdle, NavSource::kEstimate, {}};
    State truth_{};
    using Estimators = std::variant<Eskf, Ekf, Mahony, Complementary>;
    // Builds the alternative in place (variant::emplace would stage a whole variant on the stack).
    static Estimators make_estimator(const param::Setup& s);

    std::uint8_t preset_;
    std::uint32_t crc_;
    bool uav_;  // the setup's vehicle kind is the uav, else the rocket
    std::uint8_t guidance_;  // the setup's guidance kind
    Estimators estimator_;
    LocalFrame home_;  // the first GNSS fix
    ActiveController controller_;
    ActiveAllocation allocation_;
    ActiveActuators actuators_;
    TrajectoryGuidance trajectory_;
    ApogeePredictor apogee_predictor_;
    ApogeePid apogee_pid_;
    RocketBrake rocket_brake_;
    std::uint64_t t_prev_us_ = 0;
    bool have_prev_ = false;
};

}  // namespace marv
