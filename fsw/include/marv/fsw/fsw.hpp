// One flight-software tick: sensors in, telemetry and actuator commands out. Runs unchanged on the Pico
// and on the host.
//
//   mission -> guidance -> controller -> allocation -> actuators
//
// The controller flies on the navigation source the mission selects: the simulator's truth (the lab's
// truth-fed mode) or the estimate. The estimator runs every tick either way, so on a truth-fed flight it
// is in shadow: its output is reported in telemetry and compared against truth, but flies nothing.
// Until it has aligned its state is invalid, and a mission on it keeps the motors at zero.
//
// The estimator is the preset's (presets.hpp), chosen at construction: an unknown id runs preset 0.
#pragma once

#include <cstdint>
#include <variant>

#include <marv/fsw/allocation.hpp>
#include <marv/fsw/complementary.hpp>
#include <marv/fsw/contracts.hpp>
#include <marv/fsw/controller.hpp>
#include <marv/fsw/ekf.hpp>
#include <marv/fsw/eskf.hpp>
#include <marv/fsw/geo.hpp>
#include <marv/fsw/mahony.hpp>
#include <marv/fsw/presets.hpp>
#include <marv/fsw/ukf.hpp>

namespace marv {

// The modules this build flies besides the preset's estimator.
using ActiveController = Controller;
using ActiveAllocation = Allocation;

struct Tick {
    Telemetry tlm;
    ActuatorCommand act;
};

class Fsw {
public:
    explicit Fsw(std::uint8_t preset = 0);
    std::uint8_t preset() const { return preset_; }
    void on_mission(const MissionCommand& m) { mission_ = m; }
    void on_truth(const State& s) { truth_ = s; }
    Tick step(const SensorBus& bus);

private:
    MissionCommand mission_{Mode::kIdle, NavSource::kEstimate, {}};
    State truth_{};
    using Estimators = std::variant<Eskf, Ekf, Ukf, Mahony, Complementary>;
    // Builds the alternative in place (variant::emplace would stage a whole variant on the stack).
    static Estimators make_estimator(EstimatorKind k);

    std::uint8_t preset_;
    Estimators estimator_;
    LocalFrame home_;  // the first GNSS fix
    ActiveController controller_;
    ActiveAllocation allocation_;
    std::uint64_t t_prev_us_ = 0;
    bool have_prev_ = false;
};

}  // namespace marv
