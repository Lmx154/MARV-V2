// One flight-software tick: sensors in, telemetry and actuator commands out. Runs unchanged on the Pico
// and on the host.
//
//   mission -> guidance -> controller -> allocation -> actuators
//
// The controller flies on the navigation source the mission selects: the simulator's truth (the lab's
// truth-fed mode) or the estimate. The estimator runs every tick either way, so on a truth-fed flight it
// is in shadow: its output is reported in telemetry and compared against truth, but flies nothing.
// Until it has aligned its state is invalid, and a mission on it keeps the motors at zero.
#pragma once

#include <cstdint>

#include <marv/fsw/allocation.hpp>
#include <marv/fsw/contracts.hpp>
#include <marv/fsw/controller.hpp>
#include <marv/fsw/eskf.hpp>

namespace marv {

// The modules this build flies. Swap one by editing its alias.
using ActiveEstimator = Eskf;
using ActiveController = Controller;
using ActiveAllocation = Allocation;

struct Tick {
    Telemetry tlm;
    ActuatorCommand act;
};

class Fsw {
public:
    void on_mission(const MissionCommand& m) { mission_ = m; }
    void on_truth(const State& s) { truth_ = s; }
    Tick step(const SensorBus& bus);

private:
    MissionCommand mission_{Mode::kIdle, NavSource::kEstimate, {}};
    State truth_{};
    ActiveEstimator estimator_;
    ActiveController controller_;
    ActiveAllocation allocation_;
    std::uint64_t t_prev_us_ = 0;
    bool have_prev_ = false;
};

}  // namespace marv
