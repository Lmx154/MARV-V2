#include <marv/fsw/fsw.hpp>

namespace marv {

Tick Fsw::step(const SensorBus& bus) {
    const float dt = have_prev_ && bus.t_us > t_prev_us_ ? static_cast<float>(bus.t_us - t_prev_us_) * 1e-6f : 0.f;
    t_prev_us_ = bus.t_us;
    have_prev_ = true;

    estimator_.update(bus);
    const State est = estimator_.state();

    // Truth counts only if it is valid and stamped for this tick.
    const bool truth_ok = truth_.valid && truth_.t_us == bus.t_us;
    const State& nav = mission_.nav == NavSource::kTruth ? truth_ : est;
    const bool nav_ok = mission_.nav == NavSource::kTruth ? truth_ok : est.valid;
    const Mode mode = nav_ok ? mission_.mode : Mode::kIdle;

    // Guidance: the mission's reference, passed through for now.
    const Reference& ref = mission_.ref;

    Tick out{};
    const ControlRequest req = controller_.run(ref, nav, mode, dt);
    out.act = allocation_.run(req, nav, mode);
    out.act.t_us = bus.t_us;
    out.tlm.t_us = bus.t_us;
    out.tlm.est = est;
    out.tlm.req = req;
    return out;
}

}  // namespace marv
