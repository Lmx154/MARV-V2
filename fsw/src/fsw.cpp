#include <marv/fsw/fsw.hpp>

namespace marv {

namespace {

template <class E> State run(E& e, const SensorBus& bus) {
    e.update(bus);
    return e.state();
}

}  // namespace

Fsw::Estimators Fsw::make_estimator(EstimatorKind k) {
    switch (k) {
        case EstimatorKind::kEkf: return Estimators(std::in_place_type<Ekf>);
        case EstimatorKind::kUkf: return Estimators(std::in_place_type<Ukf>);
        case EstimatorKind::kMahony: return Estimators(std::in_place_type<Mahony>);
        case EstimatorKind::kComplementary: return Estimators(std::in_place_type<Complementary>);
        case EstimatorKind::kEskf: break;
    }
    return Estimators(std::in_place_type<Eskf>);
}

Fsw::Fsw(std::uint8_t preset)
    : preset_(preset_or_default(preset).id), estimator_(make_estimator(preset_or_default(preset).estimator)) {}

Tick Fsw::step(const SensorBus& bus) {
    const float dt = have_prev_ && bus.t_us > t_prev_us_ ? static_cast<float>(bus.t_us - t_prev_us_) * 1e-6f : 0.f;
    t_prev_us_ = bus.t_us;
    have_prev_ = true;

    if ((bus.fresh & kGnss) && bus.gnss.fix && !home_.valid()) home_.set({bus.gnss.lat_e7, bus.gnss.lon_e7, bus.gnss.alt_m});

    // Direct calls on the alternative held (no visit table).
    State est{};
    if (auto* e = std::get_if<Eskf>(&estimator_)) est = run(*e, bus);
    else if (auto* e = std::get_if<Ekf>(&estimator_)) est = run(*e, bus);
    else if (auto* e = std::get_if<Ukf>(&estimator_)) est = run(*e, bus);
    else if (auto* e = std::get_if<Mahony>(&estimator_)) est = run(*e, bus);
    else if (auto* e = std::get_if<Complementary>(&estimator_)) est = run(*e, bus);

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
    out.tlm.preset = preset_;
    out.tlm.home_valid = home_.valid();
    out.tlm.home = home_.origin();
    return out;
}

}  // namespace marv
