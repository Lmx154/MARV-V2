#include <marv/fsw/fsw.hpp>

#include <cstring>

namespace marv {

namespace {

template <class E> State run(E& e, const SensorBus& bus) {
    e.update(bus);
    return e.state();
}

// The factory setup s equals, kinds and values bit for bit, else 0xFF.
std::uint8_t factory_id(const param::Setup& s) {
    for (std::uint8_t i = 0; i < sizeof(kFactory) / sizeof(kFactory[0]); ++i)
        if (std::memcmp(s.kind, kFactory[i].kind, sizeof(s.kind)) == 0 &&
            std::memcmp(s.values, kFactory[i].values, sizeof(s.values)) == 0)
            return i;
    return 0xFF;
}

}  // namespace

Fsw::Estimators Fsw::make_estimator(const param::Setup& s) {
    const param::SensorParams sensors = param::sensors_suite(s);
    switch (s.kind[param::k_estimator]) {
        case param::k_estimator_ekf:
            return Estimators(std::in_place_type<Ekf>, param::estimator_ekf(s), sensors);
        case param::k_estimator_mahony:
            return Estimators(std::in_place_type<Mahony>, param::estimator_mahony(s), sensors);
        case param::k_estimator_complementary:
            return Estimators(std::in_place_type<Complementary>, param::estimator_complementary(s), sensors);
        default: break;
    }
    return Estimators(std::in_place_type<Eskf>, param::estimator_eskf(s), sensors);
}

Fsw::Fsw(const param::Setup& setup)
    : preset_(factory_id(setup)),
      crc_(param::setup_crc(setup)),
      uav_(setup.kind[param::k_vehicle] == param::k_vehicle_uav),
      estimator_(make_estimator(setup)),
      controller_(param::controller_cascaded_pid(setup), param::vehicle_uav(setup), param::sensors_suite(setup)),
      actuators_(param::actuators_rotor_speed_fraction(setup)) {}

Tick Fsw::step(const SensorBus& bus) {
    const float dt = have_prev_ && bus.t_us > t_prev_us_ ? static_cast<float>(bus.t_us - t_prev_us_) * 1e-6f : 0.f;
    t_prev_us_ = bus.t_us;
    have_prev_ = true;

    if ((bus.fresh & kGnss) && bus.gnss.fix && !home_.valid()) home_.set({bus.gnss.lat_e7, bus.gnss.lon_e7, bus.gnss.alt_m});

    // Direct calls on the alternative held (no visit table).
    State est{};
    if (auto* e = std::get_if<Eskf>(&estimator_)) est = run(*e, bus);
    else if (auto* e = std::get_if<Ekf>(&estimator_)) est = run(*e, bus);
    else if (auto* e = std::get_if<Mahony>(&estimator_)) est = run(*e, bus);
    else if (auto* e = std::get_if<Complementary>(&estimator_)) est = run(*e, bus);

    // Truth counts only if it is valid and stamped for this tick.
    const bool truth_ok = truth_.valid && truth_.t_us == bus.t_us;
    const State& nav = mission_.nav == NavSource::kTruth ? truth_ : est;
    const bool nav_ok = mission_.nav == NavSource::kTruth ? truth_ok : est.valid;
    const Mode mode = nav_ok && uav_ ? mission_.mode : Mode::kIdle;

    // Guidance: the mission's reference, passed through for now.
    const Reference& ref = mission_.ref;

    Tick out{};
    const ControlRequest req = controller_.run(ref, nav, mode, dt);
    out.act = allocation_.run(req, nav, mode);
    actuators_.run(out.act);
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
