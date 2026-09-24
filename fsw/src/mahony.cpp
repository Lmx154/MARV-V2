// Port of the avionics-toolbox mahony estimator block (src/lib/sim/lab/blocks/estimator.ts).
// Deviations from the TypeScript:
//  - float throughout.
//  - The magnetometer weight 1 / (mag_rate_hz T) is the measured interval since the previous magnetometer sample over
//    the IMU interval: the same number at steady rates. A magnetometer sample counts only on a tick with an IMU sample,
//    as in the toolbox (its bus carries both on the same frame).
//  - Heading: the magnetometer vector tilt-compensated with the estimated attitude (eskf.cpp), not a heading message.
//  - Initialisation is Eskf's stationary alignment (tilt, heading) instead of truth plus a heading error; the bias
//    estimate starts at zero, as in the toolbox (the alignment's gyro average is not used).
//  - The reported uncertainty (sigmaPNominal, sigmaThetaNominal) is not ported: State has no covariance.
#include <marv/fsw/mahony.hpp>

#include <cmath>

#include <marv/fsw/math.hpp>

namespace marv {

Mahony::Mahony(const MahonyParams& p, const EskfParams& env)
    : prm_(p),
      gravity_(env.gravity),
      mag_decl_(std::atan2(env.mag_ref_ned_ut.y, env.mag_ref_ned_ut.x)),
      alignment_(env),
      tr_(p.blend, env.gravity) {}

void Mahony::update(const SensorBus& bus) {
    t_us_ = bus.t_us;
    if ((bus.fresh & kGnss) && bus.gnss.fix && !frame_.valid()) frame_.set({bus.gnss.lat_e7, bus.gnss.lon_e7, bus.gnss.alt_m});

    if (!aligned_) {
        Alignment a;
        if (alignment_.feed(bus, a)) {
            q_ = a.q;
            baro_h0_ = a.baro_h0;
            tr_.reset(bus.t_us);
            last_imu_us_ = bus.t_us;
            last_mag_us_ = bus.t_us;
            if (bus.fresh & kImu) w_meas_ = bus.imu.gyro_frd;
            aligned_ = true;
        }
        return;
    }

    if (bus.fresh & kImu) {
        const Vec3 am = bus.imu.accel_frd, wm = bus.imu.gyro_frd;
        w_meas_ = wm;
        const float dt = static_cast<float>(bus.t_us - last_imu_us_) * 1e-6f;
        last_imu_us_ = bus.t_us;
        if (dt > 0.f) {
            tr_.predict(q_, am, dt);
            const Vec3 down = rotate_inv(q_, Vec3{0.f, 0.f, 1.f});
            // w_mes = sum k_i v_i x v_hat_i: measured x estimated gravity direction, and the heading error about the vertical.
            Vec3 e{0.f, 0.f, 0.f};
            const float f = norm(am);
            if (f > 0.f && std::fabs(f - gravity_) <= prm_.accel_gate) e = cross((-1.f / f) * am, down);
            if (bus.fresh & kMag) {
                const float w_mag = static_cast<float>(bus.t_us - last_mag_us_) * 1e-6f / dt;
                last_mag_us_ = bus.t_us;
                e += (w_mag * heading_innovation(q_, bus.mag.field_frd_ut, mag_decl_)) * down;
            }
            bias_ -= (prm_.k_i * dt) * e;
            q_ = normalized(q_ * quat_from_rotvec(dt * (wm - bias_ + prm_.k_p * e)));
        }
    }
    const float baro_h = (bus.fresh & kBaro) ? isa_height(bus.baro.pressure_pa) - baro_h0_ : 0.f;
    tr_.correct(bus, frame_, baro_h);
}

State Mahony::state() const {
    State s{};
    s.t_us = t_us_;
    s.q = kQuatIdentity;
    if (!aligned_) return s;
    s.p_ned = tr_.pos();
    s.v_ned = tr_.vel();
    s.q = q_;
    s.w_frd = w_meas_ - bias_;
    s.valid = frame_.valid();
    return s;
}

}  // namespace marv
