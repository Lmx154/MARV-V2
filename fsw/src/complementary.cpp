// Port of the avionics-toolbox complementary estimator block and translationBlend (src/lib/sim/lab/blocks/estimator.ts).
// Deviations from the TypeScript:
//  - float throughout.
//  - The per-sample blend fractions 1 - exp(-k / rate_hz) use the measured interval since the previous sample of that
//    sensor instead of a configured rate: the same number at a steady rate.
//  - Heading: the magnetometer vector tilt-compensated with the estimated attitude (eskf.cpp), not a heading message.
//  - Initialisation is Eskf's stationary alignment (tilt, heading) instead of truth plus a heading error; position and
//    velocity start at zero, the barometer reference is the aligned mean height. The gyro bias stays zero, as in the
//    toolbox (the alignment's gyro average is not used).
//  - The reported uncertainty (sigmaPNominal, sigmaThetaNominal) is not ported: State has no covariance.
#include <marv/fsw/complementary.hpp>

#include <cmath>

#include <marv/fsw/math.hpp>

namespace marv {

namespace {

float blend(float k, float dt) { return 1.f - std::exp(-k * dt); }

}  // namespace

void TranslationBlend::reset(std::uint64_t t_us) {
    pos_ = {0.f, 0.f, 0.f};
    vel_ = {0.f, 0.f, 0.f};
    last_gnss_us_ = t_us;
    last_baro_us_ = t_us;
}

void TranslationBlend::reset_horizontal() { pos_.x = pos_.y = 0.f; }

void TranslationBlend::predict(Quat q, Vec3 am, float dt) {
    const Vec3 a = rotate(q, am) + Vec3{0.f, 0.f, gravity_};
    pos_ = pos_ + dt * vel_ + (0.5f * dt * dt) * a;
    vel_ = vel_ + dt * a;
}

void TranslationBlend::correct(const SensorBus& bus, const LocalFrame& frame, float baro_h) {
    if ((bus.fresh & kGnss) && bus.gnss.fix && frame.valid()) {
        const float dt = static_cast<float>(bus.t_us - last_gnss_us_) * 1e-6f;
        last_gnss_us_ = bus.t_us;
        const Vec3 z = frame.to_ned({bus.gnss.lat_e7, bus.gnss.lon_e7, bus.gnss.alt_m});
        // Horizontal only: height is the barometer's, referenced at alignment (GNSS altitude is referenced to the
        // first fix and carries its error).
        const float k = blend(g_.k_pos, dt);
        pos_.x += k * (z.x - pos_.x);
        pos_.y += k * (z.y - pos_.y);
        vel_ += blend(g_.k_vel, dt) * (bus.gnss.vel_ned - vel_);
    }
    if (bus.fresh & kBaro) {
        const float dt = static_cast<float>(bus.t_us - last_baro_us_) * 1e-6f;
        last_baro_us_ = bus.t_us;
        pos_.z += blend(g_.k_baro, dt) * (-baro_h - pos_.z);
    }
}

Complementary::Complementary(const param::ComplementaryParams& p, const param::SensorParams& s)
    : prm_(p),
      gravity_(s.gravity),
      mag_decl_(std::atan2(s.mag_ref_ned_ut_y, s.mag_ref_ned_ut_x)),
      alignment_(s, s.gravity),
      tr_({p.k_pos, p.k_vel, p.k_baro}, s.gravity) {}

void Complementary::update(const SensorBus& bus) {
    t_us_ = bus.t_us;
    if ((bus.fresh & kGnss) && bus.gnss.fix && !frame_.valid()) {
        frame_.set({bus.gnss.lat_e7, bus.gnss.lon_e7, bus.gnss.alt_m});
        tr_.reset_horizontal();  // first fix after alignment: horizontal position restarts there
    }

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
            q_ = normalized(q_ * quat_from_rotvec(dt * wm));
            const float f = norm(am);
            if (f > 0.f && std::fabs(f - gravity_) <= prm_.accel_gate) {
                // Rotate the estimated down axis (body) toward the measured one, -a_m / |a_m|.
                const Vec3 down = rotate_inv(q_, Vec3{0.f, 0.f, 1.f});
                q_ = normalized(q_ * quat_from_rotvec(blend(prm_.k_acc, dt) * cross((-1.f / f) * am, down)));
            }
        }
    }
    if (bus.fresh & kMag) {
        const float dt = static_cast<float>(bus.t_us - last_mag_us_) * 1e-6f;
        last_mag_us_ = bus.t_us;
        const float e = heading_innovation(q_, bus.mag.field_frd_ut, mag_decl_);
        q_ = normalized(quat_from_rotvec(Vec3{0.f, 0.f, blend(prm_.k_mag, dt) * e}) * q_);
    }
    const float baro_h = (bus.fresh & kBaro) ? isa_height(bus.baro.pressure_pa) - baro_h0_ : 0.f;
    tr_.correct(bus, frame_, baro_h);
}

State Complementary::state() const {
    State s{};
    s.t_us = t_us_;
    s.q = kQuatIdentity;
    if (!aligned_) return s;
    s.p_ned = tr_.pos();
    s.v_ned = tr_.vel();
    s.q = q_;
    s.w_frd = w_meas_;
    s.valid = frame_.valid();
    return s;
}

}  // namespace marv
