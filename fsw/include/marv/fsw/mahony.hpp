// Mahony nonlinear complementary filter on SO(3) (Mahony, Hamel & Pflimlin 2008, explicit form with the PI gyro-bias
// term), ported from the avionics toolbox src/lib/sim/lab/blocks/estimator.ts (mahony block). Correction: the measured
// x estimated gravity direction (while | |a_m| - g | is within the gate) plus the magnetometer heading error about the
// vertical; the attitude integrates w_m - bias + kP e, the bias integrates -kI e. Position and velocity: the
// complementary filter's translationBlend. float only, no heap.
#pragma once

#include <cstdint>

#include <marv/fsw/complementary.hpp>
#include <marv/fsw/contracts.hpp>
#include <marv/fsw/ekf.hpp>
#include <marv/fsw/eskf.hpp>
#include <marv/fsw/geo.hpp>
#include <marv/fsw/params.hpp>

namespace marv {

class Mahony {
public:
    // s: the reference field and the alignment window (Eskf's); v: gravity.
    explicit Mahony(const param::MahonyParams& p = {}, const param::SensorParams& s = {},
                    const param::VehicleParams& v = {});

    void update(const SensorBus& bus);
    State state() const;
    bool aligned() const { return aligned_; }
    Vec3 gyro_bias() const { return bias_; }

private:
    param::MahonyParams prm_;
    float gravity_;
    float mag_decl_;
    StationaryAlignment alignment_;
    LocalFrame frame_;
    TranslationBlend tr_;
    bool aligned_ = false;
    std::uint64_t t_us_ = 0;
    std::uint64_t last_imu_us_ = 0;
    std::uint64_t last_mag_us_ = 0;
    float baro_h0_ = 0.f;
    Quat q_{1.f, 0.f, 0.f, 0.f};
    Vec3 bias_{0.f, 0.f, 0.f};
    Vec3 w_meas_{0.f, 0.f, 0.f};
};

}  // namespace marv
