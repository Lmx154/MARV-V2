#pragma once

#include "marv/types/imu_sample.hpp"

namespace marv::composition {

void init() noexcept;                    // after params_init; may read params; must not write actuators
void tick(const ImuSample& s) noexcept;  // once per primary-IMU sample, in order; the (n+1)-th call is tick n

}  // namespace marv::composition
