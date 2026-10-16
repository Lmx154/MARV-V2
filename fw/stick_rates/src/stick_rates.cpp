#include "marv/stick_rates/stick_rates.hpp"

namespace marv::stick_rates {

template struct RateCurve<float>;
template struct RatesConfig<float>;
template ConfigError validate<float>(const RateCurve<float>&) noexcept;
template ConfigError validate<float>(const RatesConfig<float>&) noexcept;
template float rate_setpoint<float>(const RateCurve<float>&, float) noexcept;
template prim::Vec3<float> rate_setpoint<float>(const RatesConfig<float>&, const prim::Vec3<float>&) noexcept;

}  // namespace marv::stick_rates
