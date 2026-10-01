#include "marv/gyro_chain/gyro_chain.hpp"

namespace marv::gyro_chain {

template class GyroChain<float>;
template ConfigError validate<float>(const GyroChainConfig<float>&) noexcept;
template BiquadCoeffs<float> lowpass_coeffs<float>(float, float) noexcept;
template BiquadCoeffs<float> notch_coeffs<float>(float, float, float) noexcept;

}  // namespace marv::gyro_chain
