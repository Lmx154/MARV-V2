// Prints the firmware's float biquad coefficients (marv_gyro_chain, gyro_chain.hpp) for the inputs on stdin, as exact
// hexadecimal floats, for test_gyro_chain_design.py (decision 0013, cross-check of the Python design block against
// the firmware). Input lines: "notch <f0> <q> <period>" or "lowpass <fc> <period>", each number a C hexadecimal float
// (a float32 value). Output, one line per input: b0 b1 b2 a1 a2 as hexadecimal doubles of the float results.
#include <cstdio>

#include "marv/gyro_chain/gyro_chain.hpp"

int main() {
  char kind[16];
  while (std::scanf("%15s", kind) == 1) {
    marv::gyro_chain::BiquadCoeffs<float> c;
    if (kind[0] == 'n') {
      float f0 = 0;
      float q = 0;
      float period = 0;
      if (std::scanf("%a %a %a", &f0, &q, &period) != 3) {
        return 1;
      }
      c = marv::gyro_chain::notch_coeffs<float>(f0, q, period);
    } else {
      float fc = 0;
      float period = 0;
      if (std::scanf("%a %a", &fc, &period) != 2) {
        return 1;
      }
      c = marv::gyro_chain::lowpass_coeffs<float>(fc, period);
    }
    std::printf("%a %a %a %a %a\n", static_cast<double>(c.b0), static_cast<double>(c.b1), static_cast<double>(c.b2),
                static_cast<double>(c.a1), static_cast<double>(c.a2));
  }
  return 0;
}
