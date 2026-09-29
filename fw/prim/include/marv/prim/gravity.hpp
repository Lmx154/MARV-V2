#pragma once

#include <cmath>

#include "marv/prim/constants.hpp"

namespace marv::prim {

// WGS 84 normal gravity magnitude (m/s^2, positive), NIMA TR8350.2 3rd ed. Amdt 1 (2000), chapter 4.
// phi is geodetic latitude in rad; h is geodetic height above the ellipsoid in m, positive up.

// On the ellipsoid, Somigliana's closed formula, eq. (4-1):
//   gamma = gamma_e (1 + k sin^2 phi) / sqrt(1 - e^2 sin^2 phi)
template <class T>
T normal_gravity_ellipsoid(T phi) {
  using std::sin;
  using std::sqrt;
  const T s = sin(phi);
  const T s2 = s * s;
  return T(kWgs84GammaE) * (T(1) + T(kWgs84K) * s2) / sqrt(T(1) - T(kWgs84E2) * s2);
}

// Above the ellipsoid, second-order series along the geodetic normal, eq. (4-3):
//   gamma_h = gamma [1 - (2/a)(1 + f + m - 2 f sin^2 phi) h + (3/a^2) h^2]
// Valid for heights small against a; the neglected terms are third order in h/a. gamma_h(phi, 0) == gamma(phi).
template <class T>
T normal_gravity(T phi, T h) {
  using std::sin;
  const T s = sin(phi);
  const T s2 = s * s;
  const T inv_a = T(1) / T(kWgs84A);
  const T linear = T(2) * inv_a * (T(1) + T(kWgs84F) + T(kWgs84M) - T(2) * T(kWgs84F) * s2) * h;
  const T quad = T(kWgs84HeightQuadCoeff) * inv_a * inv_a * h * h;
  return normal_gravity_ellipsoid(phi) * (T(1) - linear + quad);
}

}  // namespace marv::prim
