#pragma once

#include <gtest/gtest.h>

#include <cstddef>
#include <limits>

#include "marv/prim/quat.hpp"
#include "marv/prim/vec.hpp"

namespace marv::prim::test {

// Tolerance rule. Every tolerance is n * epsilon(T), where n is a stated bound on the number of
// rounded floating-point operations on the path from inputs to the checked value (each rounding
// contributes a relative error <= epsilon/2, and library sin/cos/sqrt at most 1 ulp), and every
// quantity on that path has magnitude <= 2. Summing the first-order error terms gives an absolute
// error <= n * epsilon(T) (well inside the standard gamma_n bound). A test names its n where it
// uses one. There is no tolerance chosen by feel.
template <class T>
constexpr T tol(int n) {
  return T(n) * std::numeric_limits<T>::epsilon();
}

// Bound for one vector operation or a short chain of them (dot, cross, 3x3 product row).
inline constexpr int kOpsShort = 8;
// Bound for axis-angle construction, one or two Hamilton products and one rotation.
inline constexpr int kOpsChain = 32;

template <class T>
void expect_vec_near(const Vec3<T>& got, const Vec3<T>& want, T tolerance) {
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_NEAR(got[i], want[i], tolerance) << "component " << i;
  }
}

template <class T>
void expect_quat_near(const Quat<T>& got, const Quat<T>& want, T tolerance) {
  EXPECT_NEAR(got.w, want.w, tolerance) << "w";
  EXPECT_NEAR(got.x, want.x, tolerance) << "x";
  EXPECT_NEAR(got.y, want.y, tolerance) << "y";
  EXPECT_NEAR(got.z, want.z, tolerance) << "z";
}

}  // namespace marv::prim::test
