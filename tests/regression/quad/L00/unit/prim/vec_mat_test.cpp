#include <gtest/gtest.h>

#include <cstddef>

#include "marv/prim/mat.hpp"
#include "marv/prim/vec.hpp"
#include "test_util.hpp"

namespace marv::prim {
namespace {

using test::expect_vec_near;
using test::kOpsShort;
using test::tol;

template <class T>
class VecMatTest : public ::testing::Test {};

using Scalars = ::testing::Types<float, double>;
TYPED_TEST_SUITE(VecMatTest, Scalars);

TYPED_TEST(VecMatTest, VecConstructIndexAndArithmetic) {
  using T = TypeParam;
  const Vec3<T> zero;
  EXPECT_EQ(zero[0], T(0));
  EXPECT_EQ(zero[1], T(0));
  EXPECT_EQ(zero[2], T(0));

  const Vec3<T> a(T(1), T(2), T(3));
  const Vec3<T> b(T(4), T(-5), T(6));
  EXPECT_EQ(a[0], T(1));
  EXPECT_EQ(a[1], T(2));
  EXPECT_EQ(a[2], T(3));

  // Small integers are exact in float and double: sums and products of them are exact.
  const Vec3<T> sum = a + b;
  EXPECT_EQ(sum[0], T(5));
  EXPECT_EQ(sum[1], T(-3));
  EXPECT_EQ(sum[2], T(9));
  const Vec3<T> diff = a - b;
  EXPECT_EQ(diff[0], T(-3));
  EXPECT_EQ(diff[1], T(7));
  EXPECT_EQ(diff[2], T(-3));
  const Vec3<T> scaled = a * T(-2);
  EXPECT_EQ(scaled[0], T(-2));
  EXPECT_EQ(scaled[1], T(-4));
  EXPECT_EQ(scaled[2], T(-6));
  const Vec3<T> neg = -a;
  EXPECT_EQ(neg[0], T(-1));
  EXPECT_EQ(neg[2], T(-3));

  EXPECT_EQ(a.dot(b), T(4 - 10 + 18));
}

TYPED_TEST(VecMatTest, VecNorm) {
  using T = TypeParam;
  const Vec3<T> v(T(3), T(4), T(0));
  // dot is exact (25); sqrt is correctly rounded, so 5 is exact. n = kOpsShort covers it anyway.
  EXPECT_NEAR(v.norm(), T(5), tol<T>(kOpsShort));
  EXPECT_EQ(Vec3<T>().norm(), T(0));
}

TYPED_TEST(VecMatTest, CrossProductIsRightHanded) {
  using T = TypeParam;
  const Vec3<T> ex(T(1), T(0), T(0));
  const Vec3<T> ey(T(0), T(1), T(0));
  const Vec3<T> ez(T(0), T(0), T(1));
  expect_vec_near(ex.cross(ey), ez, T(0));
  expect_vec_near(ey.cross(ez), ex, T(0));
  expect_vec_near(ez.cross(ex), ey, T(0));
  expect_vec_near(ey.cross(ex), -ez, T(0));
  expect_vec_near(ex.cross(ex), Vec3<T>(), T(0));

  const Vec3<T> a(T(1), T(2), T(3));
  const Vec3<T> b(T(4), T(5), T(6));
  expect_vec_near(a.cross(b), Vec3<T>(T(-3), T(6), T(-3)), T(0));
  EXPECT_EQ(a.cross(b).dot(a), T(0));
  EXPECT_EQ(a.cross(b).dot(b), T(0));
}

TYPED_TEST(VecMatTest, MatIdentityAndIndexing) {
  using T = TypeParam;
  const Mat3<T> i3 = Mat3<T>::identity();
  for (std::size_t r = 0; r < 3; ++r) {
    for (std::size_t c = 0; c < 3; ++c) {
      EXPECT_EQ(i3(r, c), r == c ? T(1) : T(0));
    }
  }
  const Vec3<T> v(T(1), T(-2), T(3));
  expect_vec_near(i3 * v, v, T(0));

  const Mat3<T> m({T(1), T(2), T(3), T(4), T(5), T(6), T(7), T(8), T(10)});
  EXPECT_EQ(m(0, 2), T(3));
  EXPECT_EQ(m(1, 0), T(4));
  EXPECT_EQ(m(2, 2), T(10));
  const Mat3<T> mi = m * i3;
  const Mat3<T> im = i3 * m;
  for (std::size_t k = 0; k < 9; ++k) {
    EXPECT_EQ(mi.e[k], m.e[k]);
    EXPECT_EQ(im.e[k], m.e[k]);
  }
}

TYPED_TEST(VecMatTest, MatTransposeNonSquare) {
  using T = TypeParam;
  const Mat<T, 2, 3> a({T(1), T(2), T(3), T(4), T(5), T(6)});
  const Mat<T, 3, 2> at = a.transpose();
  EXPECT_EQ(at(0, 0), T(1));
  EXPECT_EQ(at(0, 1), T(4));
  EXPECT_EQ(at(1, 0), T(2));
  EXPECT_EQ(at(1, 1), T(5));
  EXPECT_EQ(at(2, 0), T(3));
  EXPECT_EQ(at(2, 1), T(6));
  const Mat<T, 2, 3> att = at.transpose();
  for (std::size_t k = 0; k < 6; ++k) {
    EXPECT_EQ(att.e[k], a.e[k]);
  }
}

TYPED_TEST(VecMatTest, MatProductAndMatVec) {
  using T = TypeParam;
  const Mat<T, 2, 3> a({T(1), T(2), T(3), T(4), T(5), T(6)});
  const Mat<T, 3, 2> b({T(7), T(8), T(9), T(10), T(11), T(12)});
  // Small-integer products and sums are exact in both scalars.
  const Mat<T, 2, 2> ab = a * b;
  EXPECT_EQ(ab(0, 0), T(58));
  EXPECT_EQ(ab(0, 1), T(64));
  EXPECT_EQ(ab(1, 0), T(139));
  EXPECT_EQ(ab(1, 1), T(154));

  const Mat<T, 3, 3> ba = b * a;
  EXPECT_EQ(ba(0, 0), T(39));
  EXPECT_EQ(ba(1, 1), T(68));
  EXPECT_EQ(ba(2, 2), T(105));

  // (AB)^T == B^T A^T.
  const Mat<T, 2, 2> lhs = ab.transpose();
  const Mat<T, 2, 2> rhs = b.transpose() * a.transpose();
  for (std::size_t k = 0; k < 4; ++k) {
    EXPECT_EQ(lhs.e[k], rhs.e[k]);
  }

  const Vec<T, 3> v(T(1), T(0), T(-1));
  const Vec<T, 2> av = a * v;
  EXPECT_EQ(av[0], T(-2));
  EXPECT_EQ(av[1], T(-2));

  const Mat3<T> m({T(0), T(-1), T(0), T(1), T(0), T(0), T(0), T(0), T(1)});
  // Rotation of +90 deg about z: x -> y.
  expect_vec_near(m * Vec3<T>(T(1), T(0), T(0)), Vec3<T>(T(0), T(1), T(0)), T(0));
  const Mat3<T> sum = m + Mat3<T>::identity();
  EXPECT_EQ(sum(0, 0), T(1));
  EXPECT_EQ(sum(0, 1), T(-1));
  const Mat3<T> diff = m - m;
  EXPECT_EQ(diff(1, 0), T(0));
  const Mat3<T> twice = m * T(2);
  EXPECT_EQ(twice(1, 0), T(2));
}

}  // namespace
}  // namespace marv::prim
