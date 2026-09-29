#include <gtest/gtest.h>

#include <cstddef>
#include <cstring>
#include <numbers>
#include <type_traits>

#include "marv/prim/mat.hpp"
#include "marv/prim/quat.hpp"
#include "marv/prim/vec.hpp"
#include "test_util.hpp"

namespace marv::prim {
namespace {

using test::expect_quat_near;
using test::expect_vec_near;
using test::kOpsChain;
using test::kOpsShort;
using test::tol;

template <class T>
class QuatTest : public ::testing::Test {
 protected:
  static constexpr T kHalfPi = std::numbers::pi_v<T> / T(2);

  static Vec3<T> ex() { return Vec3<T>(T(1), T(0), T(0)); }
  static Vec3<T> ey() { return Vec3<T>(T(0), T(1), T(0)); }
  static Vec3<T> ez() { return Vec3<T>(T(0), T(0), T(1)); }

  // Rotations about the FRD axes: roll about x, pitch about y, yaw about z.
  static Quat<T> roll(T a) { return Quat<T>::from_axis_angle(ex(), a); }
  static Quat<T> pitch(T a) { return Quat<T>::from_axis_angle(ey(), a); }
  static Quat<T> yaw(T a) { return Quat<T>::from_axis_angle(ez(), a); }

  // A tilted unit axis (1, 2, 2) / 3.
  static Quat<T> generic(T angle) {
    return Quat<T>::from_axis_angle(Vec3<T>(T(1), T(2), T(2)) * (T(1) / T(3)), angle);
  }
};

using Scalars = ::testing::Types<float, double>;
TYPED_TEST_SUITE(QuatTest, Scalars);

TYPED_TEST(QuatTest, StorageIsWXYZInMemory) {
  using T = TypeParam;
  static_assert(std::is_standard_layout_v<Quat<T>>);
  static_assert(std::is_trivially_copyable_v<Quat<T>>);
  static_assert(sizeof(Quat<T>) == 4 * sizeof(T));
  static_assert(offsetof(Quat<T>, w) == 0);
  static_assert(offsetof(Quat<T>, x) == 1 * sizeof(T));
  static_assert(offsetof(Quat<T>, y) == 2 * sizeof(T));
  static_assert(offsetof(Quat<T>, z) == 3 * sizeof(T));

  const Quat<T> q(T(0.5), T(0.25), T(-0.75), T(2));
  T raw[4];
  std::memcpy(raw, &q, sizeof raw);
  EXPECT_EQ(raw[0], T(0.5));
  EXPECT_EQ(raw[1], T(0.25));
  EXPECT_EQ(raw[2], T(-0.75));
  EXPECT_EQ(raw[3], T(2));

  const Quat<T> id;
  std::memcpy(raw, &id, sizeof raw);
  EXPECT_EQ(raw[0], T(1));
  EXPECT_EQ(raw[1], T(0));
  EXPECT_EQ(raw[2], T(0));
  EXPECT_EQ(raw[3], T(0));
}

TYPED_TEST(QuatTest, BodyToNedDirections) {
  using T = TypeParam;
  const T tolerance = tol<T>(kOpsChain);
  // +90 deg yaw (nose right): body x (forward) -> NED east.
  expect_vec_near(this->yaw(this->kHalfPi).rotate(this->ex()), Vec3<T>(T(0), T(1), T(0)),
                  tolerance);
  // +90 deg pitch (nose up): body x -> NED up = (0, 0, -1).
  expect_vec_near(this->pitch(this->kHalfPi).rotate(this->ex()), Vec3<T>(T(0), T(0), T(-1)),
                  tolerance);
  // +90 deg roll (right side down): body y (right) -> NED down.
  expect_vec_near(this->roll(this->kHalfPi).rotate(this->ey()), Vec3<T>(T(0), T(0), T(1)),
                  tolerance);
  // Identity leaves every direction alone; negative angle reverses the sense (yaw -90: x -> west).
  expect_vec_near(Quat<T>().rotate(Vec3<T>(T(1), T(2), T(3))), Vec3<T>(T(1), T(2), T(3)),
                  tolerance);
  expect_vec_near(this->yaw(-this->kHalfPi).rotate(this->ex()), Vec3<T>(T(0), T(-1), T(0)),
                  tolerance);
}

TYPED_TEST(QuatTest, CompositionOrder) {
  using T = TypeParam;
  const T tolerance = tol<T>(kOpsChain);
  const T half = T(0.5);

  // q_na = q_nb * q_ba. Frame a is rotated into b by pitch +90 (q_ba), then b into n by yaw +90
  // (q_nb).
  const Quat<T> q_ba = this->pitch(this->kHalfPi);
  const Quat<T> q_nb = this->yaw(this->kHalfPi);
  const Quat<T> q_na = q_nb * q_ba;

  // Closed form: (cy, 0, 0, sy) * (cp, 0, sp, 0) with cy = sy = cp = sp = sqrt(1/2) is
  // (1/2, -1/2, 1/2, 1/2).
  expect_quat_near(q_na, Quat<T>(half, -half, half, half), tolerance);

  // q_na applied to v_a equals the two rotations in sequence, first q_ba then q_nb.
  const Vec3<T> vx = this->ex();
  const Vec3<T> vy = this->ey();
  expect_vec_near(q_na.rotate(vx), q_nb.rotate(q_ba.rotate(vx)), tolerance);
  expect_vec_near(q_na.rotate(vy), q_nb.rotate(q_ba.rotate(vy)), tolerance);
  // Predicted by hand: x -pitch-> (0,0,-1) -yaw-> (0,0,-1);  y -pitch-> (0,1,0) -yaw-> (-1,0,0).
  expect_vec_near(q_na.rotate(vx), Vec3<T>(T(0), T(0), T(-1)), tolerance);
  expect_vec_near(q_na.rotate(vy), Vec3<T>(T(-1), T(0), T(0)), tolerance);

  // Reversed order q_ba * q_nb is a different rotation, (1/2, 1/2, 1/2, 1/2), first yaw then
  // pitch: x -> (0,1,0) -> (0,1,0);  y -> (-1,0,0) -> (0,0,1).
  const Quat<T> q_rev = q_ba * q_nb;
  expect_quat_near(q_rev, Quat<T>(half, half, half, half), tolerance);
  expect_vec_near(q_rev.rotate(vx), Vec3<T>(T(0), T(1), T(0)), tolerance);
  expect_vec_near(q_rev.rotate(vy), Vec3<T>(T(0), T(0), T(1)), tolerance);
  // The two orders really differ: x maps to up in one, to east in the other.
  const Vec3<T> gap = q_na.rotate(vx) - q_rev.rotate(vx);
  EXPECT_GT(gap.norm(), T(1));

  // Non-axis-aligned check of the same identity, both orders.
  const Quat<T> a = this->generic(T(1));
  const Quat<T> b = this->generic(T(-2)) * this->roll(T(0.75));
  const Vec3<T> v(T(0.5), T(-1), T(0.25));
  expect_vec_near((a * b).rotate(v), a.rotate(b.rotate(v)), tolerance);
  expect_vec_near((b * a).rotate(v), b.rotate(a.rotate(v)), tolerance);
  EXPECT_GT(((a * b).rotate(v) - (b * a).rotate(v)).norm(), tolerance);
}

TYPED_TEST(QuatTest, RotationMatrixAgreesWithRotate) {
  using T = TypeParam;
  const T tolerance = tol<T>(kOpsChain);
  const Quat<T> qs[] = {Quat<T>(),
                        this->yaw(this->kHalfPi),
                        this->pitch(T(0.3)),
                        this->roll(T(-2.5)),
                        this->generic(T(1)),
                        this->generic(T(2)) * this->pitch(T(-0.7))};
  const Vec3<T> vs[] = {Vec3<T>(T(1), T(0), T(0)), Vec3<T>(T(0.5), T(-1), T(0.25)),
                        Vec3<T>(T(-1), T(1), T(1))};
  for (const Quat<T>& q : qs) {
    const Mat3<T> r = q.to_rotation_matrix();
    for (const Vec3<T>& v : vs) {
      expect_vec_near(r * v, q.rotate(v), tolerance);
    }
    // R is orthonormal: R R^T = I.
    const Mat3<T> rrt = r * r.transpose();
    for (std::size_t i = 0; i < 3; ++i) {
      for (std::size_t j = 0; j < 3; ++j) {
        EXPECT_NEAR(rrt(i, j), i == j ? T(1) : T(0), tolerance);
      }
    }
  }
  // Closed form: +90 deg yaw is R = [[0,-1,0],[1,0,0],[0,0,1]].
  const Mat3<T> ry = this->yaw(this->kHalfPi).to_rotation_matrix();
  const T want[9] = {T(0), T(-1), T(0), T(1), T(0), T(0), T(0), T(0), T(1)};
  for (std::size_t k = 0; k < 9; ++k) {
    EXPECT_NEAR(ry.e[k], want[k], tolerance) << "element " << k;
  }
}

TYPED_TEST(QuatTest, ConjugateInverseAndNormalize) {
  using T = TypeParam;
  const T tolerance = tol<T>(kOpsChain);
  const Quat<T> q = this->generic(T(1));
  expect_quat_near(q * q.conjugate(), Quat<T>(), tolerance);
  expect_quat_near(q.conjugate() * q, Quat<T>(), tolerance);
  EXPECT_NEAR(q.norm(), T(1), tolerance);

  const Quat<T> c = Quat<T>(T(1), T(2), T(3), T(4)).conjugate();
  EXPECT_EQ(c.w, T(1));
  EXPECT_EQ(c.x, T(-2));
  EXPECT_EQ(c.y, T(-3));
  EXPECT_EQ(c.z, T(-4));

  const Quat<T> raw(T(1), T(2), T(3), T(4));
  EXPECT_NEAR(raw.norm2(), T(30), tol<T>(kOpsShort));
  const Quat<T> n = raw.normalized();
  EXPECT_NEAR(n.norm(), T(1), tolerance);
  // Direction preserved: n is raw scaled by 1/sqrt(30).
  const T inv = T(1) / raw.norm();
  expect_quat_near(n, Quat<T>(raw.w * inv, raw.x * inv, raw.y * inv, raw.z * inv), tolerance);
}

TYPED_TEST(QuatTest, AxisAngleZeroAndOpposite) {
  using T = TypeParam;
  const T tolerance = tol<T>(kOpsChain);
  expect_quat_near(this->yaw(T(0)), Quat<T>(), tolerance);
  // The opposite angle about the same axis is the conjugate.
  expect_quat_near(this->generic(T(-1)), this->generic(T(1)).conjugate(), tolerance);
  // Half-angle form: +90 deg about z is (sqrt(1/2), 0, 0, sqrt(1/2)).
  const Quat<T> qy = this->yaw(this->kHalfPi);
  EXPECT_NEAR(qy.w, qy.z, tolerance);
  EXPECT_NEAR(qy.w * qy.w, T(0.5), tolerance);
  EXPECT_EQ(qy.x, T(0));
  EXPECT_EQ(qy.y, T(0));
}

TYPED_TEST(QuatTest, CanonicalSign) {
  using T = TypeParam;
  // w > 0: unchanged.
  Quat<T> c = Quat<T>(T(0.5), T(-0.5), T(0.5), T(-0.5)).canonical();
  EXPECT_EQ(c.w, T(0.5));
  EXPECT_EQ(c.x, T(-0.5));
  EXPECT_EQ(c.y, T(0.5));
  EXPECT_EQ(c.z, T(-0.5));
  // w < 0: negated.
  c = Quat<T>(T(-0.5), T(-0.5), T(0.5), T(-0.5)).canonical();
  EXPECT_EQ(c.w, T(0.5));
  EXPECT_EQ(c.x, T(0.5));
  EXPECT_EQ(c.y, T(-0.5));
  EXPECT_EQ(c.z, T(0.5));
  // w = 0, x > 0: unchanged, x < 0: negated.
  c = Quat<T>(T(0), T(0.5), T(-0.5), T(0.5)).canonical();
  EXPECT_EQ(c.x, T(0.5));
  EXPECT_EQ(c.y, T(-0.5));
  EXPECT_EQ(c.z, T(0.5));
  c = Quat<T>(T(0), T(-0.5), T(0.5), T(-0.5)).canonical();
  EXPECT_EQ(c.x, T(0.5));
  EXPECT_EQ(c.y, T(-0.5));
  EXPECT_EQ(c.z, T(0.5));
  // w = 0, x = 0, y < 0: negated; y > 0: unchanged.
  c = Quat<T>(T(0), T(0), T(-0.6), T(0.8)).canonical();
  EXPECT_EQ(c.w, T(0));
  EXPECT_EQ(c.x, T(0));
  EXPECT_EQ(c.y, T(0.6));
  EXPECT_EQ(c.z, T(-0.8));
  c = Quat<T>(T(0), T(0), T(-1), T(0)).canonical();
  EXPECT_EQ(c.y, T(1));
  c = Quat<T>(T(0), T(0), T(0.6), T(-0.8)).canonical();
  EXPECT_EQ(c.y, T(0.6));
  EXPECT_EQ(c.z, T(-0.8));
  // w = 0, x = 0, y = 0: the sign of z decides.
  c = Quat<T>(T(0), T(0), T(0), T(-1)).canonical();
  EXPECT_EQ(c.z, T(1));
  c = Quat<T>(T(0), T(0), T(0), T(1)).canonical();
  EXPECT_EQ(c.z, T(1));

  // q and -q are the same rotation and canonicalize to the same quaternion.
  const Quat<T> q = this->generic(T(1));
  const Quat<T> nq(-q.w, -q.x, -q.y, -q.z);
  const Quat<T> cq = q.canonical();
  const Quat<T> cn = nq.canonical();
  EXPECT_EQ(cq.w, cn.w);
  EXPECT_EQ(cq.x, cn.x);
  EXPECT_EQ(cq.y, cn.y);
  EXPECT_EQ(cq.z, cn.z);
  EXPECT_GE(cq.w, T(0));
  // A rotation by more than 180 deg has w < 0; canonical flips it without changing the rotation.
  const Quat<T> big = this->generic(T(4));
  EXPECT_LT(big.w, T(0));
  EXPECT_GT(big.canonical().w, T(0));
  const Vec3<T> v(T(0.5), T(-1), T(0.25));
  expect_vec_near(big.canonical().rotate(v), big.rotate(v), tol<T>(kOpsChain));
}

}  // namespace
}  // namespace marv::prim
