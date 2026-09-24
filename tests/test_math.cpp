// Pins the conventions of math.hpp: Hamilton product, body -> NED rotation, ZYX Euler, rotation vectors.
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include <marv/fsw/math.hpp>

using namespace marv;

static int failures = 0;
#define NEAR(a, b)                                                                              \
    do {                                                                                        \
        if (std::fabs((a) - (b)) > 1e-5f) {                                                     \
            std::printf("FAIL %s:%d  %s = %g, want %g\n", __FILE__, __LINE__, #a, (double)(a), (double)(b)); \
            ++failures;                                                                         \
        }                                                                                       \
    } while (0)

int main() {
    const float h = 1.5707963f;
    // Yaw +90 deg: the nose (body x) points east.
    Vec3 v = rotate(quat_from_euler(0.f, 0.f, h), {1.f, 0.f, 0.f});
    NEAR(v.x, 0.f); NEAR(v.y, 1.f); NEAR(v.z, 0.f);
    // Pitch +90 deg: the nose points up (-down).
    v = rotate(quat_from_euler(0.f, h, 0.f), {1.f, 0.f, 0.f});
    NEAR(v.x, 0.f); NEAR(v.z, -1.f);
    // Roll +90 deg: the right wing (body y) points down.
    v = rotate(quat_from_euler(h, 0.f, 0.f), {0.f, 1.f, 0.f});
    NEAR(v.y, 0.f); NEAR(v.z, 1.f);
    // ZYX order: yaw applied last (outermost). q = qz * qy * qx.
    const Quat q = quat_from_euler(0.3f, -0.2f, 1.1f);
    const Quat qc = quat_from_rotvec({0.f, 0.f, 1.1f}) * quat_from_rotvec({0.f, -0.2f, 0.f}) * quat_from_rotvec({0.3f, 0.f, 0.f});
    NEAR(q.w, qc.w); NEAR(q.x, qc.x); NEAR(q.y, qc.y); NEAR(q.z, qc.z);
    NEAR(yaw_of(q), 1.1f);
    // rotate_inv undoes rotate; rotation vector round trip; angle between attitudes.
    v = rotate_inv(q, rotate(q, {0.1f, -2.f, 3.f}));
    NEAR(v.x, 0.1f); NEAR(v.y, -2.f); NEAR(v.z, 3.f);
    const Vec3 r = rotvec_from_quat(quat_from_rotvec({0.4f, -0.5f, 0.2f}));
    NEAR(r.x, 0.4f); NEAR(r.y, -0.5f); NEAR(r.z, 0.2f);
    NEAR(quat_angle(q, q * quat_from_rotvec({0.f, 0.05f, 0.f})), 0.05f);
    std::printf("%s (%d failures)\n", failures ? "FAIL" : "PASS", failures);
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
