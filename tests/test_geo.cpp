// LocalFrame: NED <-> geodetic round trip, and metres per degree against WGS84 at the origin.
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include <marv/fsw/geo.hpp>

using namespace marv;

static int failures = 0;
#define NEAR(a, b, tol)                                                                              \
    do {                                                                                             \
        if (std::fabs((a) - (b)) > (tol)) {                                                          \
            std::printf("FAIL %s:%d  %s = %.6g, want %.6g\n", __FILE__, __LINE__, #a, (double)(a), (double)(b)); \
            ++failures;                                                                              \
        }                                                                                            \
    } while (0)

int main() {
    LocalFrame f;
    f.set({473763880, 85477780, 408.f});  // the Gazebo world's origin
    // 1e-4 deg at 47.376 deg, 408 m: 11.11853 m north, 7.55251 m east (WGS84 radii, computed in double).
    Vec3 v = f.to_ned({473764880, 85477780, 408.f});
    NEAR(v.x, 11.11853f, 0.001f); NEAR(v.y, 0.f, 1e-6f);
    v = f.to_ned({473763880, 85478780, 410.f});
    NEAR(v.y, 7.55251f, 0.001f); NEAR(v.z, -2.f, 1e-6f);
    // Round trip within one e7 step (~1 cm).
    const GeoPoint g = f.to_geo({123.4f, -56.7f, -8.9f});
    v = f.to_ned(g);
    NEAR(v.x, 123.4f, 0.012f); NEAR(v.y, -56.7f, 0.012f); NEAR(v.z, -8.9f, 1e-4f);
    std::printf("%s (%d failures)\n", failures ? "FAIL" : "PASS", failures);
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
