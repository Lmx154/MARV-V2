// marv_ground manual's mapping (ground/src/pilot.hpp): deadband, CH6 bands, Xbox profile buttons, CH5 arming with the
// throttle at the bottom, and the MissionCommand it builds (manual, normalized sticks, profile, still reference).
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "../ground/src/pilot.hpp"

using namespace marv;
using ground::Pilot;

static int failures = 0;
#define CHECK(c)                                                                    \
    do {                                                                            \
        if (!(c)) {                                                                 \
            std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c);                \
            ++failures;                                                             \
        }                                                                           \
    } while (0)
#define NEAR(a, b, tol) CHECK(std::fabs((a) - (b)) <= (tol))

constexpr std::uint8_t kButton = 1, kAxis = 2;

int main() {
    // Deadband: 10% of full scale reads 0, full scale reads 1, halfway through the live range reads 0.5.
    NEAR(ground::stick(0), 0.f, 0.f);
    NEAR(ground::stick(3276), 0.f, 0.f);
    NEAR(ground::stick(-3276), 0.f, 0.f);
    NEAR(ground::stick(32767), 1.f, 1e-6f);
    NEAR(ground::stick(-32767), -1.f, 1e-6f);
    NEAR(ground::stick(-32768), -1.f, 1e-6f);
    NEAR(ground::stick(18022), 0.5f, 1e-4f);  // (0.55 - 0.1) / 0.9

    // CH6 bands: edges at -16384, 0, +16384; -100/-33/+33/+100 % mid-band.
    CHECK(ground::ch6_profile(-32768) == param::k_profile_hold);
    CHECK(ground::ch6_profile(-16385) == param::k_profile_hold);
    CHECK(ground::ch6_profile(-16384) == param::k_profile_freestyle);
    CHECK(ground::ch6_profile(-10923) == param::k_profile_freestyle);
    CHECK(ground::ch6_profile(-1) == param::k_profile_freestyle);
    CHECK(ground::ch6_profile(0) == param::k_profile_stabilized);
    CHECK(ground::ch6_profile(10923) == param::k_profile_stabilized);
    CHECK(ground::ch6_profile(16383) == param::k_profile_stabilized);
    CHECK(ground::ch6_profile(16384) == param::k_profile_agile);
    CHECK(ground::ch6_profile(32767) == param::k_profile_agile);

    // RadioMaster: hold before CH6 reports; axes AETR; arming only with the throttle at the bottom as CH5 turns on.
    {
        Pilot p(true);
        p.update();
        CHECK(p.profile() == param::k_profile_hold);
        CHECK(!p.fly());
        const std::int16_t init[8] = {0, 0, 0, 0, -32767, -10923, 0, 0};  // throttle centred, CH5 off, CH6 freestyle
        for (std::uint8_t n = 0; n < 8; ++n) p.event(kAxis, n, init[n]);
        p.update();
        CHECK(p.profile() == param::k_profile_freestyle);
        p.event(kAxis, 4, 32767);  // CH5 on, throttle centred: refused
        p.update();
        CHECK(!p.fly() && p.refused());
        p.event(kAxis, 2, -32767);  // throttle to the bottom does not arm while CH5 stays on
        p.update();
        CHECK(!p.fly());
        p.event(kAxis, 4, -32767);  // CH5 off, then on at the bottom: armed
        p.update();
        CHECK(!p.fly() && !p.refused());
        p.event(kAxis, 4, 32767);
        p.update();
        CHECK(p.fly());
        p.event(kAxis, 0, 32767);   // aileron right
        p.event(kAxis, 1, 32767);   // elevator forward
        p.event(kAxis, 2, 32767);   // throttle up
        p.event(kAxis, 3, -32767);  // rudder left
        p.event(kAxis, 5, 32767);   // CH6 agile
        p.update();
        MissionCommand c = p.command();
        CHECK(c.mode == Mode::kFly);
        CHECK(c.manual == 1);
        CHECK(c.profile == param::k_profile_agile);
        NEAR(c.sticks.fwd, 1.f, 1e-6f);
        NEAR(c.sticks.right, 1.f, 1e-6f);
        NEAR(c.sticks.up, 1.f, 1e-6f);
        NEAR(c.sticks.yaw, -1.f, 1e-6f);
        // The reference is a still velocity: a flight controller that ignores the sticks holds.
        CHECK(c.ref.has == (kRefVel | kRefYawRate));
        NEAR(c.ref.v_ned.x, 0.f, 0.f);
        NEAR(c.ref.v_ned.y, 0.f, 0.f);
        NEAR(c.ref.v_ned.z, 0.f, 0.f);
        NEAR(c.ref.yaw_rate, 0.f, 0.f);
        // Link loss: centred sticks, same arm state and profile.
        c = p.centred();
        CHECK(c.mode == Mode::kFly && c.manual == 1 && c.profile == param::k_profile_agile);
        NEAR(c.sticks.fwd, 0.f, 0.f);
        NEAR(c.sticks.right, 0.f, 0.f);
        NEAR(c.sticks.up, 0.f, 0.f);
        NEAR(c.sticks.yaw, 0.f, 0.f);
        p.event(kAxis, 4, -32767);  // CH5 off: disarmed, idle
        p.update();
        CHECK(p.command().mode == Mode::kIdle);
        // Buttons do nothing on the radio.
        p.event(kButton, 0, 1);
        p.event(kButton, 2, 1);
        p.update();
        CHECK(!p.fly() && p.profile() == param::k_profile_agile);
    }

    // Xbox: A arms, B disarms, X/Y/LB/RB pick hold/stabilized/freestyle/agile; stick up is negative.
    {
        Pilot p(false);
        p.update();
        CHECK(p.profile() == param::k_profile_hold && !p.fly());
        p.event(kButton, 0, 1);
        p.update();
        CHECK(p.fly());
        p.event(kButton, 3, 1);
        CHECK(p.profile() == param::k_profile_stabilized);
        p.event(kButton, 4, 1);
        CHECK(p.profile() == param::k_profile_freestyle);
        p.event(kButton, 5, 1);
        CHECK(p.profile() == param::k_profile_agile);
        p.event(kButton, 5, 0);  // a release changes nothing
        CHECK(p.profile() == param::k_profile_agile);
        p.event(kButton, 2, 1);
        CHECK(p.profile() == param::k_profile_hold);
        p.event(kAxis, 0, 32767);   // left X: yaw clockwise
        p.event(kAxis, 1, -32767);  // left Y up: climb
        p.event(kAxis, 3, -32767);  // right X: left
        p.event(kAxis, 4, -32767);  // right Y up: forward
        p.update();
        const MissionCommand c = p.command();
        CHECK(c.mode == Mode::kFly && c.manual == 1 && c.profile == param::k_profile_hold);
        NEAR(c.sticks.yaw, 1.f, 1e-6f);
        NEAR(c.sticks.up, 1.f, 1e-6f);
        NEAR(c.sticks.right, -1.f, 1e-6f);
        NEAR(c.sticks.fwd, 1.f, 1e-6f);
        p.event(kButton, 1, 1);
        p.update();
        CHECK(!p.fly() && p.command().mode == Mode::kIdle);
    }

    std::printf("%s (%d failures)\n", failures ? "FAIL" : "PASS", failures);
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
