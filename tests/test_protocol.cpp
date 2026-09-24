// Round trip of every message through encode -> byte stream -> Decoder, plus corruption and resync.
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <marv/link/protocol.hpp>

using namespace marv;

static int failures = 0;
#define CHECK(c)                                                         \
    do {                                                                 \
        if (!(c)) {                                                      \
            std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c);     \
            ++failures;                                                  \
        }                                                                \
    } while (0)

static SensorBus sample_bus() {
    SensorBus s{};
    s.t_us = 0x0102030405060708ull;
    s.fresh = kImu | kGnss;
    s.imu = {{0.f, 0.1f, -9.8066f}, {0.01f, -0.02f, 0.f}};  // zeros exercise COBS
    s.baro = {96500.f, 15.f};
    s.mag = {{21.5f, 0.3f, 43.f}};
    s.gnss = {473763880, 85477780, 408.f, {0.f, 0.f, -0.5f}, true};
    return s;
}

static bool decode_all(const std::uint8_t* p, std::size_t n, link::Decoder& d, link::Packet& out) {
    bool got = false;
    for (std::size_t i = 0; i < n; ++i)
        if (d.push(p[i])) {
            out = d.packet();
            got = true;
        }
    return got;
}

int main() {
    std::uint8_t frame[link::kMaxFrame];

    // SensorBus round trip, bit-exact.
    {
        const SensorBus in = sample_bus();
        const std::size_t n = link::encode(in, frame);
        CHECK(n <= link::kMaxFrame);
        CHECK(frame[n - 1] == 0);
        for (std::size_t i = 0; i + 1 < n; ++i) CHECK(frame[i] != 0);
        link::Decoder d;
        link::Packet p{};
        CHECK(decode_all(frame, n, d, p));
        SensorBus out{};
        CHECK(p.as(out));
        CHECK(out.t_us == in.t_us && out.fresh == in.fresh);
        CHECK(std::memcmp(&out.imu, &in.imu, sizeof in.imu) == 0);
        CHECK(out.baro.pressure_pa == in.baro.pressure_pa && out.mag.field_frd_ut.z == in.mag.field_frd_ut.z);
        CHECK(out.gnss.lat_e7 == in.gnss.lat_e7 && out.gnss.lon_e7 == in.gnss.lon_e7 && out.gnss.fix);
        CHECK(out.gnss.vel_ned.z == in.gnss.vel_ned.z);
        ActuatorCommand wrong{};
        CHECK(!p.as(wrong));  // an id mismatch is refused
    }

    // ActuatorCommand round trip.
    {
        const ActuatorCommand in{42, {0.f, 0.25f, 0.82f, 1.f}, true};
        const std::size_t n = link::encode(in, frame);
        link::Decoder d;
        link::Packet p{};
        CHECK(decode_all(frame, n, d, p));
        ActuatorCommand out{};
        CHECK(p.as(out));
        CHECK(out.t_us == 42 && out.armed && out.motor[2] == 0.82f && out.motor[0] == 0.f);
    }

    // MissionCommand, State (truth) and Telemetry round trips: every byte of the body comes back.
    {
        MissionCommand in{Mode::kFly, NavSource::kTruth, {kRefVel | kRefYawRate, {1.f, -2.f, -3.f}, {0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}, 1.5708f, -0.8f, {1.f, 0.f, 0.f, 0.f}}};
        link::Decoder d;
        link::Packet p{};
        CHECK(decode_all(frame, link::encode(in, frame), d, p));
        MissionCommand out{};
        CHECK(p.as(out));
        CHECK(out.mode == Mode::kFly && out.nav == NavSource::kTruth && out.ref.has == in.ref.has);
        CHECK(out.ref.p_ned.z == -3.f && out.ref.yaw == 1.5708f && out.ref.yaw_rate == -0.8f && out.ref.q.w == 1.f);

        const State st{77, {1.f, 2.f, 3.f}, {-1.f, 0.f, 0.5f}, {0.7071f, 0.f, 0.f, 0.7071f}, {0.1f, 0.2f, 0.3f}, true};
        CHECK(decode_all(frame, link::encode(st, frame), d, p));
        State so{};
        CHECK(p.id == link::kTruth && p.as(so));
        CHECK(so.t_us == 77 && so.q.z == 0.7071f && so.w_frd.z == 0.3f && so.valid);

        const Telemetry tm{78, st, {{0.f, 0.f, -14.9f}, {0.01f, -0.02f, 0.f}}, 3, true, {473763880, 85477780, 408.5f}};
        CHECK(decode_all(frame, link::encode(tm, frame), d, p));
        Telemetry to{};
        CHECK(p.as(to));
        CHECK(to.t_us == 78 && to.est.p_ned.y == 2.f && to.req.force_ned.z == -14.9f && to.req.torque_frd.y == -0.02f);
        CHECK(to.preset == 3 && to.home_valid && to.home.lat_e7 == 473763880 && to.home.alt_m == 408.5f);

        link::SetPreset sp{};
        CHECK(decode_all(frame, link::encode(link::SetPreset{4}, frame), d, p));
        CHECK(p.as(sp) && sp.id == 4);
        link::Reboot rb;
        CHECK(decode_all(frame, link::encode(link::Reboot{}, frame), d, p));
        CHECK(p.as(rb) && !p.as(sp));
    }

    // Reset has an empty body and is told apart from every other message by its id.
    {
        link::Decoder d;
        link::Packet p{};
        CHECK(decode_all(frame, link::encode(link::Reset{}, frame), d, p));
        link::Reset r;
        SensorBus s{};
        CHECK(p.id == link::kReset && p.len == 0 && p.as(r) && !p.as(s));
    }

    // A flipped byte is dropped and counted; the next frame still decodes (resync on 0x00).
    {
        const std::size_t n = link::encode(sample_bus(), frame);
        std::uint8_t stream[2 * link::kMaxFrame + 8];
        std::memcpy(stream, "\x11\x22\x33", 3);  // mid-frame garbage before the first delimiter
        std::memcpy(stream + 3, frame, n);
        stream[3 + 10] ^= 0x40;
        std::memcpy(stream + 3 + n, frame, n);
        link::Decoder d;
        int good = 0;
        for (std::size_t i = 0; i < 3 + 2 * n; ++i)
            if (d.push(stream[i])) ++good;
        CHECK(good == 1);
        CHECK(d.errors() == 1);
    }

    // COBS across a 254-byte run of non-zero bytes.
    {
        std::uint8_t in[600], enc[620], dec[600];
        for (int i = 0; i < 600; ++i) in[i] = static_cast<std::uint8_t>(i % 300 == 299 ? 0 : 1 + i % 250);
        const std::size_t e = link::cobs_encode(in, sizeof in, enc);
        const std::size_t n = link::cobs_decode(enc, e, dec, sizeof dec);
        CHECK(n == sizeof in && std::memcmp(in, dec, n) == 0);
    }

    std::printf("%s (%d failures)\n", failures ? "FAIL" : "PASS", failures);
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
