// Round trip of every message through encode -> byte stream -> Decoder, plus corruption and resync.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <initializer_list>
#include <limits>

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

// The bitwise CRC-16/CCITT-FALSE the table-driven link::crc16 replaced.
static std::uint16_t crc16_bitwise(const std::uint8_t* p, std::size_t n) {
    std::uint16_t crc = 0xFFFF;
    for (std::size_t i = 0; i < n; ++i) {
        crc ^= static_cast<std::uint16_t>(p[i]) << 8;
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x8000) ? static_cast<std::uint16_t>((crc << 1) ^ 0x1021) : static_cast<std::uint16_t>(crc << 1);
    }
    return crc;
}

static_assert(link::kActuatorsBody == 29 && link::kReferenceBody == 89 && link::kMissionBody == 109 &&
                  link::kControlRequestBody == 32 && link::kTelemetryBody == 116 && link::kMaxBody == 116,
              "the bodies of the airframe-agnostic contracts");

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
        const ActuatorCommand in{42, {0.f, 0.25f, 0.82f, 1.f}, 0.37f, true};
        const std::size_t n = link::encode(in, frame);
        link::Decoder d;
        link::Packet p{};
        CHECK(decode_all(frame, n, d, p));
        ActuatorCommand out{};
        CHECK(p.as(out));
        CHECK(out.t_us == 42 && out.armed && out.motor[2] == 0.82f && out.motor[0] == 0.f && out.brake == 0.37f);
    }

    // Every mode round-trips; an unknown wire value reads as kIdle.
    {
        link::Decoder d;
        link::Packet p{};
        for (Mode m : {Mode::kIdle, Mode::kFly, Mode::kArmed}) {
            MissionCommand out{Mode::kFly, NavSource::kEstimate, {}};
            if (m == Mode::kFly) out.mode = Mode::kIdle;
            CHECK(decode_all(frame, link::encode(MissionCommand{m, NavSource::kEstimate, {}}, frame), d, p));
            CHECK(p.as(out) && out.mode == m);
        }
        CHECK(static_cast<std::uint8_t>(Mode::kArmed) == 2);
        std::uint8_t unknown = 3;
        const std::size_t n = link::encode(MissionCommand{static_cast<Mode>(unknown), NavSource::kEstimate, {}}, frame);
        MissionCommand out{Mode::kArmed, NavSource::kEstimate, {}};
        CHECK(decode_all(frame, n, d, p) && p.as(out) && out.mode == Mode::kIdle);
    }

    // MissionCommand, State (truth) and Telemetry round trips: every byte of the body comes back.
    {
        MissionCommand in{Mode::kFly, NavSource::kTruth, {kRefVel | kRefYawRate | kRefCoast | kRefApogee, {1.f, -2.f, -3.f}, {0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}, 1.5708f, -0.8f, {1.f, 0.f, 0.f, 0.f}, 284.f, 301.5f, {4.f, -5.f, -6.5f}, 7.25f, 2.f}, 3, 1, {0.5f, -0.25f, 1.f, -1.f}};
        link::Decoder d;
        link::Packet p{};
        CHECK(decode_all(frame, link::encode(in, frame), d, p));
        MissionCommand out{};
        CHECK(p.as(out));
        CHECK(out.mode == Mode::kFly && out.nav == NavSource::kTruth && out.ref.has == in.ref.has);
        CHECK(out.ref.p_ned.z == -3.f && out.ref.yaw == 1.5708f && out.ref.yaw_rate == -0.8f && out.ref.q.w == 1.f);
        CHECK(out.ref.has == (kRefVel | kRefYawRate | kRefCoast | kRefApogee) && out.ref.apogee_m == 284.f &&
              out.ref.apogee_pred_m == 301.5f);
        CHECK(out.ref.p_next_ned.x == 4.f && out.ref.p_next_ned.y == -5.f && out.ref.p_next_ned.z == -6.5f &&
              out.ref.speed_mps == 7.25f && out.ref.accept_m == 2.f);
        CHECK(p.len == link::kMissionBody && std::memcmp(&out.ref.p_next_ned, &in.ref.p_next_ned, sizeof(Vec3)) == 0);
        CHECK(out.profile == 3 && out.manual == 1 && std::memcmp(&out.sticks, &in.sticks, sizeof(Sticks)) == 0);

        // A sender that sets none of profile, manual and sticks sends hold, auto and centred sticks.
        const MissionCommand silent{Mode::kFly, NavSource::kEstimate, {}};
        CHECK(silent.profile == param::k_profile_hold && silent.manual == 0 && silent.sticks.fwd == 0.f &&
              silent.sticks.right == 0.f && silent.sticks.up == 0.f && silent.sticks.yaw == 0.f);

        // Decode rules: a profile >= kProfileCount reads as hold, a manual other than 1 as auto (0), each stick clamped
        // to -1..1 and NaN read as 0.
        const float nan = std::numeric_limits<float>::quiet_NaN(), inf = std::numeric_limits<float>::infinity();
        for (std::uint8_t bad : {std::uint8_t{4}, std::uint8_t{0xFF}}) {
            MissionCommand odd = in;
            odd.profile = bad;
            odd.manual = static_cast<std::uint8_t>(bad - 2);
            odd.sticks = {1.5f, -7.f, nan, inf};
            MissionCommand got = in;
            CHECK(decode_all(frame, link::encode(odd, frame), d, p) && p.as(got));
            CHECK(got.profile == param::k_profile_hold && got.manual == 0);
            CHECK(got.sticks.fwd == 1.f && got.sticks.right == -1.f && got.sticks.up == 0.f && got.sticks.yaw == 1.f);
            odd.sticks = {-inf, -nan, -1.f, 1.f};
            CHECK(decode_all(frame, link::encode(odd, frame), d, p) && p.as(got));
            CHECK(got.sticks.fwd == -1.f && got.sticks.right == 0.f && got.sticks.up == -1.f && got.sticks.yaw == 1.f);
        }
        for (std::uint8_t pr = 0; pr < param::kProfileCount; ++pr) {
            MissionCommand one = in, got{};
            one.profile = pr;
            one.manual = 0;
            CHECK(decode_all(frame, link::encode(one, frame), d, p) && p.as(got) && got.profile == pr && got.manual == 0);
        }

        // A 91-byte MissionCommand (before profile, manual and sticks) is rejected.
        {
            std::uint8_t raw91[1 + 91 + 2] = {link::kMission, static_cast<std::uint8_t>(Mode::kFly)};
            const std::uint16_t c91 = link::crc16(raw91, 1 + 91);
            raw91[1 + 91] = static_cast<std::uint8_t>(c91 & 0xFF);
            raw91[1 + 91 + 1] = static_cast<std::uint8_t>(c91 >> 8);
            std::uint8_t enc[link::kMaxFrame];
            const std::size_t n91 = link::cobs_encode(raw91, sizeof raw91, enc);
            enc[n91] = 0;
            MissionCommand kept{Mode::kArmed, NavSource::kEstimate, {}};
            CHECK(decode_all(enc, n91 + 1, d, p) && p.id == link::kMission && p.len == 91);
            CHECK(!p.as(kept) && kept.mode == Mode::kArmed);
        }

        // An older 69-byte Reference body (71-byte MissionCommand, before p_next_ned, speed_mps and accept_m) is rejected.
        std::uint8_t raw[1 + 71 + 2] = {link::kMission, static_cast<std::uint8_t>(Mode::kFly)};
        const std::uint16_t crc = link::crc16(raw, 1 + 71);
        raw[1 + 71] = static_cast<std::uint8_t>(crc & 0xFF);
        raw[1 + 71 + 1] = static_cast<std::uint8_t>(crc >> 8);
        std::uint8_t old[link::kMaxFrame];
        const std::size_t n = link::cobs_encode(raw, sizeof raw, old);
        old[n] = 0;
        MissionCommand stale{Mode::kArmed, NavSource::kEstimate, {}};
        CHECK(decode_all(old, n + 1, d, p) && p.id == link::kMission && p.len == 71);
        CHECK(!p.as(stale) && stale.mode == Mode::kArmed);

        const State st{77, {1.f, 2.f, 3.f}, {-1.f, 0.f, 0.5f}, {0.7071f, 0.f, 0.f, 0.7071f}, {0.1f, 0.2f, 0.3f}, true};
        CHECK(decode_all(frame, link::encode(st, frame), d, p));
        State so{};
        CHECK(p.id == link::kTruth && p.as(so));
        CHECK(so.t_us == 77 && so.q.z == 0.7071f && so.w_frd.z == 0.3f && so.valid);

        const Telemetry tm{78, st, {{0.f, 0.f, -0.681f}, {0.01f, -0.02f, 0.f}, 0.6811f, 0.25f}, 3, true, {473763880, 85477780, 408.5f}, 2};
        CHECK(decode_all(frame, link::encode(tm, frame), d, p));
        Telemetry to{};
        CHECK(p.as(to));
        CHECK(to.t_us == 78 && to.est.p_ned.y == 2.f && to.req.thrust_ned.z == -0.681f && to.req.torque_frd.y == -0.02f);
        CHECK(to.req.thrust_hover == 0.6811f && to.req.brake == 0.25f);
        CHECK(to.preset == 3 && to.home_valid && to.home.lat_e7 == 473763880 && to.home.alt_m == 408.5f);
        CHECK(to.profile == 2 && p.len == link::kTelemetryBody);

        link::SetPreset sp{};
        CHECK(decode_all(frame, link::encode(link::SetPreset{4}, frame), d, p));
        CHECK(p.as(sp) && sp.id == 4);
        link::Reboot rb;
        CHECK(decode_all(frame, link::encode(link::Reboot{}, frame), d, p));
        CHECK(p.as(rb) && !p.as(sp));
    }

    // The setup messages: each round-trips with its id and its fixed body length.
    {
        link::Decoder d;
        link::Packet p{};
        link::SetupRequest rq;
        CHECK(decode_all(frame, link::encode(link::SetupRequest{}, frame), d, p));
        CHECK(p.id == link::kSetupRequest && p.len == 0 && p.as(rq));

        link::SetParam sp{};
        CHECK(decode_all(frame, link::encode(link::SetParam{0x0102, -0.0347563f}, frame), d, p));
        CHECK(p.id == link::kSetParam && p.len == 6 && p.as(sp) && sp.index == 0x0102 && sp.value == -0.0347563f);

        link::SetKind sk{};
        CHECK(decode_all(frame, link::encode(link::SetKind{2, 3}, frame), d, p));
        CHECK(p.id == link::kSetKind && p.len == 2 && p.as(sk) && sk.family == 2 && sk.kind == 3);

        link::SaveSetup sv;
        CHECK(decode_all(frame, link::encode(link::SaveSetup{}, frame), d, p));
        CHECK(p.id == link::kSaveSetup && p.len == 0 && p.as(sv) && !p.as(rq));

        link::LoadFactory lf{};
        CHECK(decode_all(frame, link::encode(link::LoadFactory{3}, frame), d, p));
        CHECK(p.id == link::kLoadFactory && p.len == 1 && p.as(lf) && lf.id == 3);

        const link::SetupHeader hi{0xA1B2C3D4u, 80, {0, 0, 2, 0, 0, 0, 0}, 0x01020304u, 0xFFFFFFFFu, 0u, 1, 0};
        link::SetupHeader ho{};
        CHECK(decode_all(frame, link::encode(hi, frame), d, p));
        CHECK(p.id == link::kSetupHeader && p.len == 27 && p.as(ho));
        CHECK(ho.schema_hash == hi.schema_hash && ho.param_count == 80 && std::memcmp(ho.kind, hi.kind, sizeof hi.kind) == 0);
        CHECK(ho.running_crc == hi.running_crc && ho.staged_crc == hi.staged_crc && ho.stored_crc == 0u);
        CHECK(ho.stored_valid == 1 && ho.armed == 0);

        link::ParamValue pv{};
        CHECK(decode_all(frame, link::encode(link::ParamValue{79, 8.54858e-06f}, frame), d, p));
        CHECK(p.id == link::kParamValue && p.len == 6 && p.as(pv) && pv.index == 79 && pv.value == 8.54858e-06f);
        CHECK(!p.as(sp));  // same layout as SetParam, told apart by its id
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

    // The table CRC equals the bitwise one: the check value, and random buffers of every length up to kMaxFrame.
    {
        const std::uint8_t check[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
        CHECK(link::crc16(check, sizeof check) == 0x29B1 && crc16_bitwise(check, sizeof check) == 0x29B1);
        CHECK(link::crc16(check, 0) == 0xFFFF);
        std::uint32_t x = 0x12345678u;
        std::uint8_t buf[link::kMaxFrame];
        int differ = 0;
        for (int k = 0; k < 20000; ++k) {
            const std::size_t n = static_cast<std::size_t>(k) % (sizeof buf + 1);
            for (std::size_t i = 0; i < n; ++i) {
                x ^= x << 13;
                x ^= x >> 17;
                x ^= x << 5;
                buf[i] = static_cast<std::uint8_t>(x);
            }
            if (link::crc16(buf, n) != crc16_bitwise(buf, n)) ++differ;
        }
        CHECK(differ == 0);
    }

    std::printf("%s (%d failures)\n", failures ? "FAIL" : "PASS", failures);
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
