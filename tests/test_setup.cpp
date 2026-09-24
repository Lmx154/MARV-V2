// Setups through the shared packet dispatch (firmware/src/dispatch.hpp) on a fake platform: power-on and kReboot run
// the stored setup and stage it, kReset runs the staged one and kSaveSetup stores it, both unless the last
// ActuatorCommand was armed; a record that is corrupt, of another schema, out of range, short or the old preset record
// reads as factory 0 with stored_valid 0; kSetParam holds only values within range and echoes what it holds; each
// request gets exactly one reply. And a changed gain changes what the flight software commands. Setups stay
// class-consistent: kSetKind of the vehicle re-stages the families whose kind does not serve it, a kind that does not
// serve the staged vehicle is refused, an inconsistent record reads as factory 0, and a rocket setup never drives a motor.
// The guidance kind is dispatched: the trajectory kind flies a far position reference from the vehicle, at rest.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include <marv/fsw/fsw.hpp>
#include <marv/fsw/math.hpp>
#include <marv/fsw/presets.hpp>
#include <marv/link/protocol.hpp>

#include "../firmware/src/dispatch.hpp"

using namespace marv;

static int failures = 0;
#define CHECK(c)                                                     \
    do {                                                             \
        if (!(c)) {                                                  \
            std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); \
            ++failures;                                              \
        }                                                            \
    } while (0)

namespace {

struct FakePlatform {
    std::vector<std::uint8_t> out;     // frames sent to the PC
    std::vector<std::uint8_t> record;  // the flash
    int writes = 0;
    int reboots = 0;
    void send(const std::uint8_t* p, std::size_t n) { out.insert(out.end(), p, p + n); }
    std::size_t read_record(std::uint8_t* p, std::size_t cap) {
        const std::size_t n = record.size() < cap ? record.size() : cap;
        std::memcpy(p, record.data(), n);
        return n;
    }
    void write_record(const std::uint8_t* p, std::size_t n) {
        record.assign(p, p + n);
        ++writes;
    }
    void reboot() { ++reboots; }
};
using Node = fw::Node<FakePlatform>;

// Every frame one request produced.
struct Replies {
    std::vector<link::SetupHeader> headers;
    std::vector<link::ParamValue> values;
    int ticks = 0;  // kTelemetry and kActuators
    int other = 0;
    Telemetry tlm{};
    ActuatorCommand act{};
    std::size_t frames() const { return headers.size() + values.size() + static_cast<std::size_t>(ticks + other); }
};

template <class T> Replies ask(Node& node, FakePlatform& pf, const T& msg) {
    pf.out.clear();
    std::uint8_t frame[link::kMaxFrame];
    const std::size_t n = link::encode(msg, frame);
    link::Decoder dec;
    for (std::size_t i = 0; i < n; ++i)
        if (dec.push(frame[i])) node.dispatch(dec.packet());
    Replies r;
    link::Decoder rx;
    for (std::uint8_t b : pf.out) {
        if (!rx.push(b)) continue;
        link::SetupHeader h;
        link::ParamValue v;
        if (rx.packet().as(h)) r.headers.push_back(h);
        else if (rx.packet().as(v)) r.values.push_back(v);
        else if (rx.packet().as(r.tlm) || rx.packet().as(r.act)) ++r.ticks;
        else ++r.other;
    }
    return r;
}

// The header of a setup request, whose values must be the staged setup's.
link::SetupHeader header(Node& node, FakePlatform& pf, const param::Setup* staged = nullptr) {
    const Replies r = ask(node, pf, link::SetupRequest{});
    CHECK(r.headers.size() == 1 && r.values.size() == param::kParamCount && r.frames() == 1 + param::kParamCount);
    if (staged) {
        for (std::size_t i = 0; i < r.values.size(); ++i)
            CHECK(r.values[i].index == i && std::memcmp(&r.values[i].value, &staged->values[i], 4) == 0);
        CHECK(!r.headers.empty() && std::memcmp(r.headers[0].kind, staged->kind, sizeof(staged->kind)) == 0);
    }
    return r.headers.empty() ? link::SetupHeader{} : r.headers[0];
}

std::uint32_t crc(const param::Setup& s) { return param::setup_crc(s); }

std::vector<std::uint8_t> record_of(const param::Setup& s) {
    std::uint8_t b[fw::kRecordBytes];
    fw::encode_record(s, b);
    return {b, b + sizeof(b)};
}

// The CRC of a record's setup bytes, recomputed after an edit.
void reseal(std::vector<std::uint8_t>& r) {
    const std::uint16_t c = link::crc16(r.data() + fw::kRecordHeaderBytes, fw::kSetupBytes);
    r[10] = static_cast<std::uint8_t>(c);
    r[11] = static_cast<std::uint8_t>(c >> 8);
}

void put_f32(std::vector<std::uint8_t>& r, std::size_t at, float v) {
    std::uint32_t b;
    std::memcpy(&b, &v, 4);
    for (int i = 0; i < 4; ++i) r[at + static_cast<std::size_t>(i)] = static_cast<std::uint8_t>(b >> (8 * i));
}

constexpr std::size_t value_at(std::uint16_t index) { return fw::kRecordHeaderBytes + param::kFamilyCount + 4u * index; }

constexpr std::uint16_t kVelMax = param::k_controller_cascaded_pid_vel_max;

// A mission flown on truth: hold 2 m up, 10 m north of the vehicle at rest (level, at the origin, 2 m up); a single
// target, p_next = p.
const MissionCommand kFlyNorth = [] {
    MissionCommand m{Mode::kFly, NavSource::kTruth, {}};
    m.ref.has = kRefPos;
    m.ref.p_ned = {10.f, 0.f, -2.f};
    m.ref.p_next_ned = m.ref.p_ned;
    m.ref.q = kQuatIdentity;
    return m;
}();

State truth_at(std::uint64_t t_us) {
    State s{};
    s.t_us = t_us;
    s.p_ned = {0.f, 0.f, -2.f};
    s.q = kQuatIdentity;
    s.valid = true;
    return s;
}

SensorBus bus_at(std::uint64_t t_us) {
    SensorBus b{};
    b.t_us = t_us;
    return b;
}

// One tick through the node: truth, then the sensors.
Replies tick(Node& node, FakePlatform& pf, std::uint64_t t_us) {
    ask(node, pf, truth_at(t_us));
    return ask(node, pf, bus_at(t_us));
}

}  // namespace

int main() {
    const std::uint32_t crc0 = crc(kFactory[0]);

    // 1. Power-on with nothing stored: factory 0 runs, staged and stored, stored_valid 0. A setup request is the
    // header and every staged value, in index order.
    {
        FakePlatform pf;
        Node node{pf};
        const link::SetupHeader h = header(node, pf, &kFactory[0]);
        CHECK(h.schema_hash == param::kSchemaHash && h.param_count == param::kParamCount);
        CHECK(h.running_crc == crc0 && h.staged_crc == crc0 && h.stored_crc == crc0);
        CHECK(h.stored_valid == 0 && h.armed == 0);
        CHECK(node.fsw().preset() == 0 && node.fsw().setup_crc() == crc0);
        CHECK(pf.writes == 0);
    }

    // 2. kReset runs the staged setup; the running one does not change before it. Telemetry.preset is the factory id
    // the running setup equals, else 0xFF.
    {
        FakePlatform pf;
        Node node{pf};
        Replies r = ask(node, pf, link::SetKind{param::k_estimator, param::k_estimator_mahony});
        CHECK(r.headers.size() == 1 && r.frames() == 1);
        CHECK(!r.headers.empty() && r.headers[0].kind[param::k_estimator] == param::k_estimator_mahony);
        CHECK(!r.headers.empty() && r.headers[0].running_crc == crc0 && r.headers[0].staged_crc == crc(kFactory[2]));
        r = ask(node, pf, link::SetParam{kVelMax, 1.5f});
        CHECK(r.values.size() == 1 && r.frames() == 1 && r.values[0].index == kVelMax && r.values[0].value == 1.5f);
        CHECK(node.fsw().setup_crc() == crc0 && node.fsw().preset() == 0);
        r = tick(node, pf, 1000);
        CHECK(r.ticks == 2 && r.tlm.preset == 0);

        param::Setup staged = kFactory[2];
        staged.values[kVelMax] = 1.5f;
        r = ask(node, pf, link::Reset{});
        CHECK(r.headers.size() == 1 && r.frames() == 1);
        CHECK(!r.headers.empty() && r.headers[0].running_crc == crc(staged) && r.headers[0].staged_crc == crc(staged));
        CHECK(!r.headers.empty() && r.headers[0].stored_crc == crc0 && r.headers[0].stored_valid == 0);
        CHECK(node.fsw().preset() == 0xFF);
        CHECK(tick(node, pf, 1000).tlm.preset == 0xFF);

        ask(node, pf, link::SetParam{kVelMax, kFactory[2].values[kVelMax]});
        ask(node, pf, link::Reset{});
        CHECK(node.fsw().preset() == 2 && tick(node, pf, 1000).tlm.preset == 2);
        CHECK(pf.writes == 0);
    }

    // 3. kSaveSetup stores the staged setup, refused while the last ActuatorCommand was armed; the header tells which.
    {
        FakePlatform pf;
        Node node{pf};
        ask(node, pf, link::LoadFactory{3});
        ask(node, pf, kFlyNorth);
        Replies r = tick(node, pf, 1000);
        CHECK(r.act.armed);
        r = ask(node, pf, link::SaveSetup{});
        CHECK(r.headers.size() == 1 && r.frames() == 1);
        CHECK(!r.headers.empty() && r.headers[0].armed == 1 && r.headers[0].stored_valid == 0 &&
              r.headers[0].stored_crc == crc0 && r.headers[0].staged_crc == crc(kFactory[3]));
        CHECK(pf.writes == 0 && pf.record.empty());
        {
            // kReset while armed is refused: the header comes back unchanged (armed, running != staged) and the run
            // keeps flying. A run cut off mid-flight is ended by a kIdle MissionCommand (the bridge sends one before
            // its kReset): kIdle clears armed, so the kReset runs the staged setup and a save is allowed.
            FakePlatform pf2;
            Node cut{pf2};
            ask(cut, pf2, link::LoadFactory{3});
            ask(cut, pf2, kFlyNorth);
            CHECK(tick(cut, pf2, 1000).act.armed);
            Replies rr = ask(cut, pf2, link::Reset{});
            CHECK(rr.headers.size() == 1 && rr.frames() == 1);
            CHECK(!rr.headers.empty() && rr.headers[0].armed == 1 && rr.headers[0].running_crc == crc0 &&
                  rr.headers[0].staged_crc == crc(kFactory[3]));
            CHECK(cut.fsw().preset() == 0 && cut.fsw().setup_crc() == crc0);
            rr = tick(cut, pf2, 2000);
            CHECK(rr.act.armed && rr.act.motor[0] > 0.f);
            rr = ask(cut, pf2, MissionCommand{Mode::kIdle, NavSource::kEstimate, {}});
            CHECK(rr.frames() == 0);
            rr = ask(cut, pf2, link::Reset{});
            CHECK(rr.headers.size() == 1 && rr.frames() == 1);
            CHECK(!rr.headers.empty() && rr.headers[0].armed == 0 && rr.headers[0].running_crc == crc(kFactory[3]));
            CHECK(cut.fsw().preset() == 3);
            rr = ask(cut, pf2, link::SaveSetup{});
            CHECK(!rr.headers.empty() && rr.headers[0].stored_valid == 1 && pf2.writes == 1);
            // A non-idle mission does not clear armed.
            ask(cut, pf2, kFlyNorth);
            CHECK(tick(cut, pf2, 1000).act.armed);
            ask(cut, pf2, MissionCommand{Mode::kArmed, NavSource::kTruth, {}});
            rr = ask(cut, pf2, link::Reset{});
            CHECK(!rr.headers.empty() && rr.headers[0].armed == 1);
        }
        r = ask(node, pf, link::SetPreset{1});  // stage + save: refused too
        CHECK(r.headers.size() == 1 && r.frames() == 1 && pf.writes == 0);
        ask(node, pf, link::LoadFactory{3});

        ask(node, pf, MissionCommand{Mode::kIdle, NavSource::kTruth, {}});
        r = tick(node, pf, 2000);
        CHECK(!r.act.armed);
        r = ask(node, pf, link::SaveSetup{});
        CHECK(r.headers.size() == 1 && r.frames() == 1);
        CHECK(!r.headers.empty() && r.headers[0].armed == 0 && r.headers[0].stored_valid == 1 &&
              r.headers[0].stored_crc == crc(kFactory[3]) && r.headers[0].staged_crc == crc(kFactory[3]));
        CHECK(pf.writes == 1 && pf.record == record_of(kFactory[3]));
        CHECK(node.fsw().preset() == 0);  // stored, not running

        // Power-on on that flash runs it.
        Node again{pf};
        const link::SetupHeader h = header(again, pf, &kFactory[3]);
        CHECK(h.stored_valid == 1 && h.running_crc == crc(kFactory[3]) && again.fsw().preset() == 3);
    }

    // 4. kReboot (on a platform where it returns) runs the stored setup and stages it; it has no reply.
    {
        FakePlatform pf;
        pf.record = record_of(kFactory[2]);
        Node node{pf};
        ask(node, pf, link::LoadFactory{1});
        ask(node, pf, link::SetParam{kVelMax, 3.f});
        ask(node, pf, link::Reset{});
        CHECK(node.fsw().preset() == 0xFF);
        const Replies r = ask(node, pf, link::Reboot{});
        CHECK(r.frames() == 0 && pf.reboots == 1);
        const link::SetupHeader h = header(node, pf, &kFactory[2]);
        CHECK(h.running_crc == crc(kFactory[2]) && h.staged_crc == crc(kFactory[2]) && h.stored_crc == crc(kFactory[2]));
        CHECK(h.stored_valid == 1 && node.fsw().preset() == 2);
    }

    // 5. Records that are not valid read as factory 0 with stored_valid 0; a valid one (the control) runs.
    {
        struct Case {
            const char* name;
            std::vector<std::uint8_t> record;
            bool valid;
        };
        std::vector<Case> cases;
        cases.push_back({"valid factory 3", record_of(kFactory[3]), true});
        cases.push_back({"none", {}, false});
        {
            std::vector<std::uint8_t> r = record_of(kFactory[3]);
            r[value_at(kVelMax)] ^= 0x01;  // CRC no longer matches
            cases.push_back({"corrupt", r, false});
        }
        {
            std::vector<std::uint8_t> r = record_of(kFactory[3]);
            r[4] ^= 0x01;  // schema hash
            cases.push_back({"hash mismatch", r, false});
        }
        {
            std::vector<std::uint8_t> r = record_of(kFactory[3]);
            put_f32(r, value_at(kVelMax), 100.f);  // max 10
            reseal(r);
            cases.push_back({"value out of range", r, false});
        }
        {
            std::vector<std::uint8_t> r = record_of(kFactory[3]);
            put_f32(r, value_at(kVelMax), std::nanf(""));
            reseal(r);
            cases.push_back({"value NaN", r, false});
        }
        {
            std::vector<std::uint8_t> r = record_of(kFactory[3]);
            r[fw::kRecordHeaderBytes + param::k_estimator] = 4;  // four estimator kinds
            reseal(r);
            cases.push_back({"kind out of range", r, false});
        }
        {
            std::vector<std::uint8_t> r = record_of(kFactory[3]);
            r.pop_back();
            cases.push_back({"short", r, false});
        }
        {
            std::vector<std::uint8_t> r = record_of(kFactory[3]);
            r[0] = 'X';
            cases.push_back({"magic", r, false});
        }
        // The old preset record of firmware/src/main.cpp: "MRVP", id 2, ~id, little-endian u32s.
        cases.push_back({"old MRVP preset record", {0x4D, 0x52, 0x56, 0x50, 2, 0, 0, 0, 0xFD, 0xFF, 0xFF, 0xFF}, false});
        {
            std::vector<std::uint8_t> r = record_of(kFactory[3]);
            r.resize(4096, 0xFF);  // a whole erased-padded sector reads the same
            cases.push_back({"padded sector", r, true});
        }
        for (const Case& c : cases) {
            FakePlatform pf;
            pf.record = c.record;
            Node node{pf};
            const link::SetupHeader h = header(node, pf, c.valid ? &kFactory[3] : &kFactory[0]);
            const std::uint32_t want = c.valid ? crc(kFactory[3]) : crc0;
            const bool ok = h.stored_valid == (c.valid ? 1 : 0) && h.running_crc == want && h.staged_crc == want &&
                            h.stored_crc == want && node.fsw().preset() == (c.valid ? 3 : 0);
            CHECK(ok);
            std::printf("record %-24s stored_valid %u running factory %u\n", c.name, static_cast<unsigned>(h.stored_valid),
                        static_cast<unsigned>(node.fsw().preset()));
        }
    }

    // 6. kSetParam holds a value only within [min, max] and echoes the value held; an index out of range gets one
    // reply with NaN. kSetKind and kLoadFactory ignore what is out of range and still answer.
    {
        FakePlatform pf;
        Node node{pf};
        const param::ParamMeta& m = param::kParamMeta[kVelMax];
        const float tries[] = {100.f, m.max + 0.001f, m.min - 0.001f, std::nanf(""), -1.f};
        for (float v : tries) {
            const Replies r = ask(node, pf, link::SetParam{kVelMax, v});
            CHECK(r.values.size() == 1 && r.frames() == 1);
            CHECK(!r.values.empty() && r.values[0].index == kVelMax && r.values[0].value == m.dflt);
        }
        Replies r = ask(node, pf, link::SetParam{kVelMax, m.max});
        CHECK(r.values.size() == 1 && r.values[0].value == m.max);
        r = ask(node, pf, link::SetParam{kVelMax, m.min});
        CHECK(r.values.size() == 1 && r.values[0].value == m.min);
        r = ask(node, pf, link::SetParam{param::kParamCount, 1.f});
        CHECK(r.values.size() == 1 && r.frames() == 1 && r.values[0].index == param::kParamCount &&
              std::isnan(r.values[0].value));
        param::Setup staged = kFactory[0];
        staged.values[kVelMax] = m.min;

        r = ask(node, pf, link::SetKind{param::k_estimator, 4});
        CHECK(r.headers.size() == 1 && r.frames() == 1 && r.headers[0].staged_crc == crc(staged));
        r = ask(node, pf, link::SetKind{param::kFamilyCount, 0});
        CHECK(r.headers.size() == 1 && r.frames() == 1 && r.headers[0].staged_crc == crc(staged));
        r = ask(node, pf, link::SetKind{param::k_vehicle, 2});  // two vehicle kinds
        CHECK(r.headers.size() == 1 && r.frames() == 1 && r.headers[0].staged_crc == crc(staged));
        r = ask(node, pf, link::LoadFactory{fw::kFactoryCount});
        CHECK(r.headers.size() == 1 && r.values.size() == param::kParamCount && r.frames() == 1 + param::kParamCount);
        CHECK(!r.headers.empty() && r.headers[0].staged_crc == crc(staged));
        header(node, pf, &staged);
    }

    // 7. Exactly one reply per request: the frames each request produces.
    {
        FakePlatform pf;
        Node node{pf};
        const std::size_t all = 1 + param::kParamCount;
        CHECK(ask(node, pf, link::SetupRequest{}).frames() == all);
        CHECK(ask(node, pf, link::SetParam{0, 1.f}).frames() == 1);
        CHECK(ask(node, pf, link::SetKind{param::k_estimator, 1}).frames() == 1);
        CHECK(ask(node, pf, link::LoadFactory{2}).frames() == all);
        CHECK(ask(node, pf, link::SaveSetup{}).frames() == 1);
        CHECK(ask(node, pf, link::SetPreset{1}).frames() == 1);
        CHECK(ask(node, pf, link::Reset{}).frames() == 1);
        CHECK(ask(node, pf, bus_at(1000)).frames() == 2);
        CHECK(ask(node, pf, link::Reboot{}).frames() == 0);
        CHECK(pf.writes == 2 && node.fsw().preset() == 1);
    }

    // 8. A changed gain changes the flight software's command: vel_max halved halves the horizontal thrust a far
    // position reference asks for; the same setup (the control) commands the same thrust, bit for bit. Passthrough
    // guidance, so the reference reaches the controller on the first tick, and vel_max 2 m/s, below pos_p * 10 m.
    {
        const auto thrust = [](const param::Setup& s) {
            Fsw fsw{s};
            fsw.on_mission(kFlyNorth);
            fsw.on_truth(truth_at(1000));
            return fsw.step(bus_at(1000)).tlm.req.thrust_ned;
        };
        param::Setup base = kFactory[0];
        base.kind[param::k_guidance] = param::k_guidance_passthrough;
        base.values[kVelMax] = 2.f;
        param::Setup slow = base;
        slow.values[kVelMax] = 1.f;
        const Vec3 f0 = thrust(base), f1 = thrust(slow), f0b = thrust(base);
        std::printf("thrust north: vel_max 2 -> %.4f, vel_max 1 -> %.4f of full\n", static_cast<double>(f0.x),
                    static_cast<double>(f1.x));
        CHECK(f0.x > 0.1f && std::fabs(f1.x - 0.5f * f0.x) < 1e-5f);
        CHECK(std::memcmp(&f0, &f0b, sizeof(Vec3)) == 0);
    }

    // 9. Class consistency. Every factory setup is consistent and in range; factory 4 is the rocket.
    {
        for (std::uint8_t i = 0; i < fw::kFactoryCount; ++i) CHECK(fw::in_range(kFactory[i]) && param::consistent(kFactory[i]));
        CHECK(kFactory[4].kind[param::k_vehicle] == param::k_vehicle_rocket);
        for (std::uint8_t i = 0; i < 4; ++i) CHECK(kFactory[i].kind[param::k_vehicle] == param::k_vehicle_uav);

        FakePlatform pf;
        Node node{pf};
        // The vehicle to rocket: every uav-only family moves to its first rocket kind, the estimator (both) stays, no
        // value changes: factory 0 so becomes factory 4.
        Replies r = ask(node, pf, link::SetKind{param::k_vehicle, param::k_vehicle_rocket});
        CHECK(r.headers.size() == 1 && r.frames() == 1);
        CHECK(!r.headers.empty() && std::memcmp(r.headers[0].kind, kFactory[4].kind, sizeof(kFactory[4].kind)) == 0 &&
              r.headers[0].staged_crc == crc(kFactory[4]));
        // A uav kind under the rocket is refused: the header echoes the kind held.
        r = ask(node, pf, link::SetKind{param::k_controller, param::k_controller_cascaded_pid});
        CHECK(r.headers.size() == 1 && r.headers[0].kind[param::k_controller] == param::k_controller_apogee_pid &&
              r.headers[0].staged_crc == crc(kFactory[4]));
        r = ask(node, pf, link::SetKind{param::k_actuators, param::k_actuators_rotor_speed_fraction});
        CHECK(r.headers.size() == 1 && r.headers[0].kind[param::k_actuators] == param::k_actuators_brake_servo);
        // A kind of both classes is taken; back to the uav, the estimator chosen stays.
        r = ask(node, pf, link::SetKind{param::k_estimator, param::k_estimator_mahony});
        CHECK(r.headers.size() == 1 && r.headers[0].kind[param::k_estimator] == param::k_estimator_mahony);
        r = ask(node, pf, link::SetKind{param::k_vehicle, param::k_vehicle_uav});
        CHECK(r.headers.size() == 1 && std::memcmp(r.headers[0].kind, kFactory[2].kind, sizeof(kFactory[2].kind)) == 0 &&
              r.headers[0].staged_crc == crc(kFactory[2]));

        // An inconsistent record (the rocket with the quad's allocation) reads as factory 0; the consistent rocket
        // record runs, as factory 4.
        param::Setup mixed = kFactory[4];
        mixed.kind[param::k_allocation] = param::k_allocation_quad_x;
        CHECK(!fw::in_range(mixed));
        pf.record = record_of(mixed);
        Node bad{pf};
        const link::SetupHeader hb = header(bad, pf, &kFactory[0]);
        CHECK(hb.stored_valid == 0 && hb.running_crc == crc0 && bad.fsw().preset() == 0);
        pf.record = record_of(kFactory[4]);
        Node rocket{pf};
        const link::SetupHeader hr = header(rocket, pf, &kFactory[4]);
        CHECK(hr.stored_valid == 1 && rocket.fsw().preset() == 4);

        // A rocket setup never drives a motor: a fly mission on valid truth leaves every motor and the brake at zero
        // (no coast bit, so no apogee demand) but counts as armed, so a flash save is refused; the uav factory arms on
        // the same ticks (the control).
        const auto fly = [](const param::Setup& s) {
            Fsw fsw{s};
            fsw.on_mission(kFlyNorth);
            ActuatorCommand a{};
            for (std::uint64_t t = 1000; t <= 100000; t += 1000) {
                fsw.on_truth(truth_at(t));
                a = fsw.step(bus_at(t)).act;
            }
            return a;
        };
        const ActuatorCommand ar = fly(kFactory[4]), au = fly(kFactory[0]);
        CHECK(ar.armed && ar.brake == 0.f);
        for (float m : ar.motor) CHECK(m == 0.f);
        CHECK(au.armed && au.brake == 0.f && au.motor[0] > 0.5f);
    }

    // kArmed: on an invalid estimate the gate keeps it idle (not armed, motors zero); on a valid navigation source
    // every motor is spin_arm and the command is armed, so a flash save is refused.
    {
        MissionCommand armed_est{Mode::kArmed, NavSource::kEstimate, {}};
        Fsw fsw{kFactory[0]};
        fsw.on_mission(armed_est);
        const ActuatorCommand a = fsw.step(bus_at(1000)).act;
        CHECK(!a.armed);
        for (float m : a.motor) CHECK(m == 0.f);

        FakePlatform pf;
        Node node{pf};
        ask(node, pf, MissionCommand{Mode::kArmed, NavSource::kTruth, {}});
        const Replies r = tick(node, pf, 1000);
        const float spin_arm = kFactory[0].values[param::k_actuators_rotor_speed_fraction_spin_arm];
        CHECK(spin_arm == 0.10f);
        CHECK(r.act.armed && r.act.brake == 0.f);
        for (float m : r.act.motor) CHECK(m == spin_arm);
        const Replies s = ask(node, pf, link::SaveSetup{});
        CHECK(s.headers.size() == 1 && s.headers[0].armed == 1 && s.headers[0].stored_valid == 0 && pf.writes == 0);
    }

    // The trajectory kind through Fsw: factory 0 (guidance trajectory) starts the reference 10 m north at the vehicle,
    // at rest, so its first tick asks for no horizontal thrust where passthrough (factory 0 with guidance passthrough,
    // the control) asks for vel_max toward it; a second on (the truth held still) it pulls north.
    {
        const param::Setup& traj = kFactory[0];
        CHECK(traj.kind[param::k_guidance] == param::k_guidance_trajectory);
        param::Setup pass = kFactory[0];
        pass.kind[param::k_guidance] = param::k_guidance_passthrough;
        CHECK(fw::in_range(pass) && param::consistent(pass));
        MissionCommand m = kFlyNorth;
        m.ref.accept_m = 0.5f;
        const auto thrust_after = [&m](const param::Setup& s, int ticks) {
            Fsw fsw{s};
            fsw.on_mission(m);
            Vec3 f{};
            for (int k = 1; k <= ticks; ++k) {
                const std::uint64_t t = 1000u * static_cast<std::uint64_t>(k);
                fsw.on_truth(truth_at(t));
                f = fsw.step(bus_at(t)).tlm.req.thrust_ned;
            }
            return f;
        };
        const Vec3 t1 = thrust_after(traj, 1), p1 = thrust_after(pass, 1), t1000 = thrust_after(traj, 1000);
        std::printf("thrust north, guidance trajectory: %.4f on the first tick, %.4f after 1 s; passthrough %.4f\n",
                    static_cast<double>(t1.x), static_cast<double>(t1000.x), static_cast<double>(p1.x));
        CHECK(t1.x == 0.f && t1.y == 0.f && p1.x > 0.1f);
        CHECK(t1000.x > 0.05f && std::fabs(t1000.y) < 1e-6f);
    }

    std::printf(failures ? "FAIL (%d)\n" : "PASS\n", failures);
    return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
