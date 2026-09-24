// The wire between the PC (bridge / mission software) and the flight controller. Shared verbatim by
// the firmware and the host tools: freestanding, no heap, no exceptions.
//
// Frame on the wire:  COBS( id:u8 | body | crc16:u16le ) 0x00
//   - COBS removes every 0x00 from the frame, so 0x00 marks a frame end and a receiver resyncs on it.
//   - crc16 is CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over id and body.
//   - Every field is little-endian and written one by one, so no struct layout reaches the wire.
//
// A run starts with kReset. Lockstep: per simulation step the bridge sends kTruth (and kMission when it changes), then kSensors,
// and waits for the kActuators whose t_us echoes it before stepping the world again. kTelemetry for
// that tick arrives before it.
//
// Setup exchange (params.hpp): one reply per request is its acknowledgement. kSetupRequest and kLoadFactory -> kSetupHeader
// then kParamValue for every index (the staged setup); kSetParam -> kParamValue echoing the value held; kSetKind and
// kSaveSetup -> kSetupHeader.
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include <marv/fsw/contracts.hpp>
#include <marv/fsw/params.hpp>

namespace marv::link {

enum MsgId : std::uint8_t {
    kSensors = 0x01,    // PC -> FC: marv::SensorBus; runs one tick
    kMission = 0x02,    // PC -> FC: marv::MissionCommand; held until the next one
    kTruth = 0x03,      // PC -> FC: marv::State from the simulator; sent before kSensors, lab use only
    kReset = 0x04,      // PC -> FC: link::Reset; starts a new run from power-on state (sent first by the bridge)
    kSetPreset = 0x05,  // PC -> FC: link::SetPreset; stored (flash on the Pico), used from the next boot or kReset
    kReboot = 0x06,     // PC -> FC: link::Reboot; restarts the controller (a Pico drops off USB and re-enumerates)
    kSetupRequest = 0x07,  // PC -> FC: link::SetupRequest; answered by kSetupHeader and kParamValue per index (staged)
    kSetParam = 0x08,      // PC -> FC: link::SetParam; one staged value, answered by kParamValue
    kSetKind = 0x09,       // PC -> FC: link::SetKind; one staged family kind, answered by kSetupHeader
    kSaveSetup = 0x0A,     // PC -> FC: link::SaveSetup; stored := staged, answered by kSetupHeader
    kLoadFactory = 0x0B,   // PC -> FC: link::LoadFactory; staged := a factory setup, answered like kSetupRequest
    kActuators = 0x81,  // FC -> PC: marv::ActuatorCommand; always the last reply of a tick
    kTelemetry = 0x82,  // FC -> PC: marv::Telemetry; sent before kActuators
    kSetupHeader = 0x83,  // FC -> PC: link::SetupHeader
    kParamValue = 0x84,   // FC -> PC: link::ParamValue
};

// Body sizes, fixed per message.
inline constexpr std::size_t kSensorsBody = 8 + 1 + 24 + 8 + 12 + (4 + 4 + 4 + 12 + 1);  // 78
inline constexpr std::size_t kActuatorsBody = 8 + 4 * kMotorCount + 4 + 1;             // 29
inline constexpr std::size_t kStateBody = 8 + 12 + 12 + 16 + 12 + 1;                   // 61
inline constexpr std::size_t kReferenceBody = 1 + 12 + 12 + 12 + 4 + 4 + 16 + 4 + 4;   // 69
inline constexpr std::size_t kMissionBody = 1 + 1 + kReferenceBody;                    // 71
inline constexpr std::size_t kControlRequestBody = 12 + 12 + 4 + 4;                    // 32
inline constexpr std::size_t kTelemetryBody = 8 + kStateBody + kControlRequestBody + 1 + 1 + 12;  // 115
inline constexpr std::size_t kSetParamBody = 2 + 4;                                     // 6
inline constexpr std::size_t kSetKindBody = 1 + 1;                                      // 2
inline constexpr std::size_t kLoadFactoryBody = 1;
inline constexpr std::size_t kSetupHeaderBody = 4 + 2 + param::kFamilyCount + 4 + 4 + 4 + 1 + 1;  // 27
inline constexpr std::size_t kParamValueBody = 2 + 4;                                   // 6
inline constexpr std::size_t kMaxBody = kTelemetryBody > kSensorsBody ? kTelemetryBody : kSensorsBody;
static_assert(kSetupHeaderBody <= kMaxBody && kSetParamBody <= kMaxBody && kParamValueBody <= kMaxBody,
              "the setup messages fit the largest body");
inline constexpr std::size_t kMaxPayload = 1 + kMaxBody + 2;
// COBS adds one byte per 254 plus one; then the delimiter.
inline constexpr std::size_t kMaxFrame = kMaxPayload + kMaxPayload / 254 + 2;

// A new run: the flight software returns to its power-on state. No body.
struct Reset {};
// Selects the module preset to run from the next boot or kReset.
struct SetPreset {
    std::uint8_t id;
};
// Restart the flight controller. No body.
struct Reboot {};

// Asks for the staged setup. No body.
struct SetupRequest {};
// Stages one parameter value (index into param::Setup::values).
struct SetParam {
    std::uint16_t index;
    float value;
};
// Stages the kind of one family.
struct SetKind {
    std::uint8_t family;
    std::uint8_t kind;
};
// Stores the staged setup (refused while armed). No body.
struct SaveSetup {};
// Stages the factory setup id (presets.hpp kFactory).
struct LoadFactory {
    std::uint8_t id;
};
// The setup state: schema, staged kinds, and the CRC (param::setup_crc) of the running, staged and stored setups.
struct SetupHeader {
    std::uint32_t schema_hash;
    std::uint16_t param_count;
    std::uint8_t kind[param::kFamilyCount];
    std::uint32_t running_crc;
    std::uint32_t staged_crc;
    std::uint32_t stored_crc;
    std::uint8_t stored_valid;
    std::uint8_t armed;
};
// One staged parameter value.
struct ParamValue {
    std::uint16_t index;
    float value;
};

// ---- CRC -----------------------------------------------------------------------------------------

inline std::uint16_t crc16(const std::uint8_t* p, std::size_t n) {
    std::uint16_t crc = 0xFFFF;
    for (std::size_t i = 0; i < n; ++i) {
        crc ^= static_cast<std::uint16_t>(p[i]) << 8;
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x8000) ? static_cast<std::uint16_t>((crc << 1) ^ 0x1021) : static_cast<std::uint16_t>(crc << 1);
    }
    return crc;
}

// ---- COBS ----------------------------------------------------------------------------------------

// Encodes n bytes into out (capacity >= n + n/254 + 1). Returns the encoded length, no delimiter.
inline std::size_t cobs_encode(const std::uint8_t* in, std::size_t n, std::uint8_t* out) {
    std::size_t code_at = 0, o = 1;
    std::uint8_t code = 1;
    for (std::size_t i = 0; i < n; ++i) {
        if (in[i] == 0) {
            out[code_at] = code;
            code_at = o++;
            code = 1;
        } else {
            out[o++] = in[i];
            if (++code == 0xFF) {
                out[code_at] = code;
                code_at = o++;
                code = 1;
            }
        }
    }
    out[code_at] = code;
    return o;
}

// Decodes n bytes (no delimiter) into out. Returns the decoded length, or 0 on a malformed frame.
inline std::size_t cobs_decode(const std::uint8_t* in, std::size_t n, std::uint8_t* out, std::size_t cap) {
    std::size_t i = 0, o = 0;
    while (i < n) {
        const std::uint8_t code = in[i++];
        if (code == 0 || i + code - 1 > n) return 0;
        for (std::uint8_t k = 1; k < code; ++k) {
            if (o >= cap) return 0;
            out[o++] = in[i++];
        }
        if (code != 0xFF && i < n) {
            if (o >= cap) return 0;
            out[o++] = 0;
        }
    }
    return o;
}

// ---- field writer / reader -----------------------------------------------------------------------

struct Writer {
    std::uint8_t* p;
    std::size_t n = 0;
    void u8(std::uint8_t v) { p[n++] = v; }
    void u16(std::uint16_t v) { for (int i = 0; i < 2; ++i) u8(static_cast<std::uint8_t>(v >> (8 * i))); }
    void u32(std::uint32_t v) { for (int i = 0; i < 4; ++i) u8(static_cast<std::uint8_t>(v >> (8 * i))); }
    void u64(std::uint64_t v) { for (int i = 0; i < 8; ++i) u8(static_cast<std::uint8_t>(v >> (8 * i))); }
    void i32(std::int32_t v) { u32(static_cast<std::uint32_t>(v)); }
    void f32(float v) {
        std::uint32_t b;
        std::memcpy(&b, &v, 4);
        u32(b);
    }
    void vec3(const Vec3& v) { f32(v.x); f32(v.y); f32(v.z); }
    void quat(const Quat& q) { f32(q.w); f32(q.x); f32(q.y); f32(q.z); }
};

struct Reader {
    const std::uint8_t* p;
    std::size_t n = 0;
    std::uint8_t u8() { return p[n++]; }
    std::uint16_t u16() {
        const std::uint16_t lo = u8();
        return static_cast<std::uint16_t>(lo | (u8() << 8));
    }
    std::uint32_t u32() {
        std::uint32_t v = 0;
        for (int i = 0; i < 4; ++i) v |= static_cast<std::uint32_t>(u8()) << (8 * i);
        return v;
    }
    std::uint64_t u64() {
        std::uint64_t v = 0;
        for (int i = 0; i < 8; ++i) v |= static_cast<std::uint64_t>(u8()) << (8 * i);
        return v;
    }
    std::int32_t i32() { return static_cast<std::int32_t>(u32()); }
    float f32() {
        const std::uint32_t b = u32();
        float v;
        std::memcpy(&v, &b, 4);
        return v;
    }
    Vec3 vec3() {
        const float x = f32(), y = f32(), z = f32();
        return {x, y, z};
    }
    Quat quat() {
        const float w = f32(), x = f32(), y = f32(), z = f32();
        return {w, x, y, z};
    }
};

// ---- message bodies ------------------------------------------------------------------------------

inline void put(Writer& w, const SensorBus& s) {
    w.u64(s.t_us);
    w.u8(s.fresh);
    w.vec3(s.imu.accel_frd);
    w.vec3(s.imu.gyro_frd);
    w.f32(s.baro.pressure_pa);
    w.f32(s.baro.temperature_c);
    w.vec3(s.mag.field_frd_ut);
    w.i32(s.gnss.lat_e7);
    w.i32(s.gnss.lon_e7);
    w.f32(s.gnss.alt_m);
    w.vec3(s.gnss.vel_ned);
    w.u8(s.gnss.fix ? 1 : 0);
}

inline void get(Reader& r, SensorBus& s) {
    s.t_us = r.u64();
    s.fresh = r.u8();
    s.imu.accel_frd = r.vec3();
    s.imu.gyro_frd = r.vec3();
    s.baro.pressure_pa = r.f32();
    s.baro.temperature_c = r.f32();
    s.mag.field_frd_ut = r.vec3();
    s.gnss.lat_e7 = r.i32();
    s.gnss.lon_e7 = r.i32();
    s.gnss.alt_m = r.f32();
    s.gnss.vel_ned = r.vec3();
    s.gnss.fix = r.u8() != 0;
}

inline void put(Writer& w, const ActuatorCommand& a) {
    w.u64(a.t_us);
    for (float m : a.motor) w.f32(m);
    w.f32(a.brake);
    w.u8(a.armed ? 1 : 0);
}

inline void get(Reader& r, ActuatorCommand& a) {
    a.t_us = r.u64();
    for (float& m : a.motor) m = r.f32();
    a.brake = r.f32();
    a.armed = r.u8() != 0;
}

inline void put(Writer& w, const State& s) {
    w.u64(s.t_us);
    w.vec3(s.p_ned);
    w.vec3(s.v_ned);
    w.quat(s.q);
    w.vec3(s.w_frd);
    w.u8(s.valid ? 1 : 0);
}

inline void get(Reader& r, State& s) {
    s.t_us = r.u64();
    s.p_ned = r.vec3();
    s.v_ned = r.vec3();
    s.q = r.quat();
    s.w_frd = r.vec3();
    s.valid = r.u8() != 0;
}

inline void put(Writer& w, const Reference& f) {
    w.u8(f.has);
    w.vec3(f.p_ned);
    w.vec3(f.v_ned);
    w.vec3(f.a_ned);
    w.f32(f.yaw);
    w.f32(f.yaw_rate);
    w.quat(f.q);
    w.f32(f.apogee_m);
    w.f32(f.apogee_pred_m);
}

inline void get(Reader& r, Reference& f) {
    f.has = r.u8();
    f.p_ned = r.vec3();
    f.v_ned = r.vec3();
    f.a_ned = r.vec3();
    f.yaw = r.f32();
    f.yaw_rate = r.f32();
    f.q = r.quat();
    f.apogee_m = r.f32();
    f.apogee_pred_m = r.f32();
}

inline void put(Writer& w, const MissionCommand& m) {
    w.u8(static_cast<std::uint8_t>(m.mode));
    w.u8(static_cast<std::uint8_t>(m.nav));
    put(w, m.ref);
}

inline void get(Reader& r, MissionCommand& m) {
    m.mode = r.u8() == static_cast<std::uint8_t>(Mode::kFly) ? Mode::kFly : Mode::kIdle;
    m.nav = r.u8() == static_cast<std::uint8_t>(NavSource::kTruth) ? NavSource::kTruth : NavSource::kEstimate;
    get(r, m.ref);
}

inline void put(Writer& w, const Telemetry& t) {
    w.u64(t.t_us);
    put(w, t.est);
    w.vec3(t.req.thrust_ned);
    w.vec3(t.req.torque_frd);
    w.f32(t.req.thrust_hover);
    w.f32(t.req.brake);
    w.u8(t.preset);
    w.u8(t.home_valid ? 1 : 0);
    w.i32(t.home.lat_e7);
    w.i32(t.home.lon_e7);
    w.f32(t.home.alt_m);
}

inline void get(Reader& r, Telemetry& t) {
    t.t_us = r.u64();
    get(r, t.est);
    t.req.thrust_ned = r.vec3();
    t.req.torque_frd = r.vec3();
    t.req.thrust_hover = r.f32();
    t.req.brake = r.f32();
    t.preset = r.u8();
    t.home_valid = r.u8() != 0;
    t.home.lat_e7 = r.i32();
    t.home.lon_e7 = r.i32();
    t.home.alt_m = r.f32();
}

inline void put(Writer&, const Reset&) {}
inline void get(Reader&, Reset&) {}
inline void put(Writer& w, const SetPreset& s) { w.u8(s.id); }
inline void get(Reader& r, SetPreset& s) { s.id = r.u8(); }
inline void put(Writer&, const Reboot&) {}
inline void get(Reader&, Reboot&) {}
inline void put(Writer&, const SetupRequest&) {}
inline void get(Reader&, SetupRequest&) {}
inline void put(Writer& w, const SetParam& s) { w.u16(s.index); w.f32(s.value); }
inline void get(Reader& r, SetParam& s) { s.index = r.u16(); s.value = r.f32(); }
inline void put(Writer& w, const SetKind& s) { w.u8(s.family); w.u8(s.kind); }
inline void get(Reader& r, SetKind& s) { s.family = r.u8(); s.kind = r.u8(); }
inline void put(Writer&, const SaveSetup&) {}
inline void get(Reader&, SaveSetup&) {}
inline void put(Writer& w, const LoadFactory& l) { w.u8(l.id); }
inline void get(Reader& r, LoadFactory& l) { l.id = r.u8(); }

inline void put(Writer& w, const SetupHeader& h) {
    w.u32(h.schema_hash);
    w.u16(h.param_count);
    for (std::uint8_t k : h.kind) w.u8(k);
    w.u32(h.running_crc);
    w.u32(h.staged_crc);
    w.u32(h.stored_crc);
    w.u8(h.stored_valid);
    w.u8(h.armed);
}

inline void get(Reader& r, SetupHeader& h) {
    h.schema_hash = r.u32();
    h.param_count = r.u16();
    for (std::uint8_t& k : h.kind) k = r.u8();
    h.running_crc = r.u32();
    h.staged_crc = r.u32();
    h.stored_crc = r.u32();
    h.stored_valid = r.u8();
    h.armed = r.u8();
}

inline void put(Writer& w, const ParamValue& v) { w.u16(v.index); w.f32(v.value); }
inline void get(Reader& r, ParamValue& v) { v.index = r.u16(); v.value = r.f32(); }

template <class T> struct Traits;
template <> struct Traits<SensorBus> {
    static constexpr MsgId id = kSensors;
    static constexpr std::size_t body = kSensorsBody;
};
template <> struct Traits<ActuatorCommand> {
    static constexpr MsgId id = kActuators;
    static constexpr std::size_t body = kActuatorsBody;
};
template <> struct Traits<Reset> {
    static constexpr MsgId id = kReset;
    static constexpr std::size_t body = 0;
};
template <> struct Traits<SetPreset> {
    static constexpr MsgId id = kSetPreset;
    static constexpr std::size_t body = 1;
};
template <> struct Traits<Reboot> {
    static constexpr MsgId id = kReboot;
    static constexpr std::size_t body = 0;
};
template <> struct Traits<SetupRequest> {
    static constexpr MsgId id = kSetupRequest;
    static constexpr std::size_t body = 0;
};
template <> struct Traits<SetParam> {
    static constexpr MsgId id = kSetParam;
    static constexpr std::size_t body = kSetParamBody;
};
template <> struct Traits<SetKind> {
    static constexpr MsgId id = kSetKind;
    static constexpr std::size_t body = kSetKindBody;
};
template <> struct Traits<SaveSetup> {
    static constexpr MsgId id = kSaveSetup;
    static constexpr std::size_t body = 0;
};
template <> struct Traits<LoadFactory> {
    static constexpr MsgId id = kLoadFactory;
    static constexpr std::size_t body = kLoadFactoryBody;
};
template <> struct Traits<SetupHeader> {
    static constexpr MsgId id = kSetupHeader;
    static constexpr std::size_t body = kSetupHeaderBody;
};
template <> struct Traits<ParamValue> {
    static constexpr MsgId id = kParamValue;
    static constexpr std::size_t body = kParamValueBody;
};
template <> struct Traits<MissionCommand> {
    static constexpr MsgId id = kMission;
    static constexpr std::size_t body = kMissionBody;
};
template <> struct Traits<State> {
    static constexpr MsgId id = kTruth;
    static constexpr std::size_t body = kStateBody;
};
template <> struct Traits<Telemetry> {
    static constexpr MsgId id = kTelemetry;
    static constexpr std::size_t body = kTelemetryBody;
};

// Encodes msg into a complete frame, delimiter included. out needs kMaxFrame bytes. Returns its length.
template <class T> std::size_t encode(const T& msg, std::uint8_t* out) {
    std::uint8_t raw[kMaxPayload];
    Writer w{raw};
    w.u8(Traits<T>::id);
    put(w, msg);
    w.u16(crc16(raw, w.n));
    const std::size_t len = cobs_encode(raw, w.n, out);
    out[len] = 0;
    return len + 1;
}

// A decoded, CRC-checked payload: id, then the body.
struct Packet {
    std::uint8_t id;
    const std::uint8_t* body;
    std::size_t len;

    template <class T> bool as(T& msg) const {
        if (id != Traits<T>::id || len != Traits<T>::body) return false;
        Reader r{body};
        get(r, msg);
        return true;
    }
};

// Accumulates bytes from a stream and yields each valid packet. Bad frames are counted and dropped.
class Decoder {
public:
    // Feeds one byte. Returns true when it completed a valid packet, readable with packet().
    bool push(std::uint8_t b) {
        if (b != 0) {
            if (n_ < sizeof(buf_)) buf_[n_++] = b;
            else overflow_ = true;
            return false;
        }
        const std::size_t n = n_;
        const bool overflow = overflow_;
        n_ = 0;
        overflow_ = false;
        if (n == 0) return false;
        const std::size_t len = overflow ? 0 : cobs_decode(buf_, n, raw_, sizeof(raw_));
        if (len < 3 || crc16(raw_, len - 2) != static_cast<std::uint16_t>(raw_[len - 2] | (raw_[len - 1] << 8))) {
            ++errors_;
            return false;
        }
        pkt_ = {raw_[0], raw_ + 1, len - 3};
        return true;
    }
    const Packet& packet() const { return pkt_; }
    std::uint32_t errors() const { return errors_; }

private:
    std::uint8_t buf_[kMaxFrame];
    std::uint8_t raw_[kMaxPayload];
    std::size_t n_ = 0;
    bool overflow_ = false;
    std::uint32_t errors_ = 0;
    Packet pkt_{};
};

}  // namespace marv::link
