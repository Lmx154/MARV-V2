// The wire between the PC (bridge / mission software) and the flight controller. Shared verbatim by
// the firmware and the host tools: freestanding, no heap, no exceptions.
//
// Frame on the wire:  COBS( id:u8 | body | crc16:u16le ) 0x00
//   - COBS removes every 0x00 from the frame, so 0x00 marks a frame end and a receiver resyncs on it.
//   - crc16 is CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over id and body.
//   - Every field is little-endian and written one by one, so no struct layout reaches the wire.
//
// Lockstep: the bridge sends one kSensors per simulation step and waits for the kActuators whose t_us
// echoes it before stepping the world again.
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include <marv/fsw/contracts.hpp>

namespace marv::link {

enum MsgId : std::uint8_t {
    kSensors = 0x01,    // PC -> FC: marv::SensorBus
    kActuators = 0x81,  // FC -> PC: marv::ActuatorCommand
};

// Body sizes, fixed per message.
inline constexpr std::size_t kSensorsBody = 8 + 1 + 24 + 8 + 12 + (4 + 4 + 4 + 12 + 1);  // 78
inline constexpr std::size_t kActuatorsBody = 8 + 4 * kMotorCount + 1;                 // 25
inline constexpr std::size_t kMaxPayload = 1 + kSensorsBody + 2;
// COBS adds one byte per 254 plus one; then the delimiter.
inline constexpr std::size_t kMaxFrame = kMaxPayload + kMaxPayload / 254 + 2;

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
};

struct Reader {
    const std::uint8_t* p;
    std::size_t n = 0;
    std::uint8_t u8() { return p[n++]; }
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
    w.u8(a.armed ? 1 : 0);
}

inline void get(Reader& r, ActuatorCommand& a) {
    a.t_us = r.u64();
    for (float& m : a.motor) m = r.f32();
    a.armed = r.u8() != 0;
}

template <class T> struct Traits;
template <> struct Traits<SensorBus> {
    static constexpr MsgId id = kSensors;
    static constexpr std::size_t body = kSensorsBody;
};
template <> struct Traits<ActuatorCommand> {
    static constexpr MsgId id = kActuators;
    static constexpr std::size_t body = kActuatorsBody;
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
