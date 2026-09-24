// The flight controller's packet dispatch and its setups, shared by the firmware (main.cpp) and the bridge's SITL
// endpoint (bridge/src/endpoint.cpp), so both answer the wire protocol the same way. Header-only; the platform is a
// template parameter (no virtual calls) providing
//
//   void send(const std::uint8_t* p, std::size_t n);            // one whole frame to the PC
//   std::size_t read_record(std::uint8_t* p, std::size_t cap);  // the stored record's bytes, up to cap (0 if none)
//   void write_record(const std::uint8_t* p, std::size_t n);    // replaces it (flash on the Pico, a file in SITL)
//   void reboot();                                              // restarts the controller; returns on a platform that cannot
//
// Setups (params.hpp). The running setup is the one the flight software was built from, fixed for the run. The staged
// setup is what kSetParam, kSetKind and kLoadFactory edit, each value and kind checked against its range. Setups are
// class-consistent (param::consistent): kSetKind of the vehicle re-stages every family whose kind does not serve the new
// vehicle to its first kind that does; kSetKind of a kind that does not serve the staged vehicle is refused. The stored
// setup is the record's: power-on and kReboot run it and stage it; kReset runs the staged setup (a new run); kSaveSetup
// stores the staged setup, refused while the last ActuatorCommand sent was armed (kReset and kReboot clear that: the motors are then at zero). A record is valid iff its magic,
// schema hash, length and CRC match, every kind and value is within range and the setup is class-consistent; otherwise the stored setup is factory 0
// and stored_valid is 0 (an old "MRVP" preset record among them).
//
// One reply per request is its acknowledgement: kSetupRequest and kLoadFactory -> kSetupHeader, then kParamValue for
// every index (staged); kSetParam -> kParamValue with the value held (the old one when the value is out of range; NaN
// for an index out of range); kSetKind, kSaveSetup, kSetPreset and kReset -> kSetupHeader. kReboot has none (a Pico
// drops off USB). kSetPreset N stages factory N (factory 0 for an unknown N) and saves it: used from the next kReset
// or boot, as before setups.
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <new>

#include <marv/fsw/fsw.hpp>
#include <marv/fsw/presets.hpp>
#include <marv/link/protocol.hpp>

namespace marv::fw {

// The stored record: magic "MRVS", schema hash, length of the setup bytes, their CRC-16 (link::crc16), then the setup
// bytes: kind[], then every value; each field little-endian.
inline constexpr std::uint32_t kRecordMagic = 0x5356524Du;  // "MRVS"
inline constexpr std::size_t kSetupBytes = param::kFamilyCount + 4 * param::kParamCount;
inline constexpr std::size_t kRecordHeaderBytes = 4 + 4 + 2 + 2;
inline constexpr std::size_t kRecordBytes = kRecordHeaderBytes + kSetupBytes;
inline constexpr std::uint8_t kFactoryCount = sizeof(kFactory) / sizeof(kFactory[0]);

inline void encode_record(const param::Setup& s, std::uint8_t (&out)[kRecordBytes]) {
    link::Writer body{out + kRecordHeaderBytes};
    for (std::uint8_t k : s.kind) body.u8(k);
    for (float v : s.values) body.f32(v);
    link::Writer head{out};
    head.u32(kRecordMagic);
    head.u32(param::kSchemaHash);
    head.u16(static_cast<std::uint16_t>(kSetupBytes));
    head.u16(link::crc16(out + kRecordHeaderBytes, kSetupBytes));
}

// Every kind and every value within its range (NaN is not), and every kind serving the vehicle's class.
inline bool in_range(const param::Setup& s) {
    for (std::uint8_t f = 0; f < param::kFamilyCount; ++f)
        if (s.kind[f] >= param::kind_count(f)) return false;
    for (std::uint16_t i = 0; i < param::kParamCount; ++i)
        if (!(s.values[i] >= param::kParamMeta[i].min && s.values[i] <= param::kParamMeta[i].max)) return false;
    return param::consistent(s);
}

// Decodes the n bytes of a record into out. False, and out unspecified, unless the record is valid.
inline bool decode_record(const std::uint8_t* p, std::size_t n, param::Setup& out) {
    if (n < kRecordBytes) return false;
    link::Reader r{p};
    const std::uint32_t magic = r.u32(), hash = r.u32();
    const std::uint16_t len = r.u16(), crc = r.u16();
    if (magic != kRecordMagic || hash != param::kSchemaHash || len != kSetupBytes ||
        crc != link::crc16(p + kRecordHeaderBytes, kSetupBytes))
        return false;
    for (std::uint8_t& k : out.kind) k = r.u8();
    for (float& v : out.values) v = r.f32();
    return in_range(out);
}

template <class Platform> class Node {
public:
    // Power-on: runs the stored setup and stages it.
    explicit Node(Platform& platform) : platform_(platform), fsw_(power_on()) {}

    const Fsw& fsw() const { return fsw_; }

    void dispatch(const link::Packet& pkt) {
        MissionCommand mission;
        State truth;
        SensorBus bus;
        link::SetParam set;
        link::SetKind kind;
        link::LoadFactory factory;
        link::SetPreset preset;
        link::SetupRequest request;
        link::SaveSetup store;
        link::Reset reset;
        link::Reboot reboot;
        if (pkt.as(bus)) {
            const Tick tick = fsw_.step(bus);
            armed_ = tick.act.armed;
            send(tick.tlm);
            send(tick.act);
        } else if (pkt.as(truth)) {
            fsw_.on_truth(truth);
        } else if (pkt.as(mission)) {
            fsw_.on_mission(mission);
        } else if (pkt.as(request)) {
            send_setup();
        } else if (pkt.as(set)) {
            float held = std::numeric_limits<float>::quiet_NaN();
            if (set.index < param::kParamCount) {
                const param::ParamMeta& m = param::kParamMeta[set.index];
                if (set.value >= m.min && set.value <= m.max) staged_.values[set.index] = set.value;
                held = staged_.values[set.index];
            }
            send(link::ParamValue{set.index, held});
        } else if (pkt.as(kind)) {
            if (kind.family == param::k_vehicle && kind.kind < param::kind_count(param::k_vehicle)) {
                staged_.kind[param::k_vehicle] = kind.kind;
                for (std::uint8_t f = 0; f < param::kFamilyCount; ++f)
                    if (!param::compatible(f, staged_.kind[f], kind.kind)) staged_.kind[f] = param::first_compatible(f, kind.kind);
            } else if (kind.family < param::kFamilyCount && kind.kind < param::kind_count(kind.family) &&
                       param::compatible(kind.family, kind.kind, staged_.kind[param::k_vehicle])) {
                staged_.kind[kind.family] = kind.kind;
            }
            send_header();
        } else if (pkt.as(store)) {
            save();
            send_header();
        } else if (pkt.as(factory)) {
            if (factory.id < kFactoryCount) staged_ = kFactory[factory.id];
            send_setup();
        } else if (pkt.as(preset)) {
            staged_ = kFactory[preset.id < kFactoryCount ? preset.id : 0];
            save();
            send_header();
        } else if (pkt.as(reset)) {
            // A rebuilt Fsw starts idle with the motors at zero, so nothing is armed any more.
            armed_ = false;
            restart();
            send_header();
        } else if (pkt.as(reboot)) {
            platform_.reboot();
            // It returned: power-on in place.
            armed_ = false;
            power_on();
            restart();
        }
    }

private:
    // stored := the record, factory 0 unless it is valid; staged := stored.
    const param::Setup& power_on() {
        const std::size_t n = platform_.read_record(rec_, kRecordBytes);
        stored_valid_ = decode_record(rec_, n, stored_);
        if (!stored_valid_) stored_ = kFactory[0];
        staged_ = stored_;
        return staged_;
    }

    // stored := staged, unless armed; then stored := the record read back, so the header tells whether it landed.
    void save() {
        if (armed_) return;
        encode_record(staged_, rec_);
        platform_.write_record(rec_, kRecordBytes);
        const std::size_t n = platform_.read_record(rec_, kRecordBytes);
        stored_valid_ = decode_record(rec_, n, stored_);
        if (!stored_valid_) stored_ = kFactory[0];
    }

    // A new run on the staged setup, built in place: a temporary Fsw would not fit on the Pico's stack.
    void restart() {
        fsw_.~Fsw();
        new (&fsw_) Fsw(staged_);
    }

    template <class T> void send(const T& msg) {
        std::uint8_t tx[link::kMaxFrame];
        platform_.send(tx, link::encode(msg, tx));
    }

    void send_header() {
        link::SetupHeader h{};
        h.schema_hash = param::kSchemaHash;
        h.param_count = param::kParamCount;
        std::memcpy(h.kind, staged_.kind, sizeof(h.kind));
        h.running_crc = fsw_.setup_crc();
        h.staged_crc = param::setup_crc(staged_);
        h.stored_crc = param::setup_crc(stored_);
        h.stored_valid = stored_valid_ ? 1 : 0;
        h.armed = armed_ ? 1 : 0;
        send(h);
    }

    void send_setup() {
        send_header();
        for (std::uint16_t i = 0; i < param::kParamCount; ++i) send(link::ParamValue{i, staged_.values[i]});
    }

    Platform& platform_;
    std::uint8_t rec_[kRecordBytes];
    param::Setup stored_;
    param::Setup staged_;
    bool stored_valid_ = false;
    bool armed_ = false;  // the last ActuatorCommand sent
    Fsw fsw_;
};

}  // namespace marv::fw
