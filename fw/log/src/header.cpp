#include "marv/log/header.hpp"

#include <algorithm>
#include <cstring>

namespace marv::log {
namespace {

// One header record being built: the lead, the body, zero-terminated strings; never longer than one sector.
class Builder {
 public:
  void begin(Kind kind, std::uint8_t sub, std::size_t body_end) noexcept {
    buf_.fill(0);
    buf_[kLeadOffKind] = static_cast<std::uint8_t>(kind);
    buf_[kLeadOffSub] = sub;
    n_ = body_end;
  }
  [[nodiscard]] std::span<std::uint8_t> bytes() noexcept { return buf_; }
  // Appends `s` and its terminator, cutting `s` if the record would exceed a sector.
  void zstring(const char* s) noexcept {
    const std::size_t room = buf_.size() - 1 - n_;
    const std::size_t len = std::min(std::strlen(s), room);
    std::copy_n(s, len, buf_.begin() + static_cast<std::ptrdiff_t>(n_));
    n_ += len;
    buf_[n_++] = 0;
  }
  [[nodiscard]] std::span<const std::uint8_t> finish() noexcept {
    put<std::uint16_t>(buf_, kLeadOffLength, static_cast<std::uint16_t>(n_));
    return std::span<const std::uint8_t>{buf_}.first(n_);
  }

 private:
  std::array<std::uint8_t, kSectorBytes> buf_{};
  std::size_t n_ = 0;
};

[[nodiscard]] bool put_info(Builder& b, const HeaderInfo& h, SectorFramer& f, ChunkSink& sink) noexcept {
  b.begin(Kind::HeaderInfo, 0, info::kBytes);
  const std::span<std::uint8_t> r = b.bytes();
  put<std::uint16_t>(r, info::kOffVersion, kFormatVersion);
  put<std::uint16_t>(r, info::kOffSector, static_cast<std::uint16_t>(kSectorBytes));
  put<std::uint64_t>(r, info::kOffStart, h.session_start_us);
  std::copy(h.image_hash.begin(), h.image_hash.end(), r.begin() + info::kOffImage);
  std::copy(h.config_hash.begin(), h.config_hash.end(), r.begin() + info::kOffConfig);
  put<std::uint64_t>(r, info::kOffSchema, h.schema_hash);
  put<std::uint32_t>(r, info::kOffTickNum, h.tick_num_us);
  put<std::uint32_t>(r, info::kOffTickDen, h.tick_den);
  put_f32(r, info::kOffOdr, h.odr_hz);
  put<std::uint32_t>(r, info::kOffBootSequence, h.boot.sequence);
  std::size_t o = info::kOffBoot;
  for (const std::uint8_t v : {h.boot.config_source, h.boot.slot, h.boot.config_status, h.boot.fault, h.boot.request,
                               h.boot.request_fault}) {
    r[o++] = v;
  }
  r[info::kOffBuildFlags] = h.build_flags;
  return f.append(b.finish(), sink);
}

[[nodiscard]] bool put_kind(Builder& b, const KindSchema& k, SectorFramer& f, ChunkSink& sink) noexcept {
  b.begin(Kind::HeaderKind, static_cast<std::uint8_t>(k.kind), header_kind::kOffName);
  put<std::uint16_t>(b.bytes(), header_kind::kOffRecordBytes, static_cast<std::uint16_t>(k.record_bytes));
  b.zstring(k.name);
  return f.append(b.finish(), sink);
}

[[nodiscard]] bool put_field(Builder& b, Kind kind, const FieldDesc& d, std::size_t base, SectorFramer& f,
                             ChunkSink& sink) noexcept {
  b.begin(Kind::HeaderField, static_cast<std::uint8_t>(kind), header_field::kOffNames);
  const std::span<std::uint8_t> r = b.bytes();
  put<std::uint16_t>(r, header_field::kOffOffset, static_cast<std::uint16_t>(base + d.offset));
  r[header_field::kOffSize] = d.size;
  r[header_field::kOffType] = static_cast<std::uint8_t>(d.type);
  b.zstring(d.name);
  b.zstring(d.unit);
  return f.append(b.finish(), sink);
}

[[nodiscard]] bool put_imu_raw(Builder& b, const ImuRawScales& sc, SectorFramer& f, ChunkSink& sink) noexcept {
  b.begin(Kind::HeaderImuRaw, 0, header_imu_raw::kBytes);
  const std::span<std::uint8_t> r = b.bytes();
  put_f32(r, header_imu_raw::kOffGyroScale, sc.gyro_rad_s_per_lsb);
  put_f32(r, header_imu_raw::kOffAccelScale, sc.accel_m_s2_per_lsb);
  for (std::size_t i = 0; i < prim::kSpatialDim; ++i) {
    r[header_imu_raw::kOffSigns + i] = static_cast<std::uint8_t>(sc.sign[i]);
  }
  return f.append(b.finish(), sink);
}

[[nodiscard]] bool put_param(Builder& b, const HeaderInfo& h, std::size_t id, SectorFramer& f, ChunkSink& sink) noexcept {
  const ParamRecord& p = h.params[id];
  b.begin(Kind::HeaderParam, static_cast<std::uint8_t>(p.value.type), header_param::kOffNames);
  const std::span<std::uint8_t> r = b.bytes();
  put<std::uint16_t>(r, header_param::kOffId, static_cast<std::uint16_t>(id));
  put<std::uint16_t>(r, header_param::kOffFlags, p.locked ? header_param::kFlagLocked : std::uint16_t{0});
  put<std::uint32_t>(r, header_param::kOffValue,
                     p.value.type == ParamType::F32 ? std::bit_cast<std::uint32_t>(p.value.f32)
                                                    : static_cast<std::uint32_t>(p.value.i32));
  b.zstring(h.param_name(id));
  b.zstring(p.unit);
  return f.append(b.finish(), sink);
}

}  // namespace

bool write_header(const HeaderInfo& info, SectorFramer& framer, ChunkSink& sink) noexcept {
  Builder b;
  if (!put_info(b, info, framer, sink)) {
    return false;
  }
  for (const KindSchema& k : record_schema()) {
    if (!put_kind(b, k, framer, sink)) {
      return false;
    }
    for (const FieldDesc& d : k.fields) {
      if (!put_field(b, k.kind, d, 0, framer, sink)) {
        return false;
      }
    }
  }
  if (info.imu_raw != nullptr && !put_imu_raw(b, *info.imu_raw, framer, sink)) {
    return false;
  }
  const TelemetryHook none{};
  const TelemetryHook& t = info.telemetry != nullptr ? *info.telemetry : none;
  b.begin(Kind::HeaderTelemetry, 0, header_telemetry::kBytes);
  put<std::uint32_t>(b.bytes(), header_telemetry::kOffVersion, t.version);
  put<std::uint32_t>(b.bytes(), header_telemetry::kOffFieldCount, static_cast<std::uint32_t>(t.fields.size()));
  if (!framer.append(b.finish(), sink)) {
    return false;
  }
  for (const FieldDesc& d : t.fields) {
    if (!put_field(b, Kind::Telemetry, d, telemetry::kOffBody, framer, sink)) {
      return false;
    }
  }
  if (info.param_name != nullptr) {
    for (std::size_t id = 0; id < info.params.size(); ++id) {
      if (!put_param(b, info, id, framer, sink)) {
        return false;
      }
    }
  }
  return framer.flush(sink);
}

}  // namespace marv::log
