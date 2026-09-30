#include "lockstep_log.hpp"

#include <bit>
#include <cerrno>
#include <cstring>

static_assert(std::endian::native == std::endian::little, "the lockstep log is little-endian and written natively");

namespace marv::gz {

namespace {
constexpr char kMagic[] = {'M', 'A', 'R', 'V', 'L', 'O', 'C', 'K'};
constexpr std::uint32_t kVersion = 1;
}  // namespace

void LogBuffer::u8(std::uint8_t v) { data_.push_back(v); }
void LogBuffer::u16(std::uint16_t v) { bytes(&v, sizeof(v)); }
void LogBuffer::u32(std::uint32_t v) { bytes(&v, sizeof(v)); }
void LogBuffer::u64(std::uint64_t v) { bytes(&v, sizeof(v)); }
void LogBuffer::f64(double v) { u64(std::bit_cast<std::uint64_t>(v)); }
void LogBuffer::bytes(const void* p, std::size_t n) {
  const auto* b = static_cast<const std::uint8_t*>(p);
  data_.insert(data_.end(), b, b + n);
}

LogFile::~LogFile() {
  if (file_ != nullptr) {
    std::fclose(file_);
  }
}

bool LogFile::open(const std::string& path, std::uint32_t m, std::uint32_t num_us, std::uint32_t den,
                   std::uint64_t seed, const std::string& gz_version, std::string& error) {
  file_ = std::fopen(path.c_str(), "wb");
  if (file_ == nullptr) {
    error = "cannot create the log file: " + std::string(std::strerror(errno));
    return false;
  }
  LogBuffer tail;
  tail.u32(m);
  tail.u32(num_us);
  tail.u32(den);
  tail.u64(seed);
  tail.u32(static_cast<std::uint32_t>(gz_version.size()));
  tail.bytes(gz_version.data(), gz_version.size());
  LogBuffer b;
  b.bytes(kMagic, sizeof(kMagic));
  b.u32(kVersion);
  b.u32(static_cast<std::uint32_t>(sizeof(kMagic) + sizeof(std::uint32_t) * 2 + tail.data().size()));
  b.bytes(tail.data().data(), tail.data().size());
  if (!write(b)) {
    error = "cannot write the log header";
    return false;
  }
  return true;
}

bool LogFile::write(const LogBuffer& b) {
  if (file_ == nullptr) {
    return false;
  }
  if (std::fwrite(b.data().data(), 1, b.data().size(), file_) != b.data().size()) {
    return false;
  }
  return std::fflush(file_) == 0;
}

void LogFile::close_with_trailer(std::uint64_t steps, std::uint64_t ticks, std::uint64_t applied) {
  if (file_ == nullptr) {
    return;
  }
  LogBuffer b;
  b.u8(static_cast<std::uint8_t>(LogRecord::kTrailer));
  b.u64(steps);
  b.u64(ticks);
  b.u64(applied);
  write(b);
  std::fclose(file_);
  file_ = nullptr;
}

}  // namespace marv::gz
