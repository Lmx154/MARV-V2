// marv_validate_table <config.mcfg>: the firmware's own verdict on a config file for this image's parameter set (FC3v2), on
// the host (ground-segment unit 9, decision 0053 G7 checks 1 and 2). It runs the config store's reader, store::decode (the
// format, the payload SHA-256, this image's section and schema hash, every record: type, finiteness, invariant I1, the locks
// against the compiled values and provenance), with the composition's validator, composition::validate (the table
// init would panic on), exactly as the boot step and the staging do. The binding is reported, not checked: a candidate
// config has none yet.
//
// Output, one "key: value" per line: fault (store::fault_text), validator (the reason, or "accepted", or "not run"),
// config_hash (hex, core IF-1, once the section is found), verdict (accepted or refused). Exit 0 accepted, 1 refused,
// 2 when the file cannot be read.
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <vector>

#include <marv/composition.hpp>
#include <marv/params/param.hpp>
#include <marv/store/mcfg.hpp>

namespace {

constexpr int kExitRefused = 1;
constexpr int kExitUsage = 2;

const char* validate_table(marv::ParamTable table) noexcept { return marv::composition::validate(table).reason; }

}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) {
    std::fprintf(stderr, "usage: marv_validate_table <config.mcfg>\n");
    return kExitUsage;
  }
  std::ifstream in(argv[1], std::ios::binary);
  if (!in) {
    std::fprintf(stderr, "marv_validate_table: cannot read %s\n", argv[1]);
    return kExitUsage;
  }
  const std::vector<std::uint8_t> file{std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
  std::array<marv::ParamRecord, marv::kParamCount> out{};
  const marv::store::Decoded d = marv::store::decode(file, marv::param_defaults(), out, &validate_table);
  std::printf("fault: %s\n", marv::store::fault_text(d.fault));
  const bool validator_ran = d.fault == marv::store::Fault::None || d.fault == marv::store::Fault::Validator;
  std::printf("validator: %s\n", !validator_ran ? "not run" : (d.validator_reason != nullptr ? d.validator_reason : "accepted"));
  std::printf("config_hash: ");
  for (const std::uint8_t b : d.config_hash) {
    std::printf("%02x", static_cast<unsigned>(b));
  }
  std::printf("\n");
  const bool accepted = d.fault == marv::store::Fault::None;
  std::printf("verdict: %s\n", accepted ? "accepted" : "refused");
  return accepted ? 0 : kExitRefused;
}
