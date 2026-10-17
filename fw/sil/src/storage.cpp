// A SIL library built with STORAGE (fw/sil/CMakeLists.txt): the config store's boot step (marv/store/store.hpp) on
// hal_sim's file storage in the run directory marv_sil_storage_open names (marv/hal_sim/storage.hpp). Without one the
// storage is empty and the boot step returns the compiled table with status Ok, so the run is the run without the store.
// The SIL's image hash is not known yet (ground-segment unit 6 passes it), so no stored config is bound to it.
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include <marv/arming/arming.hpp>
#include <marv/composition.hpp>
#include <marv/hal/hal.hpp>
#include <marv/hal/storage.hpp>
#include <marv/hal_sim/storage.hpp>
#include <marv/store/store.hpp>

#include "marv_sil.h"
#include "sil_storage.hpp"

namespace marv::sil {
namespace {

bool g_booted = false;
// The store's working memory: a pending file, a slot copy, the staged config's encoding.
std::array<std::uint8_t, kConfigSlotBytes> g_scratch{};

const char* validate_table(ParamTable table) noexcept { return composition::validate(table).reason; }
bool armed_now() noexcept { return arming::is_armed(composition::arming_status()); }

}  // namespace

std::span<const ParamRecord, kParamCount> boot_table(std::span<const ParamRecord, kParamCount> defaults) noexcept {
  g_booted = true;
  const ParamTable table = store::boot(store::BootInputs{
      defaults, store::Digest{},
      store::Hooks{&validate_table, &armed_now, &composition::request_maintenance, &composition::maintenance_granted},
      g_scratch});
  if (table.size() != kParamCount) {
    hal_panic("marv_sil_init: the config store returned a table of another parameter set");
  }
  composition::set_config_status(store::boot_report().status);
  return std::span<const ParamRecord, kParamCount>{table.data(), kParamCount};
}

}  // namespace marv::sil

extern "C" {

[[gnu::visibility("default")]] marv_sil_status marv_sil_storage_open(const char* run_dir) {
  if (marv::sil::g_booted) {
    return MARV_SIL_E_STATE;
  }
  if (run_dir == nullptr) {
    return MARV_SIL_E_NULL;
  }
  return marv::hal_sim::storage_open(run_dir) ? MARV_SIL_OK : MARV_SIL_E_STORAGE;
}

}  // extern "C"
