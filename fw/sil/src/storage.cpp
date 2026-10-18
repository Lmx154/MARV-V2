// A SIL library built with STORAGE (fw/sil/CMakeLists.txt): the config store's boot step (marv/store/store.hpp) on hal_sim's
// file storage in the run directory marv_sil_storage_open names (marv/hal_sim/storage.hpp), bound to the image hash of
// marv_sil_config; and the ground link (ground-segment unit 6): the link service of fw/link (marv/link/service.hpp) on hal_sim's
// UDP socket (marv/hal_sim/link_udp.hpp), run by marv_sil_link_service between tick calls. Without a run directory the storage
// is empty and the boot step returns the compiled table with status Ok, so the run is the run without the store; without
// marv_sil_link_open the link does nothing.
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>

#include <marv/arming/arming.hpp>
#include <marv/composition.hpp>
#include <marv/hal/hal.hpp>
#include <marv/hal/storage.hpp>
#include <marv/hal_sim/link_udp.hpp>
#include <marv/hal_sim/storage.hpp>
#include <marv/link/service.hpp>
#include <marv/store/store.hpp>

#include "marv_sil.h"
#include "sil_session.hpp"
#include "sil_storage.hpp"

namespace marv::sil {
namespace {

bool g_booted = false;
// The store's working memory: a pending file, a slot copy, the staged config's encoding.
std::array<std::uint8_t, kConfigSlotBytes> g_scratch{};
store::Digest g_image{};
bool g_link_open = false;
bool g_link_started = false;

const char* validate_table(ParamTable table) noexcept { return composition::validate(table).reason; }
bool armed_now() noexcept { return arming::is_armed(composition::arming_status()); }
std::size_t link_receive(std::span<std::uint8_t> out) noexcept { return hal_sim::link_udp_receive(out); }
void link_send(std::span<const std::uint8_t> frame) noexcept { hal_sim::link_udp_send(frame); }

}  // namespace

std::span<const ParamRecord, kParamCount> boot_table(std::span<const ParamRecord, kParamCount> defaults,
                                                     const ImageDigest& image_sha256) noexcept {
  g_booted = true;
  g_image = image_sha256;
  const ParamTable table = store::boot(store::BootInputs{
      defaults, g_image,
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

[[gnu::visibility("default")]] marv_sil_status marv_sil_link_open(std::uint32_t udp_port) {
  using namespace marv::sil;
  if (g_booted || g_link_open) {
    return MARV_SIL_E_STATE;
  }
  if (udp_port == 0 || udp_port > std::numeric_limits<std::uint16_t>::max()) {
    return MARV_SIL_E_LINK;
  }
  if (!marv::hal_sim::link_udp_open(static_cast<std::uint16_t>(udp_port))) {
    return MARV_SIL_E_LINK;
  }
  g_link_open = true;
  return MARV_SIL_OK;
}

[[gnu::visibility("default")]] marv_sil_status marv_sil_link_service(std::uint32_t* reboot_requested) {
  using namespace marv::sil;
  if (reboot_requested == nullptr) {
    return MARV_SIL_E_NULL;
  }
  *reboot_requested = 0;
  if (!session_view().ready) {
    return MARV_SIL_E_STATE;
  }
  if (!g_link_open) {
    return MARV_SIL_OK;
  }
  if (!g_link_started) {
    marv::link::service_init(marv::link::ServiceConfig{
        marv::link::Transport{&link_receive, &link_send}, g_image, &armed_now});
    g_link_started = true;
  }
  *reboot_requested = marv::link::service_step() ? 1 : 0;
  return MARV_SIL_OK;
}

}  // extern "C"
