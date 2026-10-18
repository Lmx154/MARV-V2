// Negative control of the post-link parameter-table check (cmake/param_table.cmake; decision 0053, ground-segment unit 4b).
// Compiled against the FC3v2 set's header, so the FC3v2 runtime's params_init is the one linked; the link line offers the
// product set's generated archive before FC3v2's, so the runtime's reference to kParamDefaults resolves to the product
// table: the silent mix that adding the store to the m33 image once produced. The check, asked for FC3v2, must fail.
#include <marv/hal/hal.hpp>
#include <marv/params/param.hpp>

// The runtime's only HAL call; no HAL library is linked, so the control is the runtime and the two tables alone.
void marv::hal_panic(const char*) noexcept {
  for (;;) {
  }
}

int main() { return marv::params_init(marv::param_defaults()) ? 0 : 1; }
