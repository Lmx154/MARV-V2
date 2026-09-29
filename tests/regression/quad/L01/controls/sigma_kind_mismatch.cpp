// Negative control for the build-time sigma check (docs/decisions/0001, invariant I1): a record table in the form of
// a generated param_defaults.cpp, with one record whose sigma_kind (Known) contradicts its sigma (+0.0f), and the same
// static_assert the generator emits. It must fail to compile, and only at that static_assert.
#include <cstddef>

#include <marv/params/param_types.hpp>

namespace marv {

inline constexpr std::size_t kParamCount = 2;

namespace generated {
extern const ParamRecord kParamDefaults[kParamCount];
}  // namespace generated

}  // namespace marv

namespace marv::generated {

constexpr ParamRecord kParamDefaults[kParamCount] = {
    // planted_good
    {{ParamType::F32, 1.0f, 0}, 0.5f, ParamOrigin::DefaultFromCard, ParamMethod::Measured, false,
     SigmaKind::Known,
     "kg", "planted"},
    // planted_mismatch
    {{ParamType::F32, 1.0f, 0}, 0.0f, ParamOrigin::DefaultFromCard, ParamMethod::Measured, false,
     SigmaKind::Known,
     "kg", "planted"},
};

static_assert(sigma_table_consistent(kParamDefaults),
              "param_defaults: a record violates invariant I1 (SigmaKind::Known needs a finite sigma > 0, every other SigmaKind needs sigma +0.0f)");

}  // namespace marv::generated
