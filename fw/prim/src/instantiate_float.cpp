#include <cstddef>

#include "marv/prim/mat.hpp"
#include "marv/prim/quat.hpp"
#include "marv/prim/vec.hpp"

namespace marv::prim {

static_assert(sizeof(Quat<float>) == 2 * 2 * sizeof(float));
static_assert(offsetof(Quat<float>, w) == 0);
static_assert(offsetof(Quat<float>, x) == 1 * sizeof(float));
static_assert(offsetof(Quat<float>, y) == 2 * sizeof(float));
static_assert(offsetof(Quat<float>, z) == (2 + 1) * sizeof(float));

template struct Vec<float, kSpatialDim>;
template struct Mat<float, kSpatialDim, kSpatialDim>;
template struct Quat<float>;

template Mat<float, kSpatialDim, kSpatialDim> operator*(
    const Mat<float, kSpatialDim, kSpatialDim>&, const Mat<float, kSpatialDim, kSpatialDim>&);

}  // namespace marv::prim
