#pragma once

#include <cmath>
#include <cstddef>

#include "marv/prim/constants.hpp"

namespace marv::prim {

// Fixed-size vector of N scalars of type T. Static storage, no heap, no exceptions.
// Indexing is unchecked: the caller keeps 0 <= i < N.
template <class T, std::size_t N>
struct Vec {
  T e[N]{};

  constexpr Vec() = default;

  constexpr Vec(T x, T y, T z)
    requires(N == kSpatialDim)
      : e{x, y, z} {}

  constexpr T& operator[](std::size_t i) { return e[i]; }
  constexpr const T& operator[](std::size_t i) const { return e[i]; }

  constexpr Vec operator+(const Vec& r) const {
    Vec out;
    for (std::size_t i = 0; i < N; ++i) {
      out.e[i] = e[i] + r.e[i];
    }
    return out;
  }

  constexpr Vec operator-(const Vec& r) const {
    Vec out;
    for (std::size_t i = 0; i < N; ++i) {
      out.e[i] = e[i] - r.e[i];
    }
    return out;
  }

  constexpr Vec operator-() const {
    Vec out;
    for (std::size_t i = 0; i < N; ++i) {
      out.e[i] = -e[i];
    }
    return out;
  }

  constexpr Vec operator*(T s) const {
    Vec out;
    for (std::size_t i = 0; i < N; ++i) {
      out.e[i] = e[i] * s;
    }
    return out;
  }

  constexpr T dot(const Vec& r) const {
    T sum = T(0);
    for (std::size_t i = 0; i < N; ++i) {
      sum += e[i] * r.e[i];
    }
    return sum;
  }

  T norm() const {
    using std::sqrt;
    return sqrt(dot(*this));
  }

  // Right-handed cross product: this x r.
  constexpr Vec cross(const Vec& r) const
    requires(N == kSpatialDim)
  {
    return Vec(e[1] * r.e[2] - e[2] * r.e[1],
               e[2] * r.e[0] - e[0] * r.e[2],
               e[0] * r.e[1] - e[1] * r.e[0]);
  }
};

template <class T>
using Vec3 = Vec<T, kSpatialDim>;

}  // namespace marv::prim
