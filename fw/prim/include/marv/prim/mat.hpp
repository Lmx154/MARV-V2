#pragma once

#include <cstddef>

#include "marv/prim/vec.hpp"

namespace marv::prim {

// Fixed-size R x C matrix of scalars of type T, stored row-major. Static storage, no heap,
// no exceptions. Indexing is unchecked: the caller keeps 0 <= r < R and 0 <= c < C.
template <class T, std::size_t R, std::size_t C>
struct Mat {
  T e[R * C]{};

  constexpr Mat() = default;

  // Row-major elements: element (r, c) is row_major[r * C + c].
  constexpr explicit Mat(const T (&row_major)[R * C]) {
    for (std::size_t i = 0; i < R * C; ++i) {
      e[i] = row_major[i];
    }
  }

  static constexpr Mat identity()
    requires(R == C)
  {
    Mat out;
    for (std::size_t i = 0; i < R; ++i) {
      out.e[i * C + i] = T(1);
    }
    return out;
  }

  constexpr T& operator()(std::size_t r, std::size_t c) { return e[r * C + c]; }
  constexpr const T& operator()(std::size_t r, std::size_t c) const { return e[r * C + c]; }

  constexpr Mat operator+(const Mat& m) const {
    Mat out;
    for (std::size_t i = 0; i < R * C; ++i) {
      out.e[i] = e[i] + m.e[i];
    }
    return out;
  }

  constexpr Mat operator-(const Mat& m) const {
    Mat out;
    for (std::size_t i = 0; i < R * C; ++i) {
      out.e[i] = e[i] - m.e[i];
    }
    return out;
  }

  constexpr Mat operator*(T s) const {
    Mat out;
    for (std::size_t i = 0; i < R * C; ++i) {
      out.e[i] = e[i] * s;
    }
    return out;
  }

  constexpr Vec<T, R> operator*(const Vec<T, C>& v) const {
    Vec<T, R> out;
    for (std::size_t r = 0; r < R; ++r) {
      T sum = T(0);
      for (std::size_t c = 0; c < C; ++c) {
        sum += e[r * C + c] * v.e[c];
      }
      out.e[r] = sum;
    }
    return out;
  }

  constexpr Mat<T, C, R> transpose() const {
    Mat<T, C, R> out;
    for (std::size_t r = 0; r < R; ++r) {
      for (std::size_t c = 0; c < C; ++c) {
        out.e[c * R + r] = e[r * C + c];
      }
    }
    return out;
  }
};

template <class T, std::size_t R, std::size_t K, std::size_t C>
constexpr Mat<T, R, C> operator*(const Mat<T, R, K>& a, const Mat<T, K, C>& b) {
  Mat<T, R, C> out;
  for (std::size_t r = 0; r < R; ++r) {
    for (std::size_t c = 0; c < C; ++c) {
      T sum = T(0);
      for (std::size_t k = 0; k < K; ++k) {
        sum += a.e[r * K + k] * b.e[k * C + c];
      }
      out.e[r * C + c] = sum;
    }
  }
  return out;
}

template <class T>
using Mat3 = Mat<T, kSpatialDim, kSpatialDim>;

}  // namespace marv::prim
