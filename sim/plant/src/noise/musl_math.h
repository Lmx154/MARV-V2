/* The vendored musl 1.2.5 routines (third_party/musl, MIT) under the names the build gives them. These three, and
 * the IEEE square root, are the only maths functions the noise stream calls: bit-identical on every host. */
#ifndef MARV_PLANT_NOISE_MUSL_MATH_H
#define MARV_PLANT_NOISE_MUSL_MATH_H

#ifdef __cplusplus
extern "C" {
#endif

double marv_musl_log(double x);
double marv_musl_sin(double x);
double marv_musl_cos(double x);

#ifdef __cplusplus
}
#endif

#endif
