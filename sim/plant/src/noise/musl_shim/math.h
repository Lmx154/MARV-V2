/* A stand-in for <math.h> for the vendored musl sources only (third_party/musl, unmodified). They include <math.h> for
 * double_t, float_t, INFINITY and NAN; the platform header would also declare log, sin, cos, __sin, __cos ... and so
 * clash with the vendored definitions and, on some hosts, pull in the platform's own versions. This file declares
 * nothing but those four names and the prototypes of floor and scalbn,
 * which __rem_pio2_large.c calls (the build renames both to marv_musl_*; they are vendored too). It is on the include path of the marv_musl_math target only. */
#ifndef MARV_PLANT_NOISE_MUSL_SHIM_MATH_H
#define MARV_PLANT_NOISE_MUSL_SHIM_MATH_H

#include <float.h>

/* double_t is double only when intermediate results are not kept in a wider format (SSE2 on x86-64, ARM, RISC-V):
 * excess precision (x87) would change the bits. */
#if FLT_EVAL_METHOD != 0
#error "the noise stream needs FLT_EVAL_METHOD == 0 (no excess precision)"
#endif

typedef float float_t;
typedef double double_t;

#define INFINITY __builtin_inff()
#define NAN __builtin_nanf("")

double floor(double);
double scalbn(double, int);

#endif
