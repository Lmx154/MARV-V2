/* fw/sil/include/marv_truth_vertical.h - the test-only truth-vertical entry of a SIL library built with TRUTH_VERTICAL.
 * Pure C. Exported only by such libraries (marv_sil_truth_vertical.map, marv_sil_truth_state_vertical.map); a product
 * SIL library does not have it. */
#ifndef MARV_TRUTH_VERTICAL_H
#define MARV_TRUTH_VERTICAL_H

#include <stdint.h>

#include "marv_sil.h"

#ifdef __cplusplus
extern "C" {
#endif

enum { MARV_TRUTH_VERTICAL_VALID = 0, MARV_TRUTH_VERTICAL_FLAG_COUNT };
typedef struct marv_truth_vertical {
  uint32_t struct_size; /* sizeof(marv_truth_vertical): the version check */
  uint32_t flags;       /* bit MARV_TRUTH_VERTICAL_VALID only */
  uint64_t tick;        /* the tick of the next marv_sil_tick */
  float z_ned_m;        /* down-position, NED (z down), m */
  float vz_ned_m_s;     /* down-velocity, NED (z down), m/s */
} marv_truth_vertical;

/* READY; TRUTH_VERTICAL libraries only. Call order per tick j: marv_truth_vertical_set(tick = j), then
 * marv_sil_tick(j, 1, ...). E_STATE unless READY; E_NULL; E_ABI (struct_size); E_TICK (tick is not the number of
 * ticks already run); E_INPUT (a reserved flag bit; valid bit clear with a nonzero field; valid bit set with a
 * non-finite field). A second set for the same tick replaces the first. */
marv_sil_status marv_truth_vertical_set(const marv_truth_vertical* s);

#ifdef __cplusplus
}
#endif

#endif /* MARV_TRUTH_VERTICAL_H */
