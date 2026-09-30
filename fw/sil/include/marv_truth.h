/* fw/sil/include/marv_truth.h - the test-only truth-attitude entry of a SIL library built with TRUTH_STATE.
 * Pure C. Exported only by such libraries (marv_sil_truth.map); a product SIL library does not have it. */
#ifndef MARV_TRUTH_H
#define MARV_TRUTH_H

#include <stdint.h>

#include "marv_sil.h"

#ifdef __cplusplus
extern "C" {
#endif

enum { MARV_TRUTH_ATTITUDE_VALID = 0, MARV_TRUTH_FLAG_COUNT };
enum { MARV_TRUTH_Q_W = 0, MARV_TRUTH_Q_X, MARV_TRUTH_Q_Y, MARV_TRUTH_Q_Z, MARV_TRUTH_Q_COUNT };
typedef struct marv_truth_state {
  uint32_t struct_size;             /* sizeof(marv_truth_state): the version check */
  uint32_t flags;                   /* bit MARV_TRUTH_ATTITUDE_VALID only */
  uint64_t tick;                    /* the tick of the next marv_sil_tick */
  float q_wxyz[MARV_TRUTH_Q_COUNT]; /* body FRD -> NED, Hamilton, [w x y z] */
  marv_vec3f omega_frd_rad_s;       /* body rates, FRD */
} marv_truth_state;

/* READY; TRUTH_STATE libraries only. Call order per tick j: marv_truth_state_set(tick = j), then
 * marv_sil_tick(j, 1, ...). E_STATE unless READY; E_NULL; E_ABI (struct_size); E_TICK (tick is not the number of
 * ticks already run); E_INPUT (a reserved flag bit; valid bit clear with a nonzero field; valid bit set with a
 * non-finite field or a zero quaternion). A second set for the same tick replaces the first. */
marv_sil_status marv_truth_state_set(const marv_truth_state* s);

#ifdef __cplusplus
}
#endif

#endif /* MARV_TRUTH_H */
