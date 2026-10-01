# Third-party notices

## musl libc 1.2.5 (vendored maths: `log`, `sin`, `cos` and their closure)

- **Where:** `sim/plant/third_party/musl/`, unmodified files of the official release tarball
  <https://musl.libc.org/releases/musl-1.2.5.tar.gz>. Provenance, the tarball hash and the SHA-256 of every vendored file
  are in `sim/plant/third_party/musl/PROVENANCE.txt`; the musl licence text is `sim/plant/third_party/musl/COPYRIGHT`.
- **Use:** host simulation only (the plant's seeded noise stream, `sim/plant/src/noise`), for bit-identical normals on
  every host. It is not part of `fw/` and does not enter any flight image.
- **Licences:**
  - musl as a whole: MIT, Copyright © 2005-2020 Rich Felker, et al. (`COPYRIGHT`). It covers every file without its own
    notice (`floor.c`, `scalbn.c`, `__math_divzero.c`, `__math_invalid.c`, `libm.h`, `log_data.h`, `fp_arch.h`).
  - `log.c`, `log_data.c`: Copyright (c) 2018, Arm Limited, SPDX-License-Identifier: MIT (ARM Optimized Routines).
  - `sin.c`, `cos.c`, `__sin.c`, `__cos.c`, `__rem_pio2.c`, `__rem_pio2_large.c`: Sun Microsystems, Inc. (fdlibm, via
    FreeBSD msun). Each file carries this notice, which must be preserved:

```
 * ====================================================
 * Copyright (C) 1993 by Sun Microsystems, Inc. All rights reserved.
 *
 * Developed at SunPro, a Sun Microsystems, Inc. business.
 * Permission to use, copy, modify, and distribute this
 * software is freely granted, provided that this notice
 * is preserved.
 * ====================================================
```
