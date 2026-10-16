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

## Pico SDK 2.3.1 and TinyUSB 0.18.0 (fetched at configure time, m33 build only)

- **Where:** not vendored. `cmake/pico_sdk.cmake` downloads the GitHub archives of
  <https://github.com/raspberrypi/pico-sdk> at commit `079c6f39023649b154152db30f1d781e884879bc` (release 2.3.1) and of
  <https://github.com/hathach/tinyusb> at commit `86ad6e56c1700e85f1c5678607a762cfe3aa2f47` (the TinyUSB commit of that
  release), each checked against the SHA-256 in that file, into the build tree. Host builds do not fetch them.
- **Use:** the target build (`marv_hal_target` and the image, `fw/hal/target`, decision 0027). Both enter the flight image.
- **Licences:**
  - Pico SDK: BSD-3-Clause, Copyright 2020 (c) 2020 Raspberry Pi (Trading) Ltd. (`LICENSE.TXT` of the archive). The
    binary-redistribution clause (the licence text with the distribution) applies to the image.
  - TinyUSB: MIT, Copyright (c) 2018, hathach (tinyusb.org) (`LICENSE` of the archive).
