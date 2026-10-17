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

## MAVLink message definitions, generated C headers and pymavlink 2.4.50 (link codec, ground segment unit 5)

- **Where:** not vendored. `fw/link/CMakeLists.txt` runs `tools/link/fetch_defs.py`, which downloads `common.xml`,
  `standard.xml` and `minimal.xml` from <https://github.com/mavlink/mavlink> at commit
  `2e2b6af99084e4fde9bb304c84bb9253a1a98b6f` (URL template and SHA-256 of each file in `link/mavlink_pin.json`), into the
  build tree. The MARV dialect `link/marv.xml` includes them. pymavlink's `mavgen` generates the C headers into the build
  tree; `tools/link/gen_ts.py` and `tools/link/gen_cpp_structs.py` read the same XML for the TypeScript and plain C++
  message types (`gcs/src/lib/link/generated/messages.ts` is committed).
- **Use:** the link codec only (`fw/link/src/codec_mavlink.cpp`, the only code that includes a MAVLink header, GS-3;
  `gcs/src/lib/link/`, which has no MAVLink library at runtime). The codec library enters a flight image only when a
  composition links it.
- **Licences:**
  - The MAVLink message definitions (`common.xml`, `standard.xml`, `minimal.xml`): MIT, mavlink/mavlink (<https://mavlink.io/en/#license>;
    the `COPYING` of the repository at the pinned commit carries the same MIT grant as the one quoted below).
  - The generated C headers (mavgen output, including the fixed headers it copies from pymavlink's
    `generator/C/include_v2.0`): MIT, under the exception in pymavlink's `COPYING`
    (`pymavlink-2.4.50.dist-info/licenses/COPYING`). The exception reads:

> As an exception, if you use this Software to compile your source code and
> portions of this Software, including messages ("the generator output"), are embedded
> into the binary product as a result, you may redistribute such product and such
> product is hereby licensed under the following MIT license:
>
> Copyright (C) the ArduPilot project
> http://ardupilot.org/
>
> Permission is hereby granted, free of charge, to any person obtaining a copy
> of the generated software (the "Generated Software"), to deal
> in the Generated Software without restriction, including without limitation the
> rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
> copies of the Generated Software, and to permit persons to whom the Generated
> Software is furnished to do so, subject to the following conditions:
>
> The above copyright notice and this permission notice shall be included in
> all copies or substantial portions of the Generated Software.
>
> THE GENERATED SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
> OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
> FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
> AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
> LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
> OUT OF OR IN CONNECTION WITH THE GENERATED SOFTWARE OR THE USE OR OTHER DEALINGS
> IN THE GENERATED SOFTWARE.

  - pymavlink 2.4.50 itself: LGPL-3 (`License: LGPLv3`, `COPYING`). It is a build and test tool only: a dev dependency
    in `pyproject.toml` and `uv.lock`, run as `python -m pymavlink.tools.mavgen` and imported by no runtime code. None of
    it is linked into or shipped with any image, library or the ground-station package.

## websockets 17.2 (ground-segment helper dependency)

- **Where:** not vendored. A runtime dependency of `tools/gs/helper.py` in `pyproject.toml` and `uv.lock`, installed by
  `uv sync --frozen`. Copyright (c) Aymeric Augustin and contributors.
- **Licence:** BSD-3-Clause (`License-Expression: BSD-3-Clause` in the package metadata; the licence file ships in its
  distribution). It is used by the helper on the developer's machine only; it is not part of `fw/` and enters no image.
