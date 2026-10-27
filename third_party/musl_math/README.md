# musl float math for Bullet NEXT

Bullet NEXT's own copy of the float functions it calls through `btSin`, `btCos`, `btTan`,
`btAsin`, `btAcos`, `btAtan`, `btAtan2`, `btExp`, `btLog` and `btPow` (`BN_PORTABLE_MATH`,
`next/LinearMath/btScalar.h`). With it, a Bullet NEXT scene gives the same results on Windows,
Linux, macOS and in the browser: the system C libraries return different last bits for these
functions, and glibc even switches routine on CPUs with FMA.

- Source: musl 1.2.4, as shipped in Emscripten 3.1.47 (`system/lib/libc/musl/src/math`), so the
  browser build and the copy here are the same code. Licence: MIT (`COPYRIGHT`).
- Files: `sinf.c cosf.c tanf.c asinf.c acosf.c atanf.c atan2f.c expf.c logf.c powf.c`, their
  helpers `__sindf.c __cosdf.c __tandf.c __rem_pio2f.c __rem_pio2_large.c`, and the tables
  `exp2f_data.c/.h logf_data.c/.h powf_data.c/.h`.
- `libm.h` is ours: it replaces musl's internal header, gives every function a private `bn_`
  name, and implements musl's error helpers without `errno`.

Changes to musl's files, all mechanical:

1. In the three `*_data.h` headers, `#include <features.h>` (musl internal) became
   `#include "libm.h"`.
2. Hexadecimal float literals (`0x1.921fb6p-1`) are written as their exact decimal values by
   `tools/musl_decimal_literals.py`, because some Visual Studio C compilers reject hexadecimal
   floats. The compiled code and constants were checked to be byte for byte the same as before
   (GCC 13, `.text` and `.rodata` of every file).

The files are compiled as C99 with the rest of Bullet NEXT's strict floating point rules
(no FMA contraction, no fast-math). Verified: 200,000 inputs per function give the same bits as
Emscripten's own libc in the browser.
