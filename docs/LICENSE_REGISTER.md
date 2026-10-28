# License register (C0.12)

| Component | Revision | License | Used as | Obligations |
| --- | --- | --- | --- | --- |
| Bullet 2.84 (Blender copy) | Blender v2.79b `f4dc9f9` | zlib | Legacy backend (renamed) | Keep copyright headers; mark the renamed copy as altered |
| Blender patch to Bullet | Blender v2.79b | GPL-2.0-or-later (Blender) | Inside legacy backend | Combined work distributed under GPL with BGE NEXT |
| Bullet upstream | bullet3 `d3afe76` | zlib | Bullet NEXT base | Keep copyright headers; mark modified files |
| Blender 2.79b game engine sources | v2.79b | GPL-2.0-or-later | Reference for the world factory | Harness code that mirrors BGE logic is GPL-2.0-or-later |
| UPBGE | master `63bd081` (26 Oct 2026) | GPL-2.0-or-later | Reference for BGE wiring: setting and Python names only, no code copied | GPL-compatible |
| Box2D v3 (F3c) | v3.2.0 + 2, `a292912` (20 Oct 2026), reference only | MIT | Soft Step design: substeps, soft constraint coefficients (`b2MakeSoft`), relax and restitution passes; re-implemented on Bullet's data structures, no file copied | Credited in `btSoftStepConstraintSolver.cpp` |
| Rapier ideas (F3c) | none | Apache-2.0 | Rebuilding joints every substep: re-implemented with Bullet's own joints, no code copied | None |
| PositionBasedDynamics (Stage 1 reference) | v2.2.2, not vendored | MIT | Reference for the car-body damage model and elastic rods; a Lab reference engine for soft bodies and rods only. No XPBD method adopted | Keep MIT notice in any copied file; patents below |
| Jolt Physics | v5.6.0 (`third_party/jolt`, FL) | MIT | Lab engine, the yardstick; later the character controller port | Keep MIT notice (`third_party/jolt/LICENSE`) |
| Project Chrono `cbt` copy (F1 T1.7) | main `da80252` (25 Oct 2026) | BSD-3-Clause; Bullet parts zlib | Build target for the collision changes (`integration/chrono/`) | Keep notices; our patch files are zlib |
| Jolt Physics ideas (F1) | none | MIT | Flat-region rule, approach-velocity removal at time of impact: re-implemented, no code copied | None |

| HARFANG 3D | v3.2.7 `ef65b8a` (14 Aug 2023), fetched, not vendored | LGPL-3.0 (or GPL-3.0 / commercial) | Lab host engine; our changes are `integration/harfang/harfang-lab.patch` | LGPL: the patch is published; programs linking HARFANG let users relink it |
| FABgen | `241a246` (23 Nov 2022), fetched | GPL-3.0 | HARFANG's binding generator, build tool only | None for the generated bindings |
| HARFANG ImGui shaders (`lab/viewer/assets/core/shader/imgui*`) | from harfang3d/physics-qa-python | MIT (repo) / HARFANG licence header | Viewer UI shaders | Keep headers |
| bgfx `bgfx_shader.sh` (`lab/viewer/assets`) | HARFANG's bgfx pin | BSD-2-Clause | Shader include | Keep notice |
| harfang-emscripten-c-experiment | `942a472` | MIT | Reference for the HARFANG browser build; no code copied | None |
| Blender 5.3 rigid body code | `third_party/blender53` (rb_bullet_api.cpp, RBI_api.h) | GPL-2.0-or-later | Lab reference driver `bnl_rbref` (TL.5) only, not in Bullet NEXT | GPL for that program |
| Blender 5.3 Bullet copy | `third_party/blender53/bullet2` | zlib (Blender patches GPL-2.0-or-later) | Reference engine for the blender53-rb profile | As for the legacy copy |
| nlohmann/json | 3.10.5 (`third_party/nlohmann`) | MIT | Lab scene files | Keep MIT notice |
| musl float math | musl 1.2.4 as in Emscripten 3.1.47 (`third_party/musl_math`) | MIT | Bullet NEXT's portable sin, cos, atan2, exp, log, pow (`BN_PORTABLE_MATH`) | Keep `COPYRIGHT` |
| Project Chrono (Lab engine) | main `da80252`, built out of tree (`lab/engines/bnl_chrono.cpp`) | BSD-3-Clause | Lab engine for recordings | Keep notices |
| Project Chrono::Vehicle (Stage 1 rigs) | main `da80252`: ten tyre-related files compiled out of tree; the BMW E90 TMeasy tyre numbers copied into `integration/chrono/bn_vehicle_rigs.cpp`; the sedan Pacejka 2002 `.tir` file fetched at run time | BSD-3-Clause | Reference tyre and driveline curves for Rally FX's models | Keep notices (credited in the rig's header) |
| Emscripten | 3.1.47, build tool | MIT / UIUC | Browser build | None for the output |
| SDL2 (browser build) | 2.24.2 (Emscripten port) | zlib | Window and input in the browser page | Keep notice |

Not used: XPBD-based code (NVIDIA patent US 11,270,041 B2, filed 2018, listed as active to 2040 —
legal review first; the MIT licence of PositionBasedDynamics grants no patent rights). The
original position-based dynamics patent (US 7,616,204, from a 2005 filing) may be near its end;
its status is unconfirmed, so legal review comes before any method from it is adopted;
PhysX 5.11+ (Apache-2.0; take 5.10 BSD-3 if ever needed).
