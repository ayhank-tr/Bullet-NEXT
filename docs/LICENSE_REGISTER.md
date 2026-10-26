# License register (C0.12)

| Component | Revision | License | Used as | Obligations |
| --- | --- | --- | --- | --- |
| Bullet 2.84 (Blender copy) | Blender v2.79b `f4dc9f9` | zlib | Legacy backend (renamed) | Keep copyright headers; mark the renamed copy as altered |
| Blender patch to Bullet | Blender v2.79b | GPL-2.0-or-later (Blender) | Inside legacy backend | Combined work distributed under GPL with BGE NEXT |
| Bullet upstream | bullet3 `d3afe76` | zlib | Bullet NEXT base | Keep copyright headers; mark modified files |
| Blender 2.79b game engine sources | v2.79b | GPL-2.0-or-later | Reference for the world factory | Harness code that mirrors BGE logic is GPL-2.0-or-later |
| UPBGE | master `63bd081` (26 Oct 2026) | GPL-2.0-or-later | Reference for BGE wiring: setting and Python names only, no code copied | GPL-compatible |
| Box2D v3 / Box3D (planned, F3c) | to be pinned | MIT | Soft Step design and code reference | Keep MIT notice in any copied file |
| Jolt Physics (optional phase) | to be pinned | MIT | Character controller port | Keep MIT notice |
| Project Chrono `cbt` copy (F1 T1.7) | main `da80252` (25 Oct 2026) | BSD-3-Clause; Bullet parts zlib | Build target for the collision changes (`integration/chrono/`) | Keep notices; our patch files are zlib |
| Jolt Physics ideas (F1) | none | MIT | Flat-region rule, approach-velocity removal at time of impact: re-implemented, no code copied | None |

Not used: XPBD-based code (active NVIDIA patent US11270041B2 — legal review first);
PhysX 5.11+ (Apache-2.0; take 5.10 BSD-3 if ever needed).
