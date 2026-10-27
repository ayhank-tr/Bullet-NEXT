# Bullet NEXT — source manifest (C0.1)

Every third-party source in this repository is pinned to an exact revision.

| Path | Origin | Revision | Notes |
| --- | --- | --- | --- |
| `third_party/bullet284_orig/` | Blender `extern/bullet2/src` | Blender tag `v2.79b`, commit `f4dc9f9d68bddaa206b692e1d077d1a1f2bb1528` (22 Mar 2018) | Bullet 2.84 (`BT_BULLET_VERSION 284`) with Blender's patch applied in-tree; patch kept as `BLENDER_PATCH.diff`; `SOURCES.txt` = Blender's own source list (144 files) |
| `legacy/` | Generated from `third_party/bullet284_orig/` by `tools/rename_legacy.py` | same | Frozen legacy backend; never edited by hand. Regenerate, never patch |
| `third_party/bullet3_master/` | `github.com/bulletphysics/bullet3` | master `d3afe76132fc76b0f9b2359cbaef47a3766e7fbb` (9 Oct 2026); `VERSION` 3.27; last tag 3.25 | Only `LinearMath`, `BulletCollision`, `BulletDynamics`, `BulletSoftBody` and the two common headers |
| `next/` | Copy of `third_party/bullet3_master/` | same as bullet3 master above | Bullet NEXT; every change is marked with a "Bullet NEXT" comment |

| `third_party/bullet3_master/Extras/Serialize/` | bullet3 | same `d3afe76` | `.bullet` file reader on Bullet NEXT (HARFANG mesh colliders) |
| `third_party/jolt/` | `github.com/jrouwe/JoltPhysics` | v5.6.0 | Lab engine (C++17, built apart); `VERSION.txt` |
| `third_party/blender53/` | Blender 5.3 `intern/rigidbody`, `extern/bullet2` | see `VERSION.txt` | Blender profile reference: `rb_bullet_api.cpp`, `RBI_api.h`, Blender's Bullet copy |
| `third_party/nlohmann/` | nlohmann/json | 3.10.5 | Lab scene files |
| `third_party/musl_math/` | musl 1.2.4 in Emscripten 3.1.47 | `system/lib/libc/musl/src/math` | Bullet NEXT's portable float math; changes listed in its `README.md` |
| HARFANG 3D (fetched by `tools/fetch_harfang.py`) | `github.com/harfang3d/harfang3d` | v3.2.7 `ef65b8a`; FABgen `241a246` | Lab host; our changes: `integration/harfang/harfang-lab.patch` |

Reference only (not built, not shipped): Blender 2.79b `source/gameengine` for the world
factory and the interface audit.

## Layout

```
CMakeLists.txt            top-level build (C++14, strict floating point)
third_party/              pinned upstream sources
legacy/                   generated, renamed Bullet 2.84 (btl* symbols)
next/                     Bullet NEXT: upstream base plus the F1 changes (marked "Bullet NEXT")
blender_c_api/            Bullet's C API for Blender, served by the legacy backend
integration/              patches for the 2.79b game engine and for Chrono's Bullet copy
goldens/                  legacy hashes per scene (the frozen behaviour)
tools/rename_legacy.py    rename tool + legacy_extra_symbols.txt
harness/                  golden harness, compare tool, symbol check
lab/                      the Lab: scene files, engine interface, Jolt/Chrono engines, viewer, browser page, tools
docs/                     interface audit, gates, license register, gate reports
```
