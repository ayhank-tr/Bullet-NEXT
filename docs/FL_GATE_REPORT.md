# FL gate report (the Lab)

Platform: Linux x86_64 cloud machine, 2 CPUs, GCC 13; WebAssembly with Emscripten 3.1.47 (Node 22,
headless Chromium for the page). Run with `tools/run_fl_gate.py` (raw outputs in `gate_fl/`, not
committed). Windows (MSVC) runs in CI on the first push; the PC tests come at project end.

**FL gate (cloud part): PASSED.** All eight tests and the host-overhead gate pass here. Still open:
the Windows CI run (TL.1 Windows, and whether Bullet NEXT gives the same bits on Windows as on
Linux) and the PC smoke tests (viewer, physics-qa scripts, Snooker, Dogfight sandbox).

## Tests

| Test | Result | Measured |
| --- | --- | --- |
| TL.1 Builds | PASS (Linux) | Lab, HARFANG 3.2.7 with Bullet NEXT and legacy, Python module, viewer: built. HARFANG unit tests 48 of 49 (engine.audio: no sound device in the cloud). Windows: CI workflows written |
| TL.2 Legacy untouched | PASS | `legacy/` unchanged since F0; 16 of 16 F0 scenes equal the F0 goldens (harness and Lab runner) |
| TL.3 Same physics in HARFANG | PASS | 84 of 84 runs headless in HARFANG (Python) reach the Lab's hash: legacy, next, nextf, Jolt, 21 scenes each |
| TL.4 All engines | PASS | Jolt 21 of 21 scenes, Chrono 18 of 18 (vehicles, characters and soft bodies are not part of the Chrono engine), no NaN or blow-up. Chrono's 11 collision tests: same result patched as stock |
| TL.5 Host profiles | PASS | bge279: 16 of 16 F0 goldens. blender53-rb on Blender 5.3's Bullet: 5 of 5 rb scenes bit-identical to Blender 5.3's own rigid body code |
| TL.6 Recordings | PASS | 84 of 84 runs played back with the live hash on every frame |
| TL.7 Check mode | PASS | Address, leak and undefined-behaviour checkers: no reports on any scene (one thread, and two threads for Bullet NEXT). Thread checker: no reports on 21 scenes, 4 threads. 300,000 random collision pairs (100,000 per Bullet engine), with and without the checkers: no crash, no NaN, all normals unit length |
| TL.8 Viewer and browser | PASS | Viewer draws 21 of 21 scenes. Browser build, same hash as the native one-thread run: next 21/21, nextf 21/21, Jolt 21/21 (legacy 15/21, see below). The browser page itself: nextf "same as native" on 21 of 21 scenes |
| BL.1 Host overhead | PASS | 1,000-body pile: HARFANG's frame costs 2.2 % (legacy), 2.4 % (next), 2.2 % (nextf), 3.5 % (Jolt) on top of the physics step (limit 10 %) |

## What the Lab found and fixed

The Lab did its job: running the same scenes everywhere turned up these problems, all fixed.
None of them changes a hash of the 21 scenes except where stated.

**Bullet NEXT (library):**

1. **Same results on every machine and in the browser** (changes Bullet NEXT's hashes against F1).
   - `btVector3::normalize()` used the CPU's approximate inverse square root (`rsqrtss`), which
     returns different bits on Intel and AMD and in the browser. It now uses an exact square root.
   - sin, cos, tan, asin, acos, atan, atan2, exp, log and pow came from the system's C library,
     which differs between Linux, Windows and the browser (Linux even switches routine on CPUs
     with FMA). Bullet NEXT now has its own copy of musl's float math (`third_party/musl_math`,
     MIT, switch `BN_PORTABLE_MATH`, on by default).
   - The browser build runs Bullet NEXT's SSE code through WebAssembly SIMD, as on the PC.
2. **Crash:** a hull whose points lie on a plane, a line or one point crashed the hull-clipping
   feature (found by the pair fuzzer). Such hulls now take the GJK path. Also ported to Chrono's copy.
3. **Leaks:** edge maps made by the active-edges feature were never freed (now owned by their mesh
   shape, also in Chrono's copy); every SAT contact with a mesh triangle leaked its polyhedron's
   face lists.
4. **Thread races (thread checker):** the threaded solver had threads adding zeros to the shared
   static body at once; rows of one joint could be written back by two threads (applied impulse,
   feedback, break flag: a real, rare source of non-determinism); a solver maximum and a
   statistics counter were updated from several threads; the task scheduler read its flags
   without synchronisation.
5. **Undefined behaviour (address/UB checker):** the job queues were declared cache-line aligned
   but stored at 16-byte addresses; the batched solver carved its scratch memory at misaligned
   offsets; a wheel was copied before its raycast data was set.

**The Lab itself:** the Blender profile inherited two Bullet globals from a BGE-profile scene run
earlier in the same program (rb scenes depended on scene order; now reset as Blender has them);
meshes, compound children and soft-body distance caches leaked on world teardown; a 64 KB
WebAssembly stack overflowed in Bullet's GJK (now 4 MB).

**Not changed, on purpose:** legacy (Bullet 2.84 as in Blender 2.79b) keeps the system math
library, so like Blender 2.79b itself its results depend on the platform: in the browser 15 of 21
scenes match the PC, the six others use sin/cos/atan2 more heavily. Its pair-cache hash still
overflows a signed int as in 2.79b (suppressed in check mode for `legacy/` only).

## F1 re-run

The library changes above were checked against F1: `docs/F1_GATE_REPORT.md` is the re-run
(T1.1 to T1.5 and T1.7 pass, now on all 21 scenes; T1.6 waits for the PC).

## Benchmarks

Indicative only: a shared 2-CPU cloud machine, every engine stepped in turn in one program.
The PC numbers (8 threads) come at project end.

### BL.2 Reference table: physics time per scene (ms for the whole scene; 1 thread / all threads)

| Scene | legacy | next | nextf | jolt |
| --- | --- | --- | --- | --- |
| stack_10 | 4.7 | 4.6 / 6.1 | 4.3 / 5.5 | 3.5 |
| pyramid_10 | 41.4 | 39.4 / 42.8 | 47.0 / 59.1 | 29.6 |
| shape_zoo | 13.1 | 15.7 / 16.2 | 68.2 / 66.9 | 5.7 |
| joints_six | 10.5 | 8.8 / 7.6 | 8.9 / 7.5 | 8.0 |
| vehicle | 5.5 | 5.5 / 5.1 | 5.2 / 4.4 | 9.8 |
| character | 27.7 | 7.0 / 5.0 | 6.8 / 5.0 | 2.3 |
| soft_body | 217.7 | 13.4 / 12.7 | 12.1 / 11.7 | 8.0 |
| sensors_filters | 1.9 | 1.6 / 2.1 | 1.6 / 2.0 | 1.6 |
| projectile | 0.9 | 1.0 / 1.8 | 1.6 / 2.4 | 1.9 |
| tunnel_100 | 6.1 | 6.2 / 6.3 | 21.6 / 20.3 | 14.8 |
| mesh_slide | 3.0 | 3.3 / 3.7 | 5.9 / 5.3 | 4.4 |
| body_types | 1.9 | 2.4 / 2.1 | 5.2 / 1.8 | 2.5 |
| kinematic_switch | 2.5 | 2.0 / 2.5 | 2.1 / 2.2 | 2.3 |
| pile_1000 | 2918.7 | 2708.6 / 2785.9 | 3132.2 / 2545.0 | 1638.4 |
| joint_grid | 353.0 | 367.1 / 390.1 | 424.2 / 455.1 | 257.1 |
| settings | 40.7 | 38.0 / 39.8 | 62.8 / 127.7 | 48.5 |
| rb_stack | 69.0 | 63.3 / 59.7 | 66.2 / 62.6 | 50.6 |
| rb_shapes | 128.9 | 194.4 / 190.0 | 545.5 / 559.8 | 68.2 |
| rb_constraints | 88.2 | 89.6 / 85.8 | 95.4 / 85.3 | 43.3 |
| rb_kinematic | 60.8 | 56.1 / 58.2 | 56.5 / 57.4 | 61.2 |
| rb_settings | 791.1 | 712.7 / 691.2 | 799.7 / 790.6 | 681.1 |

### BL.3 Browser speed: physics time per scene, native and WebAssembly (ms, one thread)

| Scene | legacy native / wasm | nextf native / wasm | jolt native / wasm | nextf wasm / native |
| --- | --- | --- | --- | --- |
| stack_10 | 4.7 / 19.7 | 4.3 / 16.3 | 3.5 / 27.1 | 3.8x |
| pyramid_10 | 41.4 / 87.2 | 47.0 / 78.4 | 29.6 / 75.7 | 1.7x |
| shape_zoo | 13.1 / 42.6 | 68.2 / 96.9 | 5.7 / 26.4 | 1.4x |
| joints_six | 10.5 / 18.7 | 8.9 / 12.9 | 8.0 / 28.9 | 1.5x |
| vehicle | 5.5 / 12.8 | 5.2 / 7.6 | 9.8 / 31.4 | 1.5x |
| character | 27.7 / 29.9 | 6.8 / 6.2 | 2.3 / 12.3 | 0.9x |
| soft_body | 217.7 / 18.0 | 12.1 / 25.7 | 8.0 / 24.1 | 2.1x |
| sensors_filters | 1.9 / 3.6 | 1.6 / 2.2 | 1.6 / 4.8 | 1.4x |
| projectile | 0.9 / 5.9 | 1.6 / 2.3 | 1.9 / 4.9 | 1.4x |
| tunnel_100 | 6.1 / 8.7 | 21.6 / 37.4 | 14.8 / 33.9 | 1.7x |
| mesh_slide | 3.0 / 5.1 | 5.9 / 6.5 | 4.4 / 6.4 | 1.1x |
| body_types | 1.9 / 3.2 | 5.2 / 2.2 | 2.5 / 3.1 | 0.4x |
| kinematic_switch | 2.5 / 3.9 | 2.1 / 2.7 | 2.3 / 4.6 | 1.3x |
| pile_1000 | 2918.7 / 3717.2 | 3132.2 / 3774.3 | 1638.4 / 2422.8 | 1.2x |
| joint_grid | 353.0 / 490.2 | 424.2 / 497.7 | 257.1 / 499.6 | 1.2x |
| settings | 40.7 / 54.7 | 62.8 / 71.5 | 48.5 / 68.5 | 1.1x |
| rb_stack | 69.0 / 88.4 | 66.2 / 71.8 | 50.6 / 66.8 | 1.1x |
| rb_shapes | 128.9 / 191.2 | 545.5 / 556.5 | 68.2 / 101.2 | 1.0x |
| rb_constraints | 88.2 / 129.4 | 95.4 / 108.5 | 43.3 / 72.6 | 1.1x |
| rb_kinematic | 60.8 / 91.9 | 56.5 / 74.3 | 61.2 / 84.5 | 1.3x |
| rb_settings | 791.1 / 1161.2 | 799.7 / 951.9 | 681.1 / 903.1 | 1.2x |


Notes: `nextf` pays for its features on shape-heavy scenes (shape_zoo, rb_shapes: hull clipping
and SAT; tunnel_100: continuous collision). In the browser, physics runs at roughly 1.1 to 2 times
its native time on the larger scenes; the small ones are dominated by start-up.
