# Stage 0 gate report (Ready for Rally FX)

Platform: Linux x86_64 cloud machine, 2 CPUs, GCC 13; WebAssembly with Emscripten 3.1.47 (Node 22).
Run with `tools/run_s0_gate.py --build build-lab --out gate_s0 --wasm build-wasm --chrono <bin>
--f1 <f1 run> --f3c gate_f3c` (raw outputs in `gate_s0/`, not committed); tables made with
`tools/s0_report.py`. Windows (MSVC) runs in CI on the push; the PC session comes at the gate.

**Stage 0 gate (cloud part): PASSED.** TS.1 to TS.6 pass; BS.1 and BS.2 are recorded. Stage 0
makes Bullet NEXT safe for the Rally FX car before any new feature: crashes that do not gain
energy, terrain without phantom bumps, long stages without precision trouble, and replays that
match on every platform. Still open: the Windows CI run and the PC session; the engine patches
stay unapplied until you give the go.

## Tests

| Test | Result | Measured |
| --- | --- | --- |
| TS.1 Bounce | PASS | restitution 0.9 comes back as: Bullet 2.84 (2.79b) 0.9371, Bullet NEXT default 0.9371, Bullet NEXT, all collision features 0.9, Soft Step 0.9, Jolt 5.6 0.9 |
| TS.2 No regressions | PASS | F1 gate PASSED; F3c gate PASSED; hashes changed against the reference: legacy 0, Jolt 0, next 0, nextf 18, nexts 4 scenes |
| TS.3 Heightfield bumps | PASS | largest upward speed sliding at 30 and 50 m/s: Bullet 2.84 (2.79b) 10.3941 m/s, Bullet NEXT default 11.5223 m/s, Bullet NEXT, all collision features 0.0001 m/s, Soft Step 0.0 m/s, Jolt 5.6 0.0 m/s |
| TS.4 Far from the origin | PASS | path difference 10 km out, recentred / not: Bullet 2.84 (2.79b) 0.0 / 101008.455 mm, Bullet NEXT default 0.0 / 101664.122 mm, Bullet NEXT, all collision features 0.0 / 8.054 mm, Soft Step 0.0 / 55.343 mm, Jolt 5.6 0.0 / 9.99 mm |
| TS.5 Replay | PASS | same hash at 1, 2, 4, 8 threads: Bullet NEXT, all collision features yes, Soft Step yes; recordings play back frame by frame: yes; browser = PC on the rally scenes: Bullet NEXT default 4/4, Bullet NEXT, all collision features 4/4, Soft Step 4/4, Jolt 5.6 4/4 |
| TS.6 Engines | PASS | 20 of 20 runs without NaN, blow-up or fall; Chrono: heightfields skipped, as planned |
| BS.1 Rally stage cost | record | table below |
| BS.2 Heightfield or mesh | record | table below |

### BS.1 and BS.2: the 60 s rally stage (one car, 2 km of rolling terrain), whole-run time in ms

| Engine | Heightfield terrain | The same terrain as a triangle mesh |
| --- | --- | --- |
| Bullet 2.84 (2.79b) | 34.0 | 35.2 |
| Bullet NEXT default | 28.3 | 38.5 |
| Bullet NEXT, all collision features | 150.7 | 133.5 |
| Soft Step | 143.6 | 130.1 |
| Jolt 5.6 | 59.9 | 64.2 |

### TS.2: scenes whose final hash changed against the reference

| Engine | Scenes |
| --- | --- |
| Bullet NEXT, all collision features | body_types, bounce, joint_grid, joints_six, kinematic_switch, mass_ratio, mesh_slide, pile_1000, projectile, pyramid_10, pyramid_15, sensors_filters, settings, shape_zoo, soft_body, stack_10, stack_20, tunnel_100 |
| Soft Step | joint_grid, joints_six, pendulum, rb_constraints |

## What Stage 0 found and fixed

1. **Bounces gained energy (planned, CS.1).** Upstream Bullet corrects a shallow overlap in the
   velocity, at the contact ERP. F1 had set that ERP to 0.8, as 2.79b sets it, but Bullet 2.84 used
   0.8 only for deep overlaps pushed out by split impulse. Pushing shallow overlaps out 4 times
   harder in the velocity sent a ball set to restitution 0.9 back at 1.05. Four rules were tried
   (`btContactSolverInfo::m_bnContactErpRule`):
   - Rule 1, Bullet 2.84's own: bounces came back as in 2.79b (0.94), but a platform moving under
     falling boxes (kinematic_switch) settled deeper than F1 allows.
   - Rule 2, every overlap by split impulse: bounces exact, but the 1,240-box pyramid sank (87 mm
     against 3 mm).
   - Rule 3, an impact's overlap by split impulse, resting contacts as upstream: bounces exact and
     F1 passed, but a box sliding over terrain rocked on its 0.8 velocity correction and rose at
     0.059 m/s (TS.3 allows 0.05).
   - Kept, rule 4: an impact's overlap by split impulse (adds no speed), resting contacts as
     Bullet 2.84 corrected them (the ERP, 0.2). Bounces leave at exactly their restitution (0.3,
     0.6, 0.9), the sliding box rises at 0.0001 m/s, and every F1, F3c and Stage 0 test passes.
     The BGE patch S0.1 sets it for Bullet NEXT scenes.
2. **Continuous collision launched fast bodies off triangle terrain (found by the rally scenes).**
   A box, a ball or a car-sized box sliding at 30-50 m/s over a flat heightfield or mesh was thrown
   up at 5-10 m/s, with all the collision features on. The sweep that stops fast bodies grazed the
   shared edge of two triangles and took the edge's direction as the surface; leaning back against
   the motion, it looked like an impact, and the speed removal threw the body up. Sweeps against
   meshes and heightfields with edge data now use the face normal of the triangle hit, so sliding
   along a surface is never an impact: 0.000 m/s at 8, 30 and 50 m/s, on heightfields and meshes.
   The 8 m/s bump test of F1 (mesh_slide) went from 0.001 to 0.000 m/s.
3. **The Lab had no terrain.** A heightfield shape (`"shape": "heightfield"`) with a portable
   generator (plane or rolling value noise, the same heights on every engine and platform),
   drawn by the viewer, on legacy, Bullet NEXT and Jolt (Chrono reports a skip).
4. **Soft Step joints, after the contact change.** With rule 4, PGS holds the six joint types
   straighter (worst angle 1.0 mrad), and Soft Step's soft angular rows leaned more (1.6 mrad at
   60 substeps per second), failing B3.1a. Angular joint rows (a hinge's locked rotations, motors,
   angular limits) now keep Bullet's ERP on every substep, while linear rows stay soft, which is
   what prevents energy gain in chains. Soft Step's worst angle is now 0 on joints_six, joint
   stiffness went from 120 to 200 Hz, and T3.4 and B3.1 pass again (re-run in
   docs/F3C_GATE_REPORT.md).
5. **No way to recentre a large world.** `btNextShiftWorld` moves every body, its interpolation
   transform, motion state and bounds by one offset (soft bodies by `btSoftBody::translate`).
6. **Heightfield edge data was mostly shared (found by the undefined-behaviour checker).** The
   edge data that stops bumps is kept per triangle under a key made of a part number and an
   index. Bullet's key leaves 4 bits for the part, enough for meshes, but a heightfield numbers
   its triangles by column, so past column 7 the key overflowed and triangles 8 columns apart
   shared one entry: a 512 by 512 terrain kept 16,352 entries for its 522,242 triangles, and
   rolling terrain got its neighbours' edge angles. Heightfields now have their own key layout
   (`btTriangleInfoMap::bnKey`); the same terrain keeps 522,242 entries. Flat terrain and meshes
   were not affected, so no test result or hash changed; the rally car touches the ground only
   through its wheel rays. Stage 1 adds a car body sliding on rolling terrain, which would
   have shown it.

## Checks

- Address, leak and undefined-behaviour checkers: 29 scenes (all but the two largest, plus
  rally_stage_mesh) on next, nextf, nexts and Jolt, 116 runs: clean, after fix 6.
- Thread checker: the rally scenes and the joint, stack and mesh scenes on nextf and nexts at 4
  threads, 16 runs: clean, and the same hashes as one thread.
- The browser build gives the native hashes on the rally scenes (TS.5, and again after fix 6).
- Engine patch S0.1: CcdPhysicsEnvironment.cpp compiles with and without the Bullet NEXT
  backend; it is in `integration/bge279/` and not applied.

## Notes

- Re-run on the Stage 1 build (28 Oct 2026): **PASSED**, TS.1 to TS.6. Stage 1's collision fixes
  changed the slide scenes' paths with the collision features; the largest upward speed sliding at
  30 and 50 m/s is now 0.000 m/s on both Bullet NEXT solvers (0.0001 before).

- The F1 gate now runs on 30 scenes (the F3c and rally scenes joined the Lab). Its energy check
  skips scenes whose vehicle engines do work (vehicle, rally_stage).
- The rally stage costs Bullet NEXT with its collision features about 0.04 ms a frame (150 ms
  over 3,600 frames), five times its default: continuous collision sweeps the car every frame.
  It is still a small part of a 16.7 ms frame.
- Times under 10 ms in B3.1 vary by up to 1.5x from run to run on this shared 2-CPU machine
  (pendulum: Soft Step 1.6 ms in F3c, 3.7 ms here). The PC session repeats them.
- Chrono (a reference engine only) gives one of two results for the bounce scene depending on how
  it is started (the folder names passed to it): its run depends on memory layout. No Bullet
  NEXT result is affected; it is noted for Stage 1, where Chrono becomes the vehicle reference.
  The gate now passes the scene and output folders to Chrono, so its recordings land in the
  gate folder.
- Jolt's own wheeled vehicle drives the same script differently (faster, other paths); its
  rally_stage numbers are not a like-for-like comparison of car models, only of engine cost.
- TS.4 without recentring: Bullet 2.84 and Bullet NEXT without its features drift 101 m because
  their bodies are thrown up by the terrain edges (TS.3), so the paths split; this is not
  precision. With the features on, single precision 10 km from the origin keeps paths within
  about 1 cm (Soft Step 5.5 cm) of the same run at the origin; recentring makes them identical.
  Rally FX should recentre (or the game should run near the origin).
