# F3c gate report (Soft Step contacts)

Platform: Linux x86_64 cloud machine, 2 CPUs, GCC 13; WebAssembly with Emscripten 3.1.47 (Node 22).
Run with `tools/run_f3c_gate.py --build build-lab --out gate_f3c` (raw outputs in `gate_f3c/`, not
committed); tables made with `tools/f3c_report.py`. Windows (MSVC) runs in CI (green on `d12c05b`); the
PC tests come at project end.

**F3c gate (cloud part): PASSED, with B3.1 revised.** T3.1 to T3.6 pass. B3.1 passes under the
revised definition agreed on 27 October (below); as first written it could not pass, and the
reason is recorded here. CI is green on `d12c05b`: Soft Step reaches the Linux hash on every
scene on Windows (MSVC) and in the browser, and in HARFANG on Linux. Still open: the BGE
settings patch (C3.8, written and checked, applied to the engine when you give the go) and the
PC checks at project end.

## What Soft Step is

A second solver, `btSoftStepConstraintSolver`, next to Bullet's sequential impulse solver (PGS).
It is chosen per world; the old solver stays the default, so old games are unchanged. Its
structure follows Box2D v3: collision runs once per frame, then the solver runs several short
substeps. In each it integrates velocities, warm starts, solves contacts and joints with soft,
spring-like position correction, moves the bodies and relaxes (solves again without the
correction). A restitution pass follows the substeps. Contacts keep their anchors for the
frame and update their depth from the bodies' motion; joints are rebuilt every substep with
Bullet's own joint code, their locked axes solved together as one block. The solver leaves the
end position in the body's interpolation transform, so continuous collision detection still
applies.

Defaults: 540 substeps per second (9 per 60 Hz frame), contact stiffness an eighth of the
substep rate (67.5 Hz), contact damping ratio 10, push-out at most 3 m/s; joints corrected at
120 Hz with damping ratio 5 (200 Hz since Stage 0, see the re-run at the end). In the Lab: engine `nexts`, scene setting `"solver": "soft_step"`.

## Tests

| Test | Result | Measured |
| --- | --- | --- |
| T3.1 Legacy untouched | PASS | legacy/ unchanged since F0: True; legacy recordings = F0 goldens: 16/16; other engines = reference: 104/104 |
| T3.2 Stack stability | PASS | stack_20: 2.08 mm, 0.0053 m/s (PGS 182.73 mm, 27.3059 m/s, moved 43.421 m); pyramid_15: 0.18 mm, 0.0005 m/s (PGS 0.92 mm, 0.0233 m/s, moved 0.019 m) |
| T3.3 Mass ratio | PASS | moved 0.0118 m, tilt 0.0167 rad, penetration 11.2 mm (PGS moved 7.318 m, tilt 2.756 rad) |
| T3.4 Joint behaviour | PASS | joints_six: drift 33.21 mm (PGS 79.999), angle 0.00226 rad (PGS 0.00285), energy gain -0.742% (PGS -0.458%); rb_constraints: drift 0.067 mm (PGS 0.09), angle 0.00049 rad (PGS 0.00049), energy gain 0.003% (PGS -0.006%); joint_grid: drift 134.678 mm (PGS 3511.45), angle 0 rad (PGS 0), energy gain -0.002% (PGS -0.016%); pendulum: drift 0.341 mm (PGS 1198.88), angle 0 rad (PGS 0), energy gain -0.004% (PGS -0.034%) |
| T3.5 Restitution | PASS | e 0.0: measured 0.0 (PGS 0.1485), rise 0.0 m vs 0.0 m; e 0.3: measured 0.3 (PGS 0.4485), rise 0.1846 m vs 0.1837 m; e 0.6: measured 0.6 (PGS 0.7485), rise 0.7423 m vs 0.7347 m; e 0.9: measured 0.9 (PGS 1.0485), rise 1.6733 m vs 1.653 m |
| T3.6 Determinism | PASS | 26/26 scenes give the same hash at 1, 2, 4 and 8 threads |
| B3.1 Equal-quality cost | PASS | joints: PASS, stacks: PASS, resting depth: PASS (tables below) |
| B3.2 Substep sweep | record | table below |

### B3.1a Joints: the lowest substep rate at which Soft Step drifts no more than PGS (10 iterations)

| Scene | Substeps per second | Worst drift (mm), Soft Step / PGS | Time (ms), Soft Step / PGS | Time / PGS time |
| --- | --- | --- | --- | --- |
| joints_six | 60 | 76.668 / 79.999 | 3.1 / 4.0 | 0.774x |
| rb_constraints | 60 | 0.104 / 0.09 | 43.2 / 56.4 | 0.766x |
| joint_grid | 180 | 484.219 / 3511.45 | 320.4 / 289.3 | 1.107x |
| pendulum | 60 | 11.519 / 1198.88 | 1.6 / 1.7 | 0.905x |

### B3.1b Stacks: the cheapest setting of each solver that keeps the stack standing still (T3.2 / T3.3 limits)

| Scene | Soft Step: substeps per second, time (ms) | PGS: iterations, time (ms) | Time / PGS time |
| --- | --- | --- | --- |
| stack_20 | 360, 36.4 | not reached at 160 (220.0 ms at 160) | below 0.165x |
| mass_ratio | 240, 11.7 | not reached at 160 (80.2 ms at 160) | below 0.146x |

### B3.1c Resting depth at the defaults (bound 2.5 mm), with PGS (10 iterations) beside it

| Scene | Resting penetration (mm), Soft Step / PGS | PGS: how far a box moved (m) | Time (ms), Soft Step / PGS | Time / PGS time |
| --- | --- | --- | --- | --- |
| stack_10 | 0.99 / 0.67 | 0.502 | 5.5 / 2.6 | 2.093x |
| pyramid_10 | 0.48 / 0.42 | 0.015 | 56.2 / 28.5 | 1.97x |
| stack_20 | 2.08 / 182.73 | 43.421 | 54.8 / 20.1 | 2.729x |
| rb_stack | 0.42 / 0.1 | 0.014 | 41.9 / 41.8 | 1.004x |
| settings | 0.34 / 0.06 | 0.07 | 34.0 / 39.9 | 0.851x |
| pyramid_15 | 0.18 / 0.92 | 0.019 | 26753.1 / 11136.3 | 2.402x |

### B3.2 Substep sweep: Soft Step at a fixed number of substeps per 60 Hz step

| Scene | 2 substeps: time, penetration, jitter, moved | 4 substeps: time, penetration, jitter, moved | 8 substeps: time, penetration, jitter, moved |
| --- | --- | --- | --- |
| bounce | 0.8 ms, 64.14 mm, 4.9998 m/s, 0.0 m | 0.8 ms, 17.62 mm, 5.0015 m/s, 0.0 m | 1.1 ms, 46.97 mm, 5.165 m/s, 0.0 m |
| joint_grid | 294.6 ms, 68.99 mm, 29.238 m/s, 0.842 m | 396.8 ms, 104.9 mm, 12.8572 m/s, 3.28 m | 551.4 ms, 9.52 mm, 2.8011 m/s, 1.698 m |
| joints_six | 4.2 ms, 0.5 mm, 2.2776 m/s, 0.76 m | 8.9 ms, 0.12 mm, 2.2917 m/s, 0.764 m | 11.9 ms, 0.03 mm, 2.2951 m/s, 0.765 m |
| mass_ratio | 7.6 ms, 339.64 mm, 6.5239 m/s, 3.085 m | 11.7 ms, 58.65 mm, 0.0296 m/s, 0.017 m | 19.8 ms, 14.12 mm, 0.1075 m/s, 0.019 m |
| pyramid_10 | 22.5 ms, 8.56 mm, 0.0039 m/s, 0.026 m | 34.6 ms, 1.96 mm, 0.0002 m/s, 0.02 m | 52.6 ms, 0.63 mm, 0.0011 m/s, 0.014 m |
| stack_20 | 19.3 ms, 178.88 mm, 18.4053 m/s, 14.081 m | 29.1 ms, 13.51 mm, 0.4766 m/s, 0.796 m | 46.2 ms, 2.63 mm, 0.0015 m/s, 0.001 m |

## B3.1: why it was revised

As first written, B3.1 asked for Soft Step's step time "at the same penetration and drift" as
PGS, at no more than 1.5 times PGS's time. Joints and stacks meet that easily. Resting depth
does not, and cannot without a large cost:

- A soft contact is a stiff spring. Under load it settles at a depth of about load × g / ω²,
  where ω is the contact stiffness. Box2D behaves the same way. A box at the bottom of a 20-box
  stack sinks about 2 mm into the one below it; PGS, where it holds the stack at all, drives the
  depth towards zero.
- On small stacks that PGS holds at 10 iterations (10 boxes, a 10-layer pyramid), PGS's resting
  depth is 0.4 to 0.7 mm. Soft Step reaches that only at about 720 substeps per second, at 2.5
  to 3 times PGS's time. Stiffer contacts at fewer substeps start to shake: from a quarter of
  the substep rate, the pyramid jitters and the 1:100 stack tilts.
- The same depth is what keeps the tall stacks standing. At 10 iterations PGS lets the 20-box
  stack fall and the 1:100 stack collapse; at 160 iterations it still does not hold them still.

So on 27 October (agreed with Ayhan) B3.1 became three parts, all required:

- **a. Joints**: per joint scene, the lowest substep rate at which Soft Step's worst drift is no
  worse than PGS's at 10 iterations; its time there at most 1.5 times PGS's.
- **b. Stacks**: the cheapest setting of each solver that keeps the 20-box stack and the 1:100
  stack standing still (the T3.2 and T3.3 limits); Soft Step's time at most 1.5 times PGS's.
- **c. Resting depth**: at the defaults, at most 2.5 mm on stacks of equal boxes (a bound, not
  PGS's figure), recorded beside PGS's depth and time.

The cost of the defaults is recorded, not hidden. Where PGS already holds a small stack, Soft
Step at its defaults takes about 2 to 2.7 times PGS's time for a similar depth (B3.1c). On the
joint scenes the defaults take 1.6 to 3.3 times PGS's time (joints_six 13.4 against 4.1 ms,
joint_grid 622 against 269 ms, pendulum 5.6 against 1.7 ms, rb_constraints 92 against 56 ms) for
drift 1.3 to 3,500 times smaller; B3.1a shows that PGS's own drift is matched at 0.8 to 1.1
times its time. A game that needs less can lower the substep rate.

## Recorded differences (by design)

- **Resting depth under load**: 1 to 2 mm on stacks of equal boxes, more under very heavy loads
  (11 mm under the 100 kg box of the 1:100 stack, which PGS lets fall).
- **Slower push-out**: an overlap is pushed out like a stiff spring over a few frames, not in one
  step as with PGS's split impulse. Impacts leave deeper dents for a moment (rb_constraints:
  17 mm against 5 mm), and a body moved by setting its position each frame (a suspended body in
  BGE, scene kinematic_switch) pushes resting boxes out more slowly.
- **Cost**: each substep repeats the contact and joint work. B3.2 shows time and quality per
  substep count; the substep rate is a per-scene setting.
- **Restitution**: Soft Step bounces at exactly the set restitution (0.0, 0.3, 0.6, 0.9 measured
  0.0, 0.3, 0.6, 0.9); PGS adds about 0.15 to each.

## What F3c found and fixed on the way

- A first warm start at 85 % of the last frame's impulses let stacks collapse; contacts now carry
  all of it, joints carry theirs across frames too (stored on the constraint).
- Joints with more than six rows (the 6DOF spring constraint) were cut at six; now up to 18.
- Solving each manifold's normals before its friction stopped a box toppling off its pedestal.
- Joints solved before contacts let a hinge drift; contacts go first in both passes.
- Long-lever point-to-point joints drifted when their three rows were solved one by one; their
  locked rows are now solved as one block (LDLᵀ of the effective mass).
- Correcting joints with Bullet's ERP on every substep made the joint grid gain energy at high
  substep rates; joints now use the same soft correction as contacts.
- The world asked each body to integrate itself after the solver had already moved it; the world
  now reads the solver's end position (`bnPredictTransform`), so CCD and the other solvers' paths
  are unchanged (their hashes are the same as before F3c).

## Checks

- Address, leak and undefined-behaviour checkers: no reports, Soft Step on all 26 scenes at one
  and two threads. Thread checker: no reports, 26 scenes at four threads.
- Browser build: Soft Step gives the same hash as the native run on 26 of 26 scenes (next and
  nextf too). HARFANG (Python, headless): the five new scenes reach the Lab's hash on every engine.
  CI now requires `nexts` to match the reference on Linux, Windows and in the browser, and in
  HARFANG.
- `lab/scenes/native_hashes.txt` gains Soft Step and the five new scenes (130 runs); no existing
  line changed.

## BGE settings (C3.8, not applied)

`integration/bge279/c3.8-soft-step-settings.patch`: a third Solver entry, "Soft Step", with its
substep rate, contact stiffness, contact damping and push-out speed in the scene's physics panel,
and `bge.constraints.setSolverType(3)` / `setSoftStepSettings(...)` in Python. Checked against the
2.79b engine tree (DNA builds, the physics, Ketsji and RNA files compile with and without Bullet
NEXT); applied when you give the go.

## Not changed, on purpose

The legacy backend (unchanged since F0, 16 of 16 recordings equal the F0 goldens), PGS and every
other Lab engine (84 of 84 runs at their FL hashes). Soft Step is opt-in.

## Stage 0 re-run (27 Oct 2026)

Stage 0 changed two things this gate measures: PGS contacts with BGE's settings now use contact
rule 4 (see docs/S0_GATE_REPORT.md), and Soft Step's joints are corrected at 200 Hz with angular
rows held at Bullet's ERP. With rule 4, PGS holds the joints straighter than before (worst angle
1.0 mrad on joints_six), and Soft Step's soft angular rows at 60 substeps per second (1.6 mrad)
no longer met B3.1a; holding angular rows at Bullet's ERP brought Soft Step's angle to 0. The
whole gate was re-run on the Stage 0 build: **PASSED.** Times under 10 ms move by up to 1.5x
between runs on this machine.

| Test | Result | Measured |
| --- | --- | --- |
| T3.1 Legacy untouched | PASS | legacy/ unchanged since F0: True; legacy recordings = F0 goldens: 16/16; other engines = reference: 120/120 |
| T3.2 Stack stability | PASS | stack_20: 2.08 mm, 0.0053 m/s (PGS 204.27 mm, 24.3379 m/s, moved 43.445 m); pyramid_15: 0.18 mm, 0.0005 m/s (PGS 3.16 mm, 0.0235 m/s, moved 0.031 m) |
| T3.3 Mass ratio | PASS | moved 0.0118 m, tilt 0.0167 rad, penetration 11.2 mm (PGS moved 2.397 m, tilt 1.679 rad) |
| T3.4 Joint behaviour | PASS | joints_six: drift 16.322 mm (PGS 79.999), angle 0 rad (PGS 0.00104), energy gain -0.739% (PGS -0.458%); rb_constraints: drift 0.024 mm (PGS 0.09), angle 0.00049 rad (PGS 0.00049), energy gain 0.004% (PGS -0.006%); joint_grid: drift 76.3 mm (PGS 3770.64), angle 0 rad (PGS 0), energy gain -0.002% (PGS -0.016%); pendulum: drift 0.202 mm (PGS 1198.88), angle 0 rad (PGS 0), energy gain -0.004% (PGS -0.034%) |
| T3.5 Restitution | PASS | e 0.0: measured 0.0 (PGS 0.0), rise 0.0 m vs 0.0 m; e 0.3: measured 0.3 (PGS 0.3), rise 0.1846 m vs 0.1837 m; e 0.6: measured 0.6 (PGS 0.6), rise 0.7423 m vs 0.7347 m; e 0.9: measured 0.9 (PGS 0.9), rise 1.6733 m vs 1.653 m |
| T3.6 Determinism | PASS | 30/30 scenes give the same hash at 1, 2, 4 and 8 threads |
| B3.1 Equal-quality cost | PASS | joints: PASS, stacks: PASS, resting depth: PASS (tables below) |
| B3.2 Substep sweep | record | table below |

#### B3.1a Joints: the lowest substep rate at which Soft Step drifts no more than PGS (10 iterations)

| Scene | Substeps per second | Worst drift (mm), Soft Step / PGS | Time (ms), Soft Step / PGS | Time / PGS time |
| --- | --- | --- | --- | --- |
| joints_six | 60 | 76.659 / 79.999 | 5.9 / 6.2 | 0.962x |
| rb_constraints | 60 | 0.104 / 0.09 | 67.0 / 86.4 | 0.775x |
| joint_grid | 180 | 484.219 / 3770.64 | 571.9 / 403.6 | 1.417x |
| pendulum | 60 | 11.519 / 1198.88 | 3.7 / 2.6 | 1.419x |

#### B3.1b Stacks: the cheapest setting of each solver that keeps the stack standing still (T3.2 / T3.3 limits)

| Scene | Soft Step: substeps per second, time (ms) | PGS: iterations, time (ms) | Time / PGS time |
| --- | --- | --- | --- |
| stack_20 | 360, 62.5 | not reached at 160 (350.9 ms at 160) | below 0.178x |
| mass_ratio | 240, 19.3 | not reached at 160 (133.2 ms at 160) | below 0.145x |

#### B3.1c Resting depth at the defaults (bound 2.5 mm), with PGS (10 iterations) beside it

| Scene | Resting penetration (mm), Soft Step / PGS | PGS: how far a box moved (m) | Time (ms), Soft Step / PGS | Time / PGS time |
| --- | --- | --- | --- | --- |
| stack_10 | 0.99 / 2.44 | 0.057 | 9.0 / 4.3 | 2.089x |
| pyramid_10 | 0.48 / 1.48 | 0.044 | 104.5 / 44.9 | 2.327x |
| stack_20 | 2.08 / 204.27 | 43.445 | 90.2 / 36.0 | 2.505x |
| rb_stack | 0.42 / 0.1 | 0.014 | 80.4 / 73.5 | 1.094x |
| settings | 0.34 / 0.22 | 0.005 | 50.0 / 37.4 | 1.338x |
| pyramid_15 | 0.18 / 3.16 | 0.031 | 42692.1 / 16813.2 | 2.539x |

#### B3.2 Substep sweep: Soft Step at a fixed number of substeps per 60 Hz step

| Scene | 2 substeps: time, penetration, jitter, moved | 4 substeps: time, penetration, jitter, moved | 8 substeps: time, penetration, jitter, moved |
| --- | --- | --- | --- |
| bounce | 1.1 ms, 64.14 mm, 4.9998 m/s, 0.0 m | 1.3 ms, 17.62 mm, 5.0015 m/s, 0.0 m | 1.9 ms, 46.97 mm, 5.165 m/s, 0.0 m |
| joint_grid | 500.6 ms, 68.99 mm, 29.238 m/s, 0.842 m | 569.1 ms, 104.9 mm, 12.8572 m/s, 3.28 m | 963.9 ms, 69.04 mm, 15.0205 m/s, 1.827 m |
| joints_six | 6.8 ms, 0.5 mm, 2.2776 m/s, 0.76 m | 10.9 ms, 0.12 mm, 2.2917 m/s, 0.764 m | 19.1 ms, 0.03 mm, 2.2957 m/s, 0.765 m |
| mass_ratio | 14.0 ms, 339.64 mm, 6.5239 m/s, 3.085 m | 18.7 ms, 58.65 mm, 0.0296 m/s, 0.017 m | 33.7 ms, 14.12 mm, 0.1075 m/s, 0.019 m |
| pyramid_10 | 37.3 ms, 8.56 mm, 0.0039 m/s, 0.026 m | 53.6 ms, 1.96 mm, 0.0002 m/s, 0.02 m | 88.5 ms, 0.63 mm, 0.0011 m/s, 0.014 m |
| stack_20 | 33.3 ms, 178.88 mm, 18.4053 m/s, 14.081 m | 46.5 ms, 13.51 mm, 0.4766 m/s, 0.796 m | 77.7 ms, 2.63 mm, 0.0015 m/s, 0.001 m |
