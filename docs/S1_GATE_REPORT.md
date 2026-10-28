# Stage 1 gate report (what Rally FX needs from Bullet NEXT)

Platform: Linux x86_64 cloud machine, 2 CPUs, GCC 13. Run with `tools/run_s1_gate.py --build
build-lab --out gate_s1 --chrono <bin> --rigs <rigs.json> --f1 <f1 run> --f3c gate_f3c --s0 gate_s0`
(raw outputs in `gate_s1/`, not committed); tables made with `tools/s1_report.py`. The Chrono rigs
come from `tools/run_chrono_rigs.sh`. Windows (MSVC) runs in CI on the push; the PC session comes
at the gate.

**Stage 1 gate (cloud part): PASSED.** T1S.1 to T1S.7 pass; B1S.1 to B1S.3 are recorded. Rally
FX can now load its whole 3 km world as one heightfield with no edge bake, see every crash in
full (contacts and continuous collision), sweep its car body against thin obstacles, and rely on
threaded runs repeating. Two test rules were revised on the way (below). Still open: the Windows
CI run, the PC session, and comparing whole tyre and drivetrain curves with Rally FX's bench
output; the engine patches stay unapplied until you give the go.

## Tests

| Test | Result | Measured |
| --- | --- | --- |
| T1S.1 Edges from the heights | PASS | rally_stage (512 x 512): 522,242 triangles, largest angle difference 0 rad, 0 mismatches, map entries 0; rally_slide (320 x 320): 203,522 triangles, largest angle difference 0 rad, 0 mismatches, map entries 0 |
| T1S.2 Car body on terrain | PASS | speed off the 15 % slope: Bullet 2.84 (2.79b) 11.6977 m/s, Bullet NEXT default 11.1204 m/s, Bullet NEXT, all collision features 0.0074 m/s, Soft Step 0.0028 m/s, Jolt 5.6 0.002 m/s; heightfield against mesh, first 0.5 s / after 10 s (10 um nudge): Bullet NEXT, all collision features 0.0005 mm / 7.7 mm (27.23 mm), Soft Step 0.0006 mm / 20.7 mm (21.24 mm), Jolt 5.6 2.779 mm / 3.6 mm (36.31 mm) |
| T1S.3 Full-size terrain | PASS | 3001 x 3001 at 1 m: edges ready in 0.0 s, 0.0 MB (baked: 23.5202 s, 1024.1 MB); speed off the slope: Bullet 2.84 (2.79b) 11.6681 m/s, Bullet NEXT default 11.0816 m/s, Bullet NEXT, all collision features 0.0098 m/s, Soft Step 0.0007 m/s, Jolt 5.6 0.0053 m/s |
| T1S.4 Crash report | PASS | reported impulses against momentum change, worst: Bullet NEXT default 1.4e-07, Bullet NEXT, all collision features 1.9e-07, Soft Step 1.1e-07; bodies through a wall or trunk: Bullet 2.84 (2.79b) 1 of 5, Bullet NEXT default 1 of 5, Bullet NEXT, all collision features 0 of 5, Soft Step 0 of 5, Jolt 5.6 0 of 5 |
| T1S.5 Threads under load | PASS | 60 runs at 8 threads beside two busy loops: pile_1000 nextf 1 hash, pile_1000 nexts 1 hash, rally_body nextf 1 hash, rally_body nexts 1 hash, rally_crash nextf 1 hash, rally_crash nexts 1 hash |
| T1S.6 Chrono reference | PASS | Chrono on 34 scenes from two folders, forward and reversed: 0 differ; rigs: run (table below) |
| T1S.7 No regressions | PASS | F1 PASSED, F3c PASSED, Stage 0 PASSED; hashes changed against Stage 0: Bullet 2.84 (2.79b) 0, Bullet NEXT default 0, Bullet NEXT, all collision features 6, Soft Step 6, Jolt 5.6 0 |
| B1S.1 Edge setup | record | table below |
| B1S.2 Car body step cost | record | table below |
| B1S.3 Tyre rigs | record | table below |

### B1S.1: switching active edges on for a rolling field at 1 m

| Samples | Triangles | Baked: time, extra memory | Worked out: time, extra memory |
| --- | --- | --- | --- |
| 513 x 513 | 524,288 | 0.5113 s, 16.1 MB | 0.0 s, 0.0 MB |
| 1025 x 1025 | 2,097,152 | 2.2834 s, 64.1 MB | 0.0 s, 0.0 MB |
| 3001 x 3001 | 18,000,000 | 23.5202 s, 1024.1 MB | 0.0 s, 0.0 MB |

### B1S.2: the car body on rolling terrain for 10 s, whole-run time (best of 3)

| Engine | Heightfield (edges worked out) | The same ground as a mesh (baked edges) | Ratio |
| --- | --- | --- | --- |
| Bullet NEXT, all collision features | 69.8 ms | 105.7 ms | 0.66x |
| Soft Step | 62.2 ms | 100.5 ms | 0.619x |

### T1S.6 and B1S.3: Chrono's tyres on tarmac, steady state

| Tyre | Load | Peak grip, braking/driving (at slip) | Peak grip, cornering (at angle) | Cornering stiffness per unit load | Grip left at 4 degrees with 10 % slip | Time per evaluation |
| --- | --- | --- | --- | --- | --- | --- |
| TMeasy, BMW E90 front (225/45 R17) | 2 kN | 1.2483 (0.11) | 1.1509 (7.5 deg) | 26.75 /rad | 68% | 0.199 us |
| TMeasy, BMW E90 front (225/45 R17) | 3 kN | 1.2326 (0.11) | 1.1112 (7 deg) | 25.96 /rad | 68% | 0.199 us |
| TMeasy, BMW E90 front (225/45 R17) | 4 kN | 1.2169 (0.10) | 1.0722 (7 deg) | 25.17 /rad | 69% | 0.199 us |
| Pacejka 2002, sedan (245/40 R18) | 2 kN | 1.2537 (0.50) | 1.0706 (20 deg) | 21.55 /rad | 66% | 0.309 us |
| Pacejka 2002, sedan (245/40 R18) | 3 kN | 1.2111 (0.50) | 1.0291 (20 deg) | 20.05 /rad | 67% | 0.309 us |
| Pacejka 2002, sedan (245/40 R18) | 4 kN | 1.168 (0.50) | 0.9875 (20 deg) | 18.21 /rad | 68% | 0.309 us |

### T1S.6: Chrono's shafts as the Evo VI's 4WD, first gear, full throttle, split grip (left 0.3, right 0.9)

| Locking | Torque at the wheels FL, FR, RL, RR (Nm) | Total | Front share | Torque bias front / rear |
| --- | --- | --- | --- | --- |
| open diffs | 280, 581, 280, 581 | 1723 | 50% | 2.07 / 2.07 |
| Rally FX settings, full throttle | 280, 842, 280, 842 | 2244 | 50% | 3.00 / 3.00 |
| all locked | 280, 842, 280, 842 | 2244 | 50% | 3.00 / 3.00 |

### T1S.7: scenes whose final hash changed against Stage 0

| Engine | Scenes |
| --- | --- |
| Bullet NEXT, all collision features | mesh_slide, rally_far, rally_far_shift, rally_slide, rb_shapes, shape_zoo |
| Soft Step | mesh_slide, rally_far, rally_far_shift, rally_slide, rb_shapes, shape_zoo |

## What Stage 1 delivers

1. **Heightfield edges worked out from the heights (C1S.1).** `btNextEnableActiveEdges` on a
   heightfield no longer bakes a map: a contact works out its triangle's edge angles from the
   triangles around it when it needs them, with the bake's own code (`btNextFindTriangleInfo`). The
   results are the bake's, bit for bit, on every triangle of two terrains. A 3 km field at 1 m now
   needs no time and no memory for its edge data (baked: 24 s and 1 GB), so Rally FX can load its
   whole world as one heightfield, without tiles. On the car body the worked-out edges are cheaper
   than a mesh's baked map (0.62-0.66x its time).
2. **Crash report (C1S.3).** Every impulse a hit makes is now reported, on both solvers:
   `btNextContactImpulse` (normal and friction, the step's total, also with Soft Step's substeps)
   and `btNextCcdHits` (the momentum continuous collision removes at the time of impact, which was
   reported nowhere). In the crash scene, the reported impulses add up to each body's change of
   momentum within 2e-7.
3. **Continuous collision for compound bodies.** A body made of several hulls (a car body) was
   never swept, so nothing stopped it entering a thin obstacle at speed. Each convex child is now
   swept and the earliest hit counts. In the crash scene nothing passes through a wall, a 10 cm
   plank or a tree trunk with the features on (2.79b's Bullet: a crate goes through the plank).
4. **New scenes:** rally_tilt (frictionless slides across a 15 % slope), rally_body and
   rally_body_mesh (an Evo VI-sized car body thrown onto rolling terrain, heightfield and mesh),
   rally_big (the slope 3 km wide at 1 m), rally_crash (car bodies into walls at 10, 30 and 60 m/s
   and into a trunk at 30 m/s, and a crate into a plank at 60 m/s). `bnl_run --track-every` and
   the `report_impulses` scene setting go with them.
5. **Chrono vehicle rigs (C1S.6).** Chrono::Vehicle's own tyre force code (TMeasy, Pacejka 2002)
   and a 4WD driveline built from Chrono's shaft elements, run at set loads and slips; the tables
   above, and Rally FX's figures beside them below.

## What Stage 1 found and fixed

1. **A car body far from a heightfield's centre got contacts metres deep (rally_big).** SAT for a
   hull against a triangle took an axis from two nearly parallel edges (a car's sides along the
   grid lines), whose direction is rounding noise, and added a point where the two edge lines met,
   far outside the edges: 3.4 m deep, pointing down. Near-parallel edge pairs (within 0.06 degrees)
   no longer give an axis, and an edge-edge point farther apart than the overlap found is dropped.
2. **An axis in the triangle's plane clipped the triangle against the side of the hull.** When the
   hull overlapped a triangle's side less than it sank into its face, SAT chose the in-plane axis;
   the clipping then put the contact points on a side face of the hull, with the normal along the
   ground. Such pairs take the GJK path, whose normal the active-edge correction sets right. With
   both fixes the car body far out slides at 0.01 m/s off the plane (before: 0.27 m/s, a box 0.17).
3. **The same ground as a heightfield and as a mesh gave paths 9-17 cm apart (rally_body).** The
   contacts were the same, but the solver met them in the order the shape lists its triangles,
   and the two shapes list them differently. The points of a mesh contact are now put in an order
   of their own (depth, then position, then normal). The two now move identically until rounding
   noise (frame 37 or later), and stay within the scene's own sensitivity (below).
4. **A third of the car body's collision time went on a box that is not there.** For SAT, each
   triangle is wrapped in a polyhedron, and Bullet looked for an inner box in it with up to 1,024
   containment tests; a triangle has no inside, and on sloped ground none was found until the
   last step. Flat polyhedra now get a point at once (the box only lets SAT skip axes, so results
   are unchanged; rally_tilt 30 % faster, the same to the bit).
5. **The crash report raced in the threaded world (found by the F1 gate).** The CCD hits were
   collected inside the parallel move pass; the F1 gate's two-thread run lost its output once. They
   are now gathered after it, in body order, on one thread.

## Revised on the way

- **T1S.2, heightfield against mesh after 10 s.** Planned: within 5 cm. The car body tumbles, and
  the scene is chaotic: nudging the car 10 um moves its path after 10 s by 1-11 cm on every engine
  (Jolt too). A fixed 5 cm would test luck. The test now asks for the same path to 1 um for the
  first 0.5 s (through the first contacts) and, after 10 s, no more difference than the 10 um
  nudge makes. It still catches the bug above (before the fix: 16.5 cm against a 2.7 cm spread).
  Soft Step passes it narrowly (20.7 mm against 21.2 mm).
- **"Mesh scenes unchanged" (T1S.7).** Fixes 1-3 change contacts between hulls and triangles, so
  six scenes with hulls on meshes or terrain have new results with the collision features
  (listed above); legacy and Jolt are unchanged, and the F1, F3c and Stage 0 gates pass on this
  build.
- **F1's T1.5 leaves out rally_body.** 2.79b's Bullet rests the car body's hulls on their 4 cm
  collision margin (no overlap reported) after being thrown by the terrain's edges, while Bullet
  NEXT's hull clipping touches the real hull surface (about 1 mm at rest): the comparison has no
  baseline, as with tunnel_100 and projectile (2.32 against 1.03 mm on average).

## The Chrono reference

Chrono's results depended on what was in memory before its run, not only on the scene: the
bounce scene gave three different results depending on the folder name it was read from (25
characters gave another result than 17 or 33), and setting glibc's MALLOC_PERTURB_ (which fills
fresh memory with a pattern) changed them as well. That is a read of memory nobody wrote. It
comes from Chrono's build with SIMD on (the default, compiled for the machine's own CPU): the
same code built with SIMD off gives one result for every folder name and every MALLOC_PERTURB_
value. Bullet NEXT's engines give one result in all of these cases. The exact line in Chrono was
not found (valgrind cannot run the SIMD build on this machine's AVX-512 instructions); the Lab's
Chrono engine is now built against a Chrono build with SIMD off (`tools/build_chrono_engine.sh`
says so), and T1S.6 passes with it. Its hashes changed with that build; Chrono is a reference only.

## Rally FX against Chrono

The Rally FX prototype's own bench output (its tyre and drivetrain curves as data) is not in this
workspace; the comparison below uses the figures in its plan (P1.4, P1.5). Comparing whole curves
needs that output; it is the first item for the Rally FX side.

| Quantity | Rally FX | Chrono | Difference | Explained |
| --- | --- | --- | --- | --- |
| Tarmac cornering stiffness per unit load | 19 /rad | Pacejka 2002 sport tyre (245/40 R18): 20.0 /rad; TMeasy road tyre (225/45 R17): 26.0 /rad (3 kN) | -5 %; -27 % | The TMeasy tyre is a low-profile road tyre, stiffer than a rally tyre on 15" rims (205/65 R15 in the data sheet); the Pacejka sport tyre is the closer reference and agrees |
| Gravel cornering stiffness per unit load | 5.8 /rad | none: Chrono's TMeasy and Pacejka are hard-ground models | - | Loose ground in Chrono is its soil model (SCM), a different kind of model; not compared |
| Peak grip on tarmac | not stated in the plan | 1.21-1.25 braking, 1.03-1.11 cornering (3 kN) | - | Needs Rally FX's bench output |
| Driveline on split grip (0.3 / 0.9), first gear, full throttle | Rally FX's settings (centre 1,000 Nm + 20 Nm per rad/s, plates 40 Nm + 0.25 and 60 Nm + 0.5 of axle torque) | both axles lock: torque bias 3.0 (the grip ratio), 2,244 Nm at the wheels, 30 % more than open diffs | - | Run the same rig in Rally FX's core to compare |

New rows for Rally FX's learning log: the tarmac tyre agrees with a sport tyre's Pacejka data
within 5 %; and the drivetrain's locking settings fully lock both axles on split grip at full
throttle in Chrono's shaft model (worth checking in Rally FX's bench, as a fully locked front axle
pushes the car wide).

## Checks

- Address, leak and undefined-behaviour checkers: the 12 scenes with hulls on terrain, meshes and
  crashes (the five new ones among them) on next, nextf, nexts and Jolt, 48 runs: clean.
- Thread checker: rally_crash, rally_body, rally_tilt and pile_1000 on nextf and nexts at 4
  threads, 8 runs: clean.
- T1.7 (Chrono's own collision and contact tests with the patch): re-run with the Stage 1 patch,
  the same results as before (11 of 11 give the stock result; Chrono's mesh-slide bump with the
  features on 0.120 m/s, was 0.118).
- The browser build gives the native hashes (CI checks every scene on the push).

## Notes

- Hull clipping and SAT remain the largest cost for a hull car body on terrain (rally_tilt with all
  collision features: 0.7 s for 600 frames and four bodies, against 0.03 s without them); making
  them cheaper is Stage 2 (P-C). Rally FX's car touches the ground through its wheels, so this cost
  only comes in crashes and rollovers.
- Chrono's Pacejka 2002 holds its peak force past the peak (it clamps the slip term before the
  curve falls), so its "slip at peak" reads as the end of the range.
- The rigs' times per tyre evaluation (0.2-0.3 us) are Chrono's force functions alone; Rally FX's
  tyre step (1.2 us in its P0 bench) includes its relaxation and ground terms.
