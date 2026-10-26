# F0 gate report

Platform: Linux x86_64, 2 CPUs, compiler from build `build`.

## Tests

| Test | Result |
| --- | --- |
| T0.1 Build (GCC, cloud) | PASS |
| T0.2 Symbol isolation | PASS |
| T0.3 Rename fidelity | PASS |
| T0.4 Determinism (5 of 5 runs) | PASS |
| T0.5 Compare self-test | PASS |
| T0.6 Coverage (12 points) | PASS |
| T0.7 Next baseline runs every scene | PASS |
| T0.8 BGE-in-the-loop | Pending: Windows PC at project end |

T0.2 output: `legacy ball z = 0.500000, next ball z = 0.500000
symbol isolation OK`

T0.5 output: `selftest gate_f0/stack_10.legacy.bnr: identical=yes exact_catches_1ulp=yes tolerance_passes_1ulp=yes`

### T0.3 rename fidelity and T0.7 next baseline (per scene)

| Scene | Legacy = original | Next vs original (first divergence) |
| --- | --- | --- |
| stack_10 | bit-identical | frame 2 body 1 pos.x -8.85425925e-06 vs -1.64121957e-05 |
| pyramid_10 | bit-identical | frame 6 body 1 pos.y -2.35598009e-07 vs -2.27958253e-07 |
| shape_zoo | bit-identical | frame 41 body 4 pos.x -0.00172107737 vs -0.00036596242 |
| joints_six | bit-identical | frame 1 body 1 pos.x -6.00042677 vs -6.00014448 |
| vehicle | bit-identical | frame 407 body 2 lin.x -0.00105451583 vs -0.00137177377 |
| character | bit-identical | frame 1 body 7 pos.y 0.0500000007 vs 0.0500000045 |
| soft_body | bit-identical | frame 21 body 3 pos.y -1.26393079e-05 vs -1.25870929e-05 |
| sensors_filters | bit-identical | frame 58 body 4 pos.z 1.59044242 vs 1.56178224 |
| projectile | bit-identical | frame 3 body 6 pos.x 15.0877647 vs 9.96276188 |
| mesh_slide | bit-identical | frame 5 body 2 ang.y 0.365205765 vs 0.365205795 |
| body_types | bit-identical | frame 34 body 3 pos.x 2.86090231 vs 2.860852 |
| kinematic_switch | bit-identical | frame 54 body 1 pos.z 0.189220145 vs 0.156880647 |
| settings | bit-identical | frame 4 body 1 pos.y -4.16439036e-07 vs -1.08641575e-06 |

## Benchmarks

### B0.1 step time (best of 5 runs, single thread, whole scene)

| Scene | Frames | Legacy ms | Next ms | Next / legacy |
| --- | --- | --- | --- | --- |
| stack_10 | 300 | 3.49 | 2.93 | 0.84 |
| pyramid_10 | 300 | 26.52 | 31.00 | 1.17 |
| shape_zoo | 300 | 9.12 | 13.09 | 1.44 |
| joints_six | 480 | 4.42 | 4.16 | 0.94 |
| vehicle | 480 | 2.66 | 2.09 | 0.79 |
| character | 480 | 11.98 | 2.36 | 0.20 |
| soft_body | 300 | 95.28 | 9.34 | 0.10 |
| sensors_filters | 240 | 1.93 | 1.00 | 0.52 |
| projectile | 120 | 0.82 | 0.73 | 0.89 |
| mesh_slide | 240 | 2.37 | 2.65 | 1.12 |
| body_types | 240 | 1.53 | 0.97 | 0.63 |
| kinematic_switch | 240 | 1.95 | 1.35 | 0.69 |
| settings | 300 | 28.93 | 26.00 | 0.90 |

### B0.2 quality baseline

| Scene | Backend | Max penetration (mm) | Max joint drift (mm) | Energy start (J) | Energy end (J) |
| --- | --- | --- | --- | --- | --- |
| stack_10 | legacy | 5.85 | 0.00 | 490.4 | 489.9 |
| stack_10 | next | 5.83 | 0.00 | 490.4 | 489.8 |
| pyramid_10 | legacy | 43.11 | 0.00 | 1887.7 | 1811.8 |
| pyramid_10 | next | 42.28 | 0.00 | 1887.7 | 1811.8 |
| shape_zoo | legacy | 93.49 | 0.00 | 235.3 | 31.4 |
| shape_zoo | next | 93.49 | 0.00 | 235.3 | 31.4 |
| joints_six | legacy | 22.89 | 80.00 | 470.0 | 427.6 |
| joints_six | next | 22.89 | 80.00 | 469.6 | 427.6 |
| vehicle | legacy | 0.00 | 0.00 | 9406.9 | 6100.6 |
| vehicle | next | 0.00 | 0.00 | 9406.9 | 6110.5 |
| character | legacy | 88.78 | 0.00 | 0.0 | 0.0 |
| character | next | 150.00 | 0.00 | 0.0 | 0.0 |
| soft_body | legacy | 15.64 | 0.00 | 9.8 | 4.9 |
| soft_body | next | 15.64 | 0.00 | 9.8 | 4.9 |
| sensors_filters | legacy | 161.21 | 0.00 | 156.9 | -18.2 |
| sensors_filters | next | 161.21 | 0.00 | 156.9 | -18.2 |
| projectile | legacy | 135.20 | 0.00 | 9896.3 | 7646.9 |
| projectile | next | 135.20 | 0.00 | 9896.3 | 3824.1 |
| mesh_slide | legacy | 17.42 | 0.00 | 70.0 | 12.8 |
| mesh_slide | next | 16.31 | 0.00 | 70.0 | 13.7 |
| body_types | legacy | 48.51 | 0.00 | 113.6 | 31.3 |
| body_types | next | 53.60 | 0.00 | 113.6 | 30.5 |
| kinematic_switch | legacy | 164.74 | 0.00 | 284.4 | 9.8 |
| kinematic_switch | next | 156.43 | 0.00 | 284.4 | 9.7 |
| settings | legacy | 14.91 | 0.00 | 1888.1 | 1812.7 |
| settings | next | 15.48 | 0.00 | 1888.1 | 1812.8 |

**F0 gate (cloud part): PASSED.** T0.8 and the MSVC build remain for the Windows PC.
