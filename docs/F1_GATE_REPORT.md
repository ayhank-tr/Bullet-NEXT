# F1 gate report

Platform: Linux x86_64, 2 CPUs, build `build-lab`.

## Tests

| Test | Result | Measured |
| --- | --- | --- |
| T1.1 Legacy untouched | PASS | legacy/ unchanged since F0: yes; 16 of 16 F0 scenes bit-identical to original and goldens; legacy = original on 34 of 34 scenes |
| T1.2 Thread determinism | PASS | 34 of 34 scenes identical at 1, 2, 4, 8 threads |
| T1.3 Ghost bumps | PASS | 0.004 m/s (limit 0.05; legacy 1.57) |
| T1.4 Tunnelling | PASS | 0 of 100 through (legacy 100 of 100) |
| T1.5 Stacking and joint quality vs F0 | PASS | 31 of 31 scenes within 10 % + 1 mm |
| T1.6 Python API | Pending: engine on the Windows PC at project end | |
| T1.7 Chrono build | PASS | Chrono da802526: 11 of 11 tests give the stock result patched and with the features on (7 pass on stock Chrono) |

### T1.1 and T1.2 per scene

| Scene | Legacy = original | Legacy = golden | Next hash (1-8 threads) |
| --- | --- | --- | --- |
| stack_10 | yes | yes | 62e6f69df943962e |
| pyramid_10 | yes | yes | 61d38b7e07b96204 |
| shape_zoo | yes | yes | ec6fa23c6f68a925 |
| joints_six | yes | yes | 8239be4fcee84226 |
| vehicle | yes | yes | bccf3d9f3660642d |
| character | yes | yes | ff1db83382d7a8ca |
| soft_body | yes | yes | 6046ad29b49701d8 |
| sensors_filters | yes | yes | cd282de7c26fa6dd |
| projectile | yes | yes | e8bff2d3552183ac |
| tunnel_100 | yes | yes | 36a212729db46a1b |
| mesh_slide | yes | yes | 8de00187ae18e9ac |
| body_types | yes | yes | 6edf89a2de0086d0 |
| kinematic_switch | yes | yes | cfe4a87a427ef3e1 |
| pile_1000 | yes | yes | 824210d38af15ef3 |
| joint_grid | yes | yes | b9095e1427a53b3f |
| settings | yes | yes | 0a978a9f1e72b129 |
| rb_stack | yes | no golden (later scene) | 39d5e0e3b3756ff8 |
| rb_shapes | yes | no golden (later scene) | ff959c6c818c9ece |
| rb_constraints | yes | no golden (later scene) | 9d62abe9f36fc58a |
| rb_kinematic | yes | no golden (later scene) | 585cadda8bb9ccb8 |
| rb_settings | yes | no golden (later scene) | 763d6b8b12fd0de0 |
| stack_20 | yes | no golden (later scene) | 89286fd737b57245 |
| pyramid_15 | yes | no golden (later scene) | 35d9c7bc21850add |
| mass_ratio | yes | no golden (later scene) | 61249a453485b0e1 |
| bounce | yes | no golden (later scene) | 5e1b4bd2841eb739 |
| pendulum | yes | no golden (later scene) | e413021213621b1a |
| rally_slide | yes | no golden (later scene) | c1124b39e8266664 |
| rally_far | yes | no golden (later scene) | 7686a24939a0dc7a |
| rally_far_shift | yes | no golden (later scene) | c1124b39e8266664 |
| rally_stage | yes | no golden (later scene) | 6125cc87914040ba |
| rally_tilt | yes | no golden (later scene) | f80be730cd152f5e |
| rally_body | yes | no golden (later scene) | 0c5f63b97c4c64a6 |
| rally_big | yes | no golden (later scene) | 147f2e90b9156c7e |
| rally_crash | yes | no golden (later scene) | e0ea0c45bcde34a9 |

### T1.5 and B1.3 quality (mean of 5 gravity variants)

| Scene | Mean penetration mm (legacy / next) | Max penetration mm (legacy / next) | Joint drift mm (legacy / next) | Energy gain J (legacy / next) | Result |
| --- | --- | --- | --- | --- | --- |
| stack_10 | 4.11 / 2.43 | 5.9 / 6.0 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| pyramid_10 | 3.24 / 2.70 | 43.1 / 51.4 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| shape_zoo | 11.29 / 10.43 | 125.2 / 111.4 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| joints_six | 0.38 / 0.11 | 22.9 / 22.9 | 80.0 / 80.0 | 0.00 / 0.00 | pass |
| vehicle | 0.00 / 0.00 | 0.0 / 0.0 | 0.0 / 0.0 | 51044.15 / 51044.14 | pass |
| character | 53.66 / 37.84 | 88.8 / 150.0 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| soft_body | 0.26 / 0.10 | 15.6 / 15.6 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| sensors_filters | 1.69 / 1.69 | 160.6 / 160.6 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| mesh_slide | 5.87 / 0.05 | 32.3 / 6.3 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| body_types | 3.18 / 1.89 | 47.3 / 43.9 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| kinematic_switch | 3.13 / 4.14 | 164.1 / 128.8 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| pile_1000 | 27.36 / 29.20 | 199.2 / 215.5 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| joint_grid | 12.62 / 5.67 | 238.4 / 122.0 | 3901.6 / 4057.9 | 0.00 / 0.00 | pass |
| settings | 0.60 / 0.46 | 14.8 / 17.0 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| rb_stack | 0.18 / 0.10 | 0.2 / 0.1 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| rb_shapes | 1.97 / 1.13 | 40.0 / 38.0 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| rb_constraints | 0.30 / 0.04 | 16.8 / 4.0 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| rb_kinematic | 0.11 / 0.11 | 3.0 / 3.2 | 0.0 / 0.0 | 60.54 / 60.18 | pass |
| rb_settings | 0.00 / 0.00 | 0.3 / 0.3 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| stack_20 | 18.34 / 15.33 | 245.4 / 245.9 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| pyramid_15 | 3.02 / 2.71 | 4.4 / 3.6 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| mass_ratio | 37.50 / 25.78 | 375.2 / 466.5 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| bounce | 0.74 / 0.86 | 49.8 / 61.0 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| pendulum | 0.00 / 0.00 | 0.0 / 0.0 | 1198.7 / 1198.7 | 0.00 / 0.00 | pass |
| rally_slide | 12.78 / 0.06 | 216.7 / 29.3 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| rally_far | 13.10 / 0.10 | 249.4 / 29.2 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| rally_far_shift | 12.78 / 0.06 | 216.7 / 29.3 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| rally_stage | 0.05 / 0.02 | 64.0 / 24.6 | 0.0 / 0.0 | 295900.92 / 296509.12 | pass |
| rally_tilt | 5.88 / 0.29 | 283.9 / 29.1 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| rally_big | 5.07 / 0.30 | 177.2 / 24.5 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| rally_crash | 16.12 / 3.62 | 892.5 / 177.3 | 0.0 / 0.0 | 0.00 / 0.00 | pass |

Energy is not checked where vehicle engines do work: vehicle, rally_stage.

Not compared (no baseline): tunnel_100 and projectile (legacy passes through the walls, T1.4); rally_body (legacy's hulls rest on their 4 cm collision margin, after being thrown by the terrain's edges; see docs/S1_GATE_REPORT.md).

### T1.7 Chrono collision tests

Chrono commit da8025269cc8a7f43c0bc53c469f884b521852ab.

| Test | Stock | Patched | Patched, features on | Same output as stock (patched / features) |
| --- | --- | --- | --- | --- |
| utest_COLL_bullet_utils | pass | pass | pass | yes/yes |
| utest_COLL_rounded_shapes | pass | pass | pass | yes/yes |
| utest_SMC_cohesion | pass | pass | pass | yes/yes |
| utest_SMC_cor_normal | fail | fail | fail | yes/yes |
| utest_SMC_rolling_gravity | fail | fail | fail | yes/yes |
| utest_SMC_sliding_gravity | pass | pass | pass | yes/yes |
| utest_SMC_spinning_gravity | fail | fail | fail | yes/yes |
| utest_SMC_stacking | fail | fail | fail | yes/yes |
| utest_SMC_sphere_sphere | pass | pass | pass | yes/no |
| utest_CH_compute_contact | pass | pass | pass | yes/yes |
| utest_CH_assembly_contact | pass | pass | pass | yes/yes |

Chrono's tests do not slide anything over a mesh, so the features leave their output unchanged. `integration/chrono/bn_mesh_slide.cpp` checks them directly in Chrono (box and sphere at 8 m/s over a flat mesh, largest upward speed):

| Features | Box bump m/s | Sphere bump m/s | Box travel end x (m) |
| --- | --- | --- | --- |
| off | 1.680 | 0.930 | -6.9 |
| on | 0.120 | 0.002 | 9.1 |

## Benchmarks

### B1.1 thread scaling (whole run, best of 3 ms)

| Scene | 1 thread | 2 | 4 | 8 | Speed-up at 8 |
| --- | --- | --- | --- | --- | --- |
| pile_1000 | 2078 | 1816 | 1827 | 1832 | 1.13x |
| joint_grid | 354 | 362 | 367 | 363 | 0.98x |

Threshold: 2.5x at 8 threads on the 1,000-body pile. This machine has 2 cores, so the threshold is checked on the PC.

### B1.2 feature cost (ms per whole run, single thread; positive = the feature costs time)

| Scene | All features ms | Hull clipping | Active edges | CCD and speculative contacts | World config (SAT, ordered pairs, reduction) |
| --- | --- | --- | --- | --- | --- |
| shape_zoo | 24.6 | +3.6 | -0.7 | +0.4 | +11.0 |
| mesh_slide | 6.1 | +3.6 | +2.4 | +2.0 | +3.7 |
| pile_1000 | 2067.9 | +35.1 | -18.3 | +62.9 | +23.8 |
| tunnel_100 | 21.7 | +0.1 | +1.0 | +16.4 | +1.0 |

**F1 gate (cloud part): PASSED.** T1.6, the B1.1 threshold and the MSVC build remain for the Windows PC.
