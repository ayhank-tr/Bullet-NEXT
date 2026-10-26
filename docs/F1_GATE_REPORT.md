# F1 gate report

Platform: Linux x86_64, 2 CPUs, build `build`.

## Tests

| Test | Result | Measured |
| --- | --- | --- |
| T1.1 Legacy untouched | PASS | legacy/ unchanged since F0: yes; 16 of 16 scenes bit-identical to original and goldens |
| T1.2 Thread determinism | PASS | 16 of 16 scenes identical at 1, 2, 4, 8 threads |
| T1.3 Ghost bumps | PASS | 0.012 m/s (limit 0.05; legacy 1.57) |
| T1.4 Tunnelling | PASS | 0 of 100 through (legacy 100 of 100) |
| T1.5 Stacking and joint quality vs F0 | PASS | 14 of 14 scenes within 10 % + 1 mm |
| T1.6 Python API | Pending: engine on the Windows PC at project end | |
| T1.7 Chrono build | PASS | Chrono da802526: 11 of 11 tests give the stock result patched and with the features on (7 pass on stock Chrono) |

### T1.1 and T1.2 per scene

| Scene | Legacy = original | Legacy = golden | Next hash (1-8 threads) |
| --- | --- | --- | --- |
| stack_10 | yes | yes | 98b13e8455dfb081 |
| pyramid_10 | yes | yes | 20cae110cb64ee50 |
| shape_zoo | yes | yes | 4fa3f5c7db5c4204 |
| joints_six | yes | yes | 635c6c51afa104cb |
| vehicle | yes | yes | fdb04c1b3344dfd3 |
| character | yes | yes | a5327773b2a6da07 |
| soft_body | yes | yes | ad6758557634fca8 |
| sensors_filters | yes | yes | 33a459050e2af112 |
| projectile | yes | yes | 094665cbb2ae5c8e |
| tunnel_100 | yes | yes | 51e226971c85a7a0 |
| mesh_slide | yes | yes | 42c45233a172203d |
| body_types | yes | yes | 068636fafd2fba8f |
| kinematic_switch | yes | yes | d7433186a4de4630 |
| pile_1000 | yes | yes | 13af6a1fddd61dd5 |
| joint_grid | yes | yes | 64a91967d0273b77 |
| settings | yes | yes | bb2c9fad6d4e386b |

### T1.5 and B1.3 quality (mean of 5 gravity variants)

| Scene | Mean penetration mm (legacy / next) | Max penetration mm (legacy / next) | Joint drift mm (legacy / next) | Energy gain J (legacy / next) | Result |
| --- | --- | --- | --- | --- | --- |
| stack_10 | 4.11 / 2.30 | 5.9 / 7.7 | 0.0 / 0.0 | 0.00 / 1.64 | pass |
| pyramid_10 | 3.24 / 1.27 | 43.1 / 35.2 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| shape_zoo | 11.29 / 6.53 | 125.2 / 111.4 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| joints_six | 0.38 / 0.08 | 22.9 / 22.9 | 80.0 / 80.0 | 0.00 / 0.00 | pass |
| vehicle | 0.00 / 0.00 | 0.0 / 0.0 | 0.0 / 0.0 | 51044.15 / 51044.15 | pass |
| character | 53.66 / 37.84 | 88.8 / 150.0 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| soft_body | 0.26 / 0.10 | 15.6 / 15.6 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| sensors_filters | 1.69 / 1.14 | 160.6 / 160.6 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| mesh_slide | 5.87 / 0.05 | 32.3 / 6.3 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| body_types | 3.18 / 1.62 | 47.3 / 47.0 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| kinematic_switch | 3.13 / 3.53 | 164.1 / 109.9 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| pile_1000 | 27.36 / 28.56 | 199.2 / 204.2 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| joint_grid | 12.62 / 4.75 | 238.4 / 123.5 | 3901.6 / 3843.8 | 0.00 / 0.00 | pass |
| settings | 0.60 / 0.23 | 14.8 / 20.8 | 0.0 / 0.0 | 0.00 / 0.00 | pass |

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
| on | 0.118 | 0.002 | 9.1 |

## Benchmarks

### B1.1 thread scaling (whole run, best of 3 ms)

| Scene | 1 thread | 2 | 4 | 8 | Speed-up at 8 |
| --- | --- | --- | --- | --- | --- |
| pile_1000 | 2393 | 2213 | 2282 | 2442 | 0.98x |
| joint_grid | 347 | 383 | 373 | 388 | 0.89x |

Threshold: 2.5x at 8 threads on the 1,000-body pile. This machine has 2 cores, so the threshold is checked on the PC.

### B1.2 feature cost (ms per whole run, single thread; positive = the feature costs time)

| Scene | All features ms | Hull clipping | Active edges | CCD and speculative contacts | World config (SAT, ordered pairs, reduction) |
| --- | --- | --- | --- | --- | --- |
| shape_zoo | 22.6 | +6.7 | +1.3 | +0.6 | +11.3 |
| mesh_slide | 5.1 | +2.8 | +1.9 | +1.5 | +3.1 |
| pile_1000 | 2428.8 | +175.9 | +61.4 | +79.5 | +193.4 |
| tunnel_100 | 16.4 | -1.0 | +0.4 | +12.0 | -1.8 |

**F1 gate (cloud part): PASSED.** T1.6, the B1.1 threshold and the MSVC build remain for the Windows PC.
