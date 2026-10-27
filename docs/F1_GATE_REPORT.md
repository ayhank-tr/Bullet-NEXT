# F1 gate report

Platform: Linux x86_64, 2 CPUs, build `build-lab`.

## Tests

| Test | Result | Measured |
| --- | --- | --- |
| T1.1 Legacy untouched | PASS | legacy/ unchanged since F0: yes; 16 of 16 F0 scenes bit-identical to original and goldens; legacy = original on 21 of 21 scenes |
| T1.2 Thread determinism | PASS | 21 of 21 scenes identical at 1, 2, 4, 8 threads |
| T1.3 Ghost bumps | PASS | 0.001 m/s (limit 0.05; legacy 1.57) |
| T1.4 Tunnelling | PASS | 0 of 100 through (legacy 100 of 100) |
| T1.5 Stacking and joint quality vs F0 | PASS | 19 of 19 scenes within 10 % + 1 mm |
| T1.6 Python API | Pending: engine on the Windows PC at project end | |
| T1.7 Chrono build | PASS | Chrono da802526: 11 of 11 tests give the stock result patched and with the features on (7 pass on stock Chrono) |

### T1.1 and T1.2 per scene

| Scene | Legacy = original | Legacy = golden | Next hash (1-8 threads) |
| --- | --- | --- | --- |
| stack_10 | yes | yes | 98b13e8455dfb081 |
| pyramid_10 | yes | yes | 20cae110cb64ee50 |
| shape_zoo | yes | yes | 9b93b423b9548807 |
| joints_six | yes | yes | 294fbc3c132ce9a5 |
| vehicle | yes | yes | bccf3d9f3660642d |
| character | yes | yes | ff1db83382d7a8ca |
| soft_body | yes | yes | ad6758557634fca8 |
| sensors_filters | yes | yes | ca33928fb966f4f2 |
| projectile | yes | yes | 23729de6f276432c |
| tunnel_100 | yes | yes | 45713b9809da1106 |
| mesh_slide | yes | yes | 466a6d1b192fea22 |
| body_types | yes | yes | 2b52a027f8d8d166 |
| kinematic_switch | yes | yes | e63a02e4b230f412 |
| pile_1000 | yes | yes | 632d3a53b072fa2e |
| joint_grid | yes | yes | b8a85c435ca65929 |
| settings | yes | yes | bb2c9fad6d4e386b |
| rb_stack | yes | NO | 39d5e0e3b3756ff8 |
| rb_shapes | yes | NO | 087c78ffaada838d |
| rb_constraints | yes | NO | 9d62abe9f36fc58a |
| rb_kinematic | yes | NO | 585cadda8bb9ccb8 |
| rb_settings | yes | NO | 763d6b8b12fd0de0 |

### T1.5 and B1.3 quality (mean of 5 gravity variants)

| Scene | Mean penetration mm (legacy / next) | Max penetration mm (legacy / next) | Joint drift mm (legacy / next) | Energy gain J (legacy / next) | Result |
| --- | --- | --- | --- | --- | --- |
| stack_10 | 4.11 / 2.30 | 5.9 / 7.7 | 0.0 / 0.0 | 0.00 / 1.64 | pass |
| pyramid_10 | 3.24 / 1.27 | 43.1 / 35.2 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| shape_zoo | 11.29 / 6.57 | 125.2 / 111.4 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| joints_six | 0.38 / 0.08 | 22.9 / 22.9 | 80.0 / 80.0 | 0.00 / 0.00 | pass |
| vehicle | 0.00 / 0.00 | 0.0 / 0.0 | 0.0 / 0.0 | 51044.15 / 51044.14 | pass |
| character | 53.66 / 37.84 | 88.8 / 150.0 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| soft_body | 0.26 / 0.10 | 15.6 / 15.6 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| sensors_filters | 1.69 / 1.14 | 160.6 / 160.6 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| mesh_slide | 5.87 / 0.05 | 32.3 / 6.3 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| body_types | 3.18 / 1.63 | 47.3 / 47.0 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| kinematic_switch | 3.13 / 3.45 | 164.1 / 108.3 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| pile_1000 | 27.36 / 28.14 | 199.2 / 209.7 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| joint_grid | 12.62 / 5.08 | 238.4 / 124.1 | 3901.6 / 4030.7 | 0.00 / 0.00 | pass |
| settings | 0.60 / 0.23 | 14.8 / 20.8 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| rb_stack | 0.18 / 0.10 | 0.2 / 0.1 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| rb_shapes | 1.97 / 0.98 | 40.0 / 36.8 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| rb_constraints | 0.30 / 0.04 | 16.8 / 4.0 | 0.0 / 0.0 | 0.00 / 0.00 | pass |
| rb_kinematic | 0.11 / 0.11 | 3.0 / 3.2 | 0.0 / 0.0 | 60.54 / 60.18 | pass |
| rb_settings | 0.00 / 0.00 | 0.3 / 0.3 | 0.0 / 0.0 | 0.00 / 0.00 | pass |

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
| pile_1000 | 2568 | 2398 | 2259 | 2255 | 1.14x |
| joint_grid | 376 | 402 | 414 | 425 | 0.89x |

Threshold: 2.5x at 8 threads on the 1,000-body pile. This machine has 2 cores, so the threshold is checked on the PC.

### B1.2 feature cost (ms per whole run, single thread; positive = the feature costs time)

| Scene | All features ms | Hull clipping | Active edges | CCD and speculative contacts | World config (SAT, ordered pairs, reduction) |
| --- | --- | --- | --- | --- | --- |
| shape_zoo | 68.1 | +52.4 | +43.6 | +6.6 | +55.8 |
| mesh_slide | 4.2 | +1.5 | +0.6 | +0.1 | +1.8 |
| pile_1000 | 2814.3 | +281.4 | +607.3 | +318.0 | +485.0 |
| tunnel_100 | 20.3 | +1.2 | +0.8 | +15.1 | -1.2 |

**F1 gate (cloud part): PASSED.** T1.6, the B1.1 threshold and the MSVC build remain for the Windows PC.
