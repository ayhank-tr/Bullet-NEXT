# Bullet NEXT in the 2.79b game engine (F1: C1.11, C1.12; F3c: C3.8; Stage 0: S0.1)

Four patches wire Bullet NEXT into the engine repo (github.com/ayhank-tr/yofrankie-bge279), in
this order: `c1.11-bullet-next-engine.patch`, `c1.12-contact-points.patch`,
`c3.8-soft-step-settings.patch`, then `s0.1-contact-erp.patch`. They are made against that repo's
working copy of 26 Oct 2026 (each on top of the ones before) and are applied when you give the
go; until then nothing is written into the engine repo.

## What it changes

- **Build.** `extern/bullet_next` (this repo, as a git submodule) replaces `extern/bullet2`.
  `WITH_BULLET_NEXT` (on by default) builds both backends; off gives the old build.
  The CI workflow fetches the submodule.
- **Two copies of the physics module.** `ge_phys_bullet` is built against Bullet NEXT.
  `ge_phys_bullet_legacy` is the same source, renamed at build time by `tools/rename_legacy.py`
  (`BN_legacy_symbols.txt` lists the module's own names), built against the frozen Bullet 2.84.
- **Per-scene choice.** Physics Engine gets a "Bullet NEXT" entry (`WOPHY_BULLET_NEXT`).
  "Bullet" stays the frozen 2.79b behaviour, so old files are unchanged.
- **Blender itself.** The rigid body world and Bullet's C API (cloth collision, Convex Hull)
  keep using the frozen Bullet 2.84, so old .blend simulations give the same results.
- **Bullet NEXT scenes (C1.11).** Threaded world (same result on any thread count), the F1
  collision features, and settings: solver (Sequential or NNCG), iterations, threads, joint ERP,
  contact ERP, CFM, and switches for CCD, smooth mesh contacts and hull clipping.
  The 2.79b Python setters that did nothing now work: `setNumIterations`, `setCcdMode`,
  `setSolverType`, `setLinearAirDamping`. New, with UPBGE's names: `setERPNonContact`,
  `setERPContact`, `setCFM`.
- **Contact list (C1.12).** Collision callbacks may take a 4th argument with every contact
  point of the pair, with UPBGE's attribute names. Old 1- and 3-argument callbacks are unchanged.
- **Soft Step solver (C3.8).** The scene's Solver gets a third entry, "Soft Step"
  (`GAME_SOLVER_SOFT_STEP`), with its settings in the same panel: substep rate (540 per second),
  contact stiffness (an eighth of the substep rate), contact damping ratio (10) and push-out speed (3 m/s); 0 keeps a
  default. Python: `bge.constraints.setSolverType(3)` and
  `bge.constraints.setSoftStepSettings(substepRate, contactStiffness, contactDamping, pushSpeed)`.
  Soft Step solves every island on the per-thread solvers, so results stay the same for any
  thread count. Old files keep their solver.
- **Impacts that add no speed (S0.1).** In Bullet NEXT scenes an impact's overlap is pushed out
  by split impulse at the Contact ERP (0.8), which adds no speed; resting contacts are corrected
  at the ERP, as in 2.79b. Upstream Bullet corrects shallow overlaps in the velocity at the
  Contact ERP, so with 0.8 a bounce came back faster than it hit (restitution 0.9 returned 1.05;
  2.79b's own rule gave 0.94). Now a bounce leaves at its restitution. The Contact ERP tooltip
  and the Python docs say so.

## Checked here (Linux, GCC 13)

- Both physics copies compile and link together without clashes; every Bullet name they use
  resolves in the matching library.
- The changed converter, Ketsji, RNA, DNA, kernel and loader files compile; makesdna accepts
  the new scene fields (C1.11, and again with C3.8's).
- C3.8: the physics module compiles with and without `BN_NEXT_BACKEND`, the Python binding and
  the scene RNA compile, the panel script parses, and makesdna builds the DNA with the new fields.
- Blender's rigid body module and the C API callers compile against the legacy backend.
- S0.1: the physics module compiles with and without `BN_NEXT_BACKEND`.

The full Windows build and the game tests run on the PC at the end of the project.
