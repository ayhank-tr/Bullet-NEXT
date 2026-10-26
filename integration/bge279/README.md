# Bullet NEXT in the 2.79b game engine (F1, C1.11 and C1.12)

Two patches wire Bullet NEXT into the engine repo (github.com/ayhank-tr/yofrankie-bge279):
`c1.11-bullet-next-engine.patch`, then `c1.12-contact-points.patch`. They are made against that
repo's working copy of 26 Oct 2026 and are applied once, when F1 passes its gate.

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

## Checked here (Linux, GCC 13)

- Both physics copies compile and link together without clashes; every Bullet name they use
  resolves in the matching library.
- The changed converter, Ketsji, RNA, DNA, kernel and loader files compile; makesdna accepts
  the new scene fields.
- Blender's rigid body module and the C API callers compile against the legacy backend.

The full Windows build and the game tests run on the PC at the end of the project.
