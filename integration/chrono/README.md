# Bullet NEXT collision changes in Chrono's Bullet copy (F1, T1.7)

Project Chrono embeds a renamed Bullet (`cbt` prefix) in `src/chrono/collision/bullet` and uses
it only for collision detection. `chrono-cbt.patch` (made against Chrono `da80252`, 25 Oct 2026)
brings Bullet NEXT's collision-layer changes into that copy, renamed the same way:

- one-shot mesh contact reduction (the 4 best points over all touched triangles) and a mesh
  query margin that is never zero (C1.6b);
- hull clipping, with the 1,024-step loop on flat triangle hulls skipped (C1.5);
- active edges with the flat-region rule and exact sphere-on-face contacts (C1.6), in the new
  `cbtNextCollision.h/.cpp`; they also work for meshes inside a compound, which is how Chrono
  stores every shape (heightfields are left out: Chrono's copy has no edge data for them);
- `ChCollisionSystemBullet::EnableNextFeatures()` turns them on for new systems; the environment
  variable `CH_BN_NEXT_FEATURES=1` does the same for a whole program (used by the tests).

The features are off by default. Two small changes are always on: the hull-loop skip only saves
time, and the query margin only matters for meshes whose margin is below the contact breaking
threshold.
Blender-specific changes (inertia, AABB margins, linked bodies) are not part of this patch.

`tools/make_chrono_patch.py` makes the patch from `next/`. `tools/run_chrono_check.sh` builds
Chrono's collision and contact unit tests stock, patched, and patched with the features on, runs
`bn_mesh_slide.cpp` (ghost bumps on a flat mesh) with the features off and on, and writes the
results the F1 gate reads.
