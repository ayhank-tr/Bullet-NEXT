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

**Stage 1.** The patch also carries Stage 1's collision fixes: the points of a mesh contact in an
order of their own (not the triangles'), no edge-edge axis from nearly parallel edges, no point
from edge lines that meet far outside their edges, a flat polyhedron's internal box set to a
point at once, and an in-plane SAT axis against a triangle sent to the GJK path. T1.7 was re-run
with it: the same test results as before.

`bn_vehicle_rigs.cpp` (Stage 1, C1S.6) is not part of the patch: it runs Chrono::Vehicle's own
tyre force code (TMeasy with the BMW E90 front tyre's numbers, Pacejka 2002 with the sedan tyre
file) at set loads and slips, and a 4WD driveline built from Chrono's shaft elements with Rally
FX's locking settings on split grip. `tools/run_chrono_rigs.sh` builds it from ten Chrono::Vehicle
files and runs it; the Stage 1 gate puts the results beside Rally FX's own figures.
