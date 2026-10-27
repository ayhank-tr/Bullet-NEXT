#!/usr/bin/env python3
"""Bullet NEXT - builds integration/chrono/chrono-cbt.patch (F1, T1.7).

Usage: make_chrono_patch.py <chrono_checkout>
The checkout must be clean (Chrono da80252 or close). The script ports Bullet NEXT's
collision-layer changes into Chrono's renamed Bullet copy (src/chrono/collision/bullet),
writes the patch with `git diff`, and restores the checkout.

Ported: the files below, renamed with Chrono's own rule (bt[A-Z0-9] -> cbt...), plus the
collision-only part of btNextFeatures as cbtNextCollision.h/.cpp and a switch in
ChCollisionSystemBullet. Not ported: Blender-specific changes and everything in the dynamics.
"""
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
UP = os.path.join(ROOT, "third_party", "bullet3_master")
NX = os.path.join(ROOT, "next")
PORTED = [
    "BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.h",
    "BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.cpp",
    "BulletCollision/CollisionDispatch/btManifoldResult.h",
    "BulletCollision/CollisionDispatch/btManifoldResult.cpp",
    "BulletCollision/CollisionShapes/btConvexPolyhedron.cpp",
]


def ren(text):
    return re.sub(r"bt([A-Z0-9])", r"cbt\1", text)


def sh(cmd, cwd, inp=None):
    p = subprocess.run(cmd, cwd=cwd, input=inp, capture_output=True, text=True)
    if p.returncode:
        sys.exit(f"{' '.join(cmd)} failed:\n{p.stdout}{p.stderr}")
    return p.stdout


def edit(path, old, new):
    s = open(path).read()
    if s.count(old) != 1:
        sys.exit(f"{path}: anchor not found once: {old[:60]!r}")
    open(path, "w").write(s.replace(old, new, 1))


def main():
    chrono = os.path.abspath(sys.argv[1])
    if sh(["git", "status", "--porcelain", "--untracked-files=no", "src"], chrono).strip():
        sys.exit("Chrono checkout has local changes in src/; start from a clean one")
    base = os.path.join(chrono, "src", "chrono", "collision", "bullet")

    # 1. the upstream -> next diff of each ported file, renamed, applied to Chrono's copy
    diff = ""
    for f in PORTED:
        p = subprocess.run(["diff", "-u", "--label", "a/" + f, "--label", "b/" + f,
                            os.path.join(UP, f), os.path.join(NX, f)], capture_output=True, text=True)
        diff += p.stdout
    sh(["patch", "-p1", "--fuzz=3", "--no-backup-if-mismatch"], base, ren(diff))

    # Chrono's internal-edge utility is older than upstream's (no heightfield branch), so the
    # compound-child change is made by hand: look the mesh up through the triangle's parent.
    ie = os.path.join(base, "BulletCollision", "CollisionDispatch", "cbtInternalEdgeUtility.cpp")
    s = open(ie).read()
    a = s.index("void cbtAdjustInternalEdgeContacts(")
    b = s.index("cbtTriangleInfoMap* triangleInfoMapPtr", a)
    head, fn = s[:a], s[a:b]
    old_lookup = "colObj0Wrap->getCollisionObject()->getCollisionShape()"
    if fn.count(old_lookup) < 2:
        sys.exit("cbtInternalEdgeUtility.cpp: unexpected layout")
    fn = fn.replace(old_lookup, "meshShape")
    anchor = "\t\treturn;\n\n"
    i = fn.index(anchor) + len(anchor)
    fn = (fn[:i] + "\t// Bullet NEXT (F1): the mesh is the triangle's parent shape, so meshes that are children of a\n"
          "\t// compound get their internal-edge data too (upstream only looked at the object's own shape).\n"
          "\tconst cbtCollisionShape* meshShape = colObj0Wrap->m_parent ? colObj0Wrap->m_parent->getCollisionShape() : "
          + old_lookup + ";\n\n" + fn[i:])
    open(ie, "w").write(head + fn + s[b:])

    # 1b. FL fixes found by the Lab, made by hand (the files differ too much from upstream to diff):
    # hulls without faces take the GJK path, and edge maps made by the features die with their shape.
    cc = os.path.join(base, "BulletCollision", "CollisionDispatch", "cbtConvexConvexAlgorithm.cpp")
    edit(cc, "\t\t\tif (polyhedronA->getConvexPolyhedron() && polyhedronB->getConvexPolyhedron())\n",
         "\t\t\t// Bullet NEXT (FL): a hull whose points are all on a plane, a line or one point has a\n"
         "\t\t\t// polyhedron with no faces; clipping against it crashed. Such hulls take the GJK path below.\n"
         "\t\t\tconst bool solidA = polyhedronA->getConvexPolyhedron() && polyhedronA->getConvexPolyhedron()->m_faces.size() >= 4;\n"
         "\t\t\tconst bool solidB = polyhedronB->getConvexPolyhedron() && polyhedronB->getConvexPolyhedron()->m_faces.size() >= 4;\n"
         "\t\t\tif (solidA && solidB)\n")
    edit(cc, "dispatchInfo.m_enableSatConvex && polyhedronA->getConvexPolyhedron() && polyhedronB->getShapeType() == TRIANGLE_SHAPE_PROXYTYPE",
         "dispatchInfo.m_enableSatConvex && solidA && polyhedronB->getShapeType() == TRIANGLE_SHAPE_PROXYTYPE")
    tm = os.path.join(base, "BulletCollision", "CollisionShapes", "cbtTriangleInfoMap.h")
    edit(tm, "\tcbtTriangleInfoMap()\n\t{\n",
         "\t// Bullet NEXT (FL): made by cbtNextEnableActiveEdges for its mesh, which then deletes it.\n"
         "\tbool m_bnOwnedByShape;\n\n\tcbtTriangleInfoMap()\n\t{\n\t\tm_bnOwnedByShape = false;\n")
    bv = os.path.join(base, "BulletCollision", "CollisionShapes", "cbtBvhTriangleMeshShape.cpp")
    edit(bv, "cbtBvhTriangleMeshShape::~cbtBvhTriangleMeshShape()\n{\n",
         "cbtBvhTriangleMeshShape::~cbtBvhTriangleMeshShape()\n{\n"
         "\tif (m_triangleInfoMap && m_triangleInfoMap->m_bnOwnedByShape)  // Bullet NEXT (cbtNextCollision)\n"
         "\t\tdelete m_triangleInfoMap;\n")

    # 2. cbtNextCollision.h/.cpp: the collision-only part of btNextFeatures
    src = open(os.path.join(NX, "BulletDynamics", "Dynamics", "btNextFeatures.cpp")).read()
    body = src[src.index("int btNextEnableHullClipping"):src.index("// --- CCD")]
    includes = "".join(l + "\n" for l in src.splitlines() if l.startswith('#include "BulletCollision/'))
    cpp = ("/*\nBullet NEXT collision features for Chrono's Bullet copy (F1, T1.7).\n"
           "Copyright (c) 2026 BGE NEXT contributors. Same zlib terms as Bullet.\n\n"
           "The collision-only part of Bullet NEXT's btNextFeatures: hull clipping, active edges with the\n"
           "flat-region rule and exact sphere-on-face contacts, and SAT for triangles plus the one-shot mesh\n"
           "contact reduction. Chrono's own dynamics and CCD are untouched.\n*/\n"
           "#define BT_NEXT_HEIGHTFIELD_EDGES 0  // Chrono's heightfield has no internal-edge data\n"
           '#include "btNextCollision.h"\n\n'
           '#include "BulletCollision/CollisionDispatch/btCollisionWorld.h"\n' + includes + "\n" + body +
           "void btNextConfigureCollision(btCollisionWorld* world)\n{\n"
           "\tworld->getDispatchInfo().m_enableSatConvex = true;\n\tgBnMeshContactReduction = true;\n}\n")
    hdr = ("/*\nBullet NEXT collision features for Chrono's Bullet copy (F1, T1.7).\n"
           "Copyright (c) 2026 BGE NEXT contributors. Same zlib terms as Bullet.\n*/\n"
           "#ifndef BT_NEXT_COLLISION_H\n#define BT_NEXT_COLLISION_H\n\n"
           "class btCollisionShape;\nclass btCollisionObject;\nclass btCollisionWorld;\n\n"
           "/// Polyhedral features for every convex polyhedron in the shape (compounds recursively), so hull\n"
           "/// pairs get clipped contact faces. Returns the number of shapes set up.\n"
           "int btNextEnableHullClipping(btCollisionShape* shape);\n\n"
           "/// Meshes in the object (also compound children): bake internal-edge data and\n"
           "/// correct contact normals on internal edges (no ghost bumps). False when there is none.\n"
           "bool btNextEnableActiveEdges(btCollisionObject* obj);\n\n"
           "/// SAT for convex-triangle pairs and one-shot mesh contact reduction.\n"
           "void btNextConfigureCollision(btCollisionWorld* world);\n\n"
           "/// Flat-region rule of the active-edge correction (on by default).\nextern bool gBnFlatRegionRule;\n\n#endif\n")
    open(os.path.join(base, "cbtNextCollision.cpp"), "w").write(ren(cpp))
    open(os.path.join(base, "cbtNextCollision.h"), "w").write(ren(hdr))

    # 3. the switch in ChCollisionSystemBullet and the build list
    h = os.path.join(base, "ChCollisionSystemBullet.h")
    c = os.path.join(base, "ChCollisionSystemBullet.cpp")
    edit(h, "    virtual void SetNumThreads(int nthreads) override;\n",
         "    virtual void SetNumThreads(int nthreads) override;\n\n"
         "    /// Bullet NEXT collision features (hull clipping, active mesh edges, SAT for triangles, one-shot\n"
         "    /// mesh contact reduction) for systems created from now on. Off by default; the environment\n"
         "    /// variable CH_BN_NEXT_FEATURES=1 turns it on for a whole program (used by the unit tests).\n"
         "    static void EnableNextFeatures(bool enable);\n")
    edit(c, '#include "chrono/collision/bullet/LinearMath/cbtIDebugDraw.h"\n',
         '#include "chrono/collision/bullet/LinearMath/cbtIDebugDraw.h"\n'
         '#include "chrono/collision/bullet/cbtNextCollision.h"\n\n#include <cstdlib>\n')
    edit(c, "ChCollisionSystemBullet::ChCollisionSystemBullet() : m_debug_drawer(nullptr) {",
         "// Bullet NEXT collision features (see EnableNextFeatures)\n"
         'static bool s_bn_next_features = std::getenv("CH_BN_NEXT_FEATURES") != nullptr;\n\n'
         "void ChCollisionSystemBullet::EnableNextFeatures(bool enable) {\n    s_bn_next_features = enable;\n}\n\n"
         "ChCollisionSystemBullet::ChCollisionSystemBullet() : m_debug_drawer(nullptr) {")
    edit(c, "    bt_collision_world = new cbtCollisionWorld(bt_dispatcher, bt_broadphase, bt_collision_configuration);\n",
         "    bt_collision_world = new cbtCollisionWorld(bt_dispatcher, bt_broadphase, bt_collision_configuration);\n"
         "    if (s_bn_next_features)\n        cbtNextConfigureCollision(bt_collision_world);\n")
    edit(c, "        model->SyncPosition();\n        bt_collision_world->addCollisionObject(",
         "        model->SyncPosition();\n        if (s_bn_next_features) {\n"
         "            cbtNextEnableHullClipping(bt_model->GetBulletObject()->getCollisionShape());\n"
         "            cbtNextEnableActiveEdges(bt_model->GetBulletObject());  // meshes and heightfields only\n"
         "        }\n        bt_collision_world->addCollisionObject(")
    cm = os.path.join(chrono, "src", "chrono", "CMakeLists.txt")
    edit(cm, "    collision/bullet/ChCollisionUtilsBullet.cpp\n",
         "    collision/bullet/ChCollisionUtilsBullet.cpp\n    collision/bullet/cbtNextCollision.cpp\n")
    edit(cm, "    collision/bullet/ChCollisionUtilsBullet.h\n",
         "    collision/bullet/ChCollisionUtilsBullet.h\n    collision/bullet/cbtNextCollision.h\n")

    # 4. write the patch, restore the checkout
    new = ["src/chrono/collision/bullet/cbtNextCollision.h", "src/chrono/collision/bullet/cbtNextCollision.cpp"]
    sh(["git", "add", "-N"] + new, chrono)
    patch = sh(["git", "diff", "--", "src"], chrono)
    sh(["git", "rm", "--cached", "-q"] + new, chrono)
    sh(["git", "checkout", "--", "src"], chrono)
    for f in new:
        os.remove(os.path.join(chrono, f))
    out = os.path.join(ROOT, "integration", "chrono", "chrono-cbt.patch")
    open(out, "w").write(patch)
    print(f"wrote {out} ({patch.count('diff --git')} files)")


if __name__ == "__main__":
    main()
