// Bullet NEXT Lab - triangle meshes of scene shapes, for drawing (FL, CL.9).
// Same axes and sizes as Bullet's shapes (Z up for the *_z shapes); no physics-engine types.
#ifndef BNL_MESH_H
#define BNL_MESH_H

#include "bnl_scene.h"

namespace bnl {

struct TriMesh {
  std::vector<Vec3> verts;
  std::vector<int> tris;   // three indices per triangle, counter-clockwise seen from outside
};

// The shape's surface in its own space (compound children placed by their transforms).
void shapeMesh(const ShapeDesc& s, TriMesh& out, int segments = 24);
// A body's surface in its own space; for a soft body, the triangles of its node mesh.
void bodyMesh(const BodyDesc& b, TriMesh& out, int segments = 24);
// Convex hull of a point set (for hull shapes).
void convexHull(const std::vector<Vec3>& points, TriMesh& out);

Vec3 rotate(const Quat& q, const Vec3& v);
Vec3 transform(const Xform& x, const Vec3& v);

}  // namespace bnl

#endif
