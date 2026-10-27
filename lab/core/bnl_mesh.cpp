// Bullet NEXT Lab - triangle meshes of scene shapes, for drawing (FL, CL.9).
#include "bnl_mesh.h"

#include <cmath>

namespace bnl {

namespace {

const float kPi = 3.14159265358979f;

Vec3 sub(const Vec3& a, const Vec3& b) { return Vec3(a.x - b.x, a.y - b.y, a.z - b.z); }
Vec3 cross(const Vec3& a, const Vec3& b) { return Vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x); }
float dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

void append(TriMesh& out, const TriMesh& m, const Xform* x) {
  const int base = (int)out.verts.size();
  for (size_t i = 0; i < m.verts.size(); ++i) out.verts.push_back(x ? transform(*x, m.verts[i]) : m.verts[i]);
  for (size_t i = 0; i < m.tris.size(); ++i) out.tris.push_back(base + m.tris[i]);
}

// A surface of revolution around Z: rings of (radius, z) from bottom to top, closed with caps.
void lathe(const std::vector<float>& r, const std::vector<float>& z, int seg, TriMesh& out) {
  const int rings = (int)r.size();
  for (int k = 0; k < rings; ++k)
    for (int i = 0; i < seg; ++i) {
      const float a = 2 * kPi * i / seg;
      out.verts.push_back(Vec3(r[k] * std::cos(a), r[k] * std::sin(a), z[k]));
    }
  for (int k = 0; k + 1 < rings; ++k)
    for (int i = 0; i < seg; ++i) {
      const int a = k * seg + i, b = k * seg + (i + 1) % seg, c = a + seg, d = b + seg;
      out.tris.push_back(a); out.tris.push_back(b); out.tris.push_back(d);
      out.tris.push_back(a); out.tris.push_back(d); out.tris.push_back(c);
    }
  const int bottom = (int)out.verts.size();
  out.verts.push_back(Vec3(0, 0, z.front()));
  const int top = (int)out.verts.size();
  out.verts.push_back(Vec3(0, 0, z.back()));
  for (int i = 0; i < seg; ++i) {
    const int n = (i + 1) % seg, last = (rings - 1) * seg;
    out.tris.push_back(bottom); out.tris.push_back(n); out.tris.push_back(i);
    out.tris.push_back(top); out.tris.push_back(last + i); out.tris.push_back(last + n);
  }
}

}  // namespace

Vec3 rotate(const Quat& q, const Vec3& v) {
  const Vec3 u(q.x, q.y, q.z);
  const Vec3 t = cross(u, v);
  const Vec3 t2(2 * t.x, 2 * t.y, 2 * t.z);
  const Vec3 c = cross(u, t2);
  return Vec3(v.x + q.w * t2.x + c.x, v.y + q.w * t2.y + c.y, v.z + q.w * t2.z + c.z);
}

Vec3 transform(const Xform& x, const Vec3& v) {
  const Vec3 r = rotate(x.q, v);
  return Vec3(r.x + x.p.x, r.y + x.p.y, r.z + x.p.z);
}

void convexHull(const std::vector<Vec3>& p, TriMesh& out) {
  out.verts = p;
  out.tris.clear();
  const int n = (int)p.size();
  if (n < 4) return;
  // Starting tetrahedron from the first four points that are not coplanar.
  int i0 = 0, i1 = -1, i2 = -1, i3 = -1;
  for (int i = 1; i < n && i1 < 0; ++i) if (dot(sub(p[i], p[i0]), sub(p[i], p[i0])) > 1e-12f) i1 = i;
  if (i1 < 0) return;
  for (int i = 1; i < n && i2 < 0; ++i) {
    const Vec3 c = cross(sub(p[i1], p[i0]), sub(p[i], p[i0]));
    if (dot(c, c) > 1e-12f) i2 = i;
  }
  if (i2 < 0) return;
  const Vec3 nrm = cross(sub(p[i1], p[i0]), sub(p[i2], p[i0]));
  for (int i = 1; i < n && i3 < 0; ++i) if (std::fabs(dot(nrm, sub(p[i], p[i0]))) > 1e-7f) i3 = i;
  if (i3 < 0) return;
  struct Face { int a, b, c; bool live; };
  std::vector<Face> faces;
  auto add = [&](int a, int b, int c, int inside) {
    const Vec3 fn = cross(sub(p[b], p[a]), sub(p[c], p[a]));
    if (dot(fn, sub(p[inside], p[a])) > 0) std::swap(b, c);  // normal points away from the inside point
    faces.push_back({a, b, c, true});
  };
  add(i0, i1, i2, i3); add(i0, i1, i3, i2); add(i0, i2, i3, i1); add(i1, i2, i3, i0);
  for (int i = 0; i < n; ++i) {
    if (i == i0 || i == i1 || i == i2 || i == i3) continue;
    std::vector<std::pair<int, int> > edges;
    bool outside = false;
    for (size_t f = 0; f < faces.size(); ++f) {
      Face& F = faces[f];
      if (!F.live) continue;
      const Vec3 fn = cross(sub(p[F.b], p[F.a]), sub(p[F.c], p[F.a]));
      if (dot(fn, sub(p[i], p[F.a])) > 1e-7f * std::sqrt(dot(fn, fn))) {
        F.live = false;
        outside = true;
        const int e[3][2] = {{F.a, F.b}, {F.b, F.c}, {F.c, F.a}};
        for (int k = 0; k < 3; ++k) {
          bool shared = false;
          for (size_t m = 0; m < edges.size(); ++m)
            if (edges[m].first == e[k][1] && edges[m].second == e[k][0]) { edges.erase(edges.begin() + m); shared = true; break; }
          if (!shared) edges.push_back(std::make_pair(e[k][0], e[k][1]));
        }
      }
    }
    if (!outside) continue;
    for (size_t m = 0; m < edges.size(); ++m) faces.push_back({edges[m].first, edges[m].second, i, true});
  }
  for (size_t f = 0; f < faces.size(); ++f)
    if (faces[f].live) { out.tris.push_back(faces[f].a); out.tris.push_back(faces[f].b); out.tris.push_back(faces[f].c); }
}

void shapeMesh(const ShapeDesc& s, TriMesh& out, int seg) {
  out.verts.clear();
  out.tris.clear();
  switch (s.type) {
    case ShapeDesc::kBox: {
      std::vector<Vec3> v; std::vector<int> t;
      boxMesh(s.half, v, t);
      out.verts = v; out.tris = t;
      // boxMesh winds for collision; make every face point outwards for drawing.
      for (size_t i = 0; i + 2 < out.tris.size(); i += 3) {
        const Vec3& a = out.verts[out.tris[i]]; const Vec3& b = out.verts[out.tris[i + 1]]; const Vec3& c = out.verts[out.tris[i + 2]];
        const Vec3 centre((a.x + b.x + c.x) / 3, (a.y + b.y + c.y) / 3, (a.z + b.z + c.z) / 3);
        if (dot(cross(sub(b, a), sub(c, a)), centre) < 0) std::swap(out.tris[i + 1], out.tris[i + 2]);
      }
      break;
    }
    case ShapeDesc::kSphere: {
      std::vector<float> r, z;
      const int rings = seg / 2;
      for (int k = 1; k < rings; ++k) {
        const float a = -kPi / 2 + kPi * k / rings;
        r.push_back(s.radius * std::cos(a)); z.push_back(s.radius * std::sin(a));
      }
      lathe(r, z, seg, out);
      out.verts[out.verts.size() - 2].z = -s.radius;
      out.verts[out.verts.size() - 1].z = s.radius;
      break;
    }
    case ShapeDesc::kCylinderZ:
      lathe({s.half.x, s.half.x}, {-s.half.z, s.half.z}, seg, out);
      break;
    case ShapeDesc::kConeZ:
      lathe({s.radius, 0.001f * s.radius}, {-s.height / 2, s.height / 2}, seg, out);
      break;
    case ShapeDesc::kCapsuleZ: {
      std::vector<float> r, z;
      const int q = seg / 4;
      for (int k = 1; k <= q; ++k) {
        const float a = -kPi / 2 + (kPi / 2) * k / q;
        r.push_back(s.radius * std::cos(a)); z.push_back(-s.height / 2 + s.radius * std::sin(a));
      }
      for (int k = 0; k < q; ++k) {
        const float a = (kPi / 2) * k / q;
        r.push_back(s.radius * std::cos(a)); z.push_back(s.height / 2 + s.radius * std::sin(a));
      }
      lathe(r, z, seg, out);
      out.verts[out.verts.size() - 2].z = -s.height / 2 - s.radius;
      out.verts[out.verts.size() - 1].z = s.height / 2 + s.radius;
      break;
    }
    case ShapeDesc::kHull:
      convexHull(s.points, out);
      break;
    case ShapeDesc::kMesh: {
      std::vector<Vec3> v; std::vector<int> t;
      meshData(s.mesh, v, t);
      for (size_t i = 0; i < v.size(); ++i) out.verts.push_back(Vec3(v[i].x * s.scale.x, v[i].y * s.scale.y, v[i].z * s.scale.z));
      out.tris = t;
      break;
    }
    case ShapeDesc::kCompound:
      for (size_t i = 0; i < s.children.size(); ++i) {
        TriMesh child;
        shapeMesh(s.children[i].shape, child, seg);
        append(out, child, &s.children[i].xform);
      }
      break;
  }
}

void bodyMesh(const BodyDesc& b, TriMesh& out, int seg) {
  if (b.kind == kSoft) {
    std::vector<Vec3> v; std::vector<int> t;
    meshData(b.softMesh, v, t);
    out.verts = v; out.tris = t;
    return;
  }
  shapeMesh(b.shape, out, seg);
}

}  // namespace bnl
