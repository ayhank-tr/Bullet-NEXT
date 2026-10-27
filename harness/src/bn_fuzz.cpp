// Bullet NEXT Lab - random collision pairs (FL, CL.11, test TL.7).
// Each pair is a two-body Lab scene with random shapes, sizes, margins, rotations and overlap,
// built by the same code as the scene files (bn_load.cpp), so every shape the scenes can use is
// covered, in both host profiles. After collision detection every contact point must be finite
// with a unit normal, and after two frames every body must still have a finite position and
// velocity. Compiled once per backend, like the rest of the harness.
#include "bn_load.h"
#include "bnl_engine.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdint>

BN_NAMESPACE_BEGIN

namespace {

struct Rng {
  std::uint64_t s;
  explicit Rng(std::uint64_t seed) : s(seed * 0x9E3779B97F4A7C15ULL + 0x632BE59BD9B4E019ULL) { next(); next(); }
  std::uint32_t next() {  // xorshift64*
    s ^= s >> 12; s ^= s << 25; s ^= s >> 27;
    return (std::uint32_t)((s * 0x2545F4914F6CDD1DULL) >> 32);
  }
  float uni() { return (next() >> 8) * (1.0f / 16777216.0f); }
  float range(float a, float b) { return a + (b - a) * uni(); }
  float logRange(float a, float b) { return a * std::exp(std::log(b / a) * uni()); }
  int below(int n) { return (int)(next() % (std::uint32_t)n); }
  bool chance(float p) { return uni() < p; }
};

// Mostly ordinary game sizes, sometimes tiny or huge.
float sizeOf(Rng& r) {
  const float u = r.uni();
  if (u < 0.03f) return 0.001f;
  if (u < 0.06f) return 300.0f;
  return r.logRange(0.02f, 20.0f);
}

bnl::Quat rotationOf(Rng& r) {
  const int k = r.below(10);
  if (k == 0) return bnl::Quat(0, 0, 0, 1);
  if (k == 1) {  // a quarter turn about an axis: faces and edges exactly aligned
    const float h = 0.70710678f;
    const int a = r.below(3);
    return bnl::Quat(a == 0 ? h : 0, a == 1 ? h : 0, a == 2 ? h : 0, h);
  }
  const float u1 = r.uni(), u2 = r.range(0, 6.2831853f), u3 = r.range(0, 6.2831853f);  // uniform (Shoemake)
  const float a = std::sqrt(1 - u1), b = std::sqrt(u1);
  return bnl::Quat(a * std::sin(u2), a * std::cos(u2), b * std::sin(u3), b * std::cos(u3));
}

bnl::Vec3 directionOf(Rng& r) {
  for (;;) {
    const bnl::Vec3 v(r.range(-1, 1), r.range(-1, 1), r.range(-1, 1));
    const float l = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (l > 0.05f && l <= 1) return bnl::Vec3(v.x / l, v.y / l, v.z / l);
  }
}

bnl::MeshDesc meshOfShape(Rng& r, float size) {
  bnl::MeshDesc m;
  switch (r.below(3)) {
    case 0:
      m.gen = bnl::MeshDesc::kGrid;
      m.nx = 1 + r.below(8); m.ny = 1 + r.below(8); m.size = size;
      break;
    case 1:
      m.gen = bnl::MeshDesc::kBox;
      m.half = bnl::Vec3(size, size * r.range(0.2f, 1.5f), size * r.range(0.2f, 1.5f));
      break;
    default: {  // a loose soup of triangles, some of them degenerate
      m.gen = bnl::MeshDesc::kExplicit;
      const int nt = 1 + r.below(24);
      for (int t = 0; t < nt; ++t) {
        const int base = (int)m.verts.size();
        bnl::Vec3 a(r.range(-size, size), r.range(-size, size), r.range(-size * 0.3f, size * 0.3f));
        m.verts.push_back(a);
        const int kind = r.below(8);
        for (int k = 0; k < 2; ++k) {
          if (kind == 0) m.verts.push_back(a);  // all three corners the same
          else if (kind == 1) m.verts.push_back(bnl::Vec3(a.x + (k + 1) * 0.1f * size, a.y, a.z));  // a line
          else m.verts.push_back(bnl::Vec3(r.range(-size, size), r.range(-size, size), r.range(-size * 0.3f, size * 0.3f)));
        }
        m.tris.push_back(base); m.tris.push_back(base + 1); m.tris.push_back(base + 2);
      }
    }
  }
  return m;
}

// A random shape; extent is a rough radius for placing the other body.
bnl::ShapeDesc shapeOf(Rng& r, bool topLevel, bool dynamic, float& extent) {
  bnl::ShapeDesc s;
  const int n = topLevel ? 8 : 6;  // children: primitives and hulls only
  s.type = (bnl::ShapeDesc::Type)r.below(n);
  const float m = r.uni();
  s.margin = m < 0.1f ? 0.0f : (m < 0.2f ? 0.5f : 0.04f);
  const float a = sizeOf(r), b = r.chance(0.5f) ? a : sizeOf(r), c = r.chance(0.5f) ? a : sizeOf(r);
  switch (s.type) {
    case bnl::ShapeDesc::kBox:
    case bnl::ShapeDesc::kCylinderZ:
      s.half = bnl::Vec3(a, b, c);
      extent = std::sqrt(a * a + b * b + c * c);
      break;
    case bnl::ShapeDesc::kSphere:
      s.radius = a;
      extent = a;
      break;
    case bnl::ShapeDesc::kConeZ:
    case bnl::ShapeDesc::kCapsuleZ:
      s.radius = a; s.height = 2 * b;
      extent = a + b * 2;
      break;
    case bnl::ShapeDesc::kHull: {
      const int kind = r.below(6), np = kind == 0 ? 1 + r.below(3) : 4 + r.below(40);
      for (int i = 0; i < np; ++i) {
        bnl::Vec3 p(r.range(-a, a), r.range(-b, b), r.range(-c, c));
        if (kind == 1) p.z = 0;                    // flat
        if (kind == 2) { p.y = 0; p.z = 0; }       // a line
        if (kind == 3 && i > 0) p = s.points[r.below(i)];  // repeated points
        s.points.push_back(p);
      }
      extent = std::sqrt(a * a + b * b + c * c);
      break;
    }
    case bnl::ShapeDesc::kMesh:
      s.mesh = meshOfShape(r, a);
      s.dynamicMesh = dynamic;
      extent = 2 * a;
      break;
    case bnl::ShapeDesc::kCompound: {
      const int nc = 1 + r.below(3);
      extent = 0;
      for (int i = 0; i < nc; ++i) {
        bnl::ChildDesc ch;
        float e = 0;
        ch.shape = shapeOf(r, false, dynamic, e);
        const bnl::Vec3 d = directionOf(r);
        const float off = r.chance(0.3f) ? 0.0f : r.range(0, a);
        ch.xform.p = bnl::Vec3(d.x * off, d.y * off, d.z * off);
        ch.xform.q = rotationOf(r);
        s.children.push_back(ch);
        extent = std::max(extent, off + e);
      }
      break;
    }
  }
  return s;
}

bool finite3(const btVector3& v) { return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z()); }

void fail(bnl::FuzzResult& r, unsigned seed, int pair, const char* what) {
  ++r.failed;
  if (r.failures.size() < 8) {
    char buf[160];
    std::snprintf(buf, sizeof(buf), "seed %u pair %d: %s", seed, pair, what);
    r.failures.push_back(buf);
  }
}

}  // namespace

void fuzzPairs(unsigned seed, int pairs, bnl::FuzzResult& r) {
  for (int p = 0; p < pairs; ++p) {
    Rng rng((std::uint64_t)seed * 1000003ULL + (std::uint64_t)p);
    bnl::SceneDesc sc;
    sc.name = "fuzz";
    sc.frames = 2;
    sc.profile = rng.chance(0.25f) ? "blender53-rb" : "bge279";
    float ea = 0, eb = 0;
    bnl::BodyDesc a, b;
    a.kind = rng.chance(0.4f) ? bnl::kStatic : bnl::kRigid;
    a.shape = shapeOf(rng, true, a.kind != bnl::kStatic, ea);
    if (a.shape.type == bnl::ShapeDesc::kMesh && !a.shape.dynamicMesh) a.kind = bnl::kStatic;
    a.mass = a.kind == bnl::kStatic ? 0.0f : rng.logRange(0.01f, 1000.0f);
    a.xform.q = rotationOf(rng);
    b.kind = rng.chance(0.3f) ? bnl::kDynamic : bnl::kRigid;  // at least one moving body
    b.shape = shapeOf(rng, true, true, eb);
    b.mass = rng.logRange(0.01f, 1000.0f);
    const int place = rng.below(10);
    if (place == 0) {
      b.xform = a.xform;  // exactly on top of each other
    } else {
      const bnl::Vec3 d = directionOf(rng);
      const float dist = place == 1 ? 1e-6f : rng.uni() * (ea + eb) * 1.2f;
      b.xform.p = bnl::Vec3(d.x * dist, d.y * dist, d.z * dist);
      b.xform.q = rotationOf(rng);
    }
    if (rng.chance(0.2f)) {  // thrown at the other body
      const float v = rng.chance(0.3f) ? 200.0f : rng.range(1, 30);
      b.linVel = bnl::Vec3(-b.xform.p.x * v, -b.xform.p.y * v, -b.xform.p.z * v);
    }
    sc.addBody(a);
    sc.addBody(b);

    Settings st = settingsOf(sc);
    st.threads = 0;
    World w(st);
    build(w, sc);
    ++r.pairs;

    // Contacts straight from collision detection
    w.dyn()->performDiscreteCollisionDetection();
    btDispatcher* d = w.dyn()->getDispatcher();
    bool touching = false, bad = false;
    for (int i = 0; i < d->getNumManifolds() && !bad; ++i) {
      const btPersistentManifold* m = d->getManifoldByIndexInternal(i);
      for (int j = 0; j < m->getNumContacts(); ++j) {
        const btManifoldPoint& pt = m->getContactPoint(j);
        touching = true;
        ++r.contacts;
        if (!finite3(pt.m_positionWorldOnA) || !finite3(pt.m_positionWorldOnB) || !finite3(pt.m_normalWorldOnB) ||
            !std::isfinite(pt.getDistance())) {
          ++r.nonFinite;
          fail(r, seed, p, "contact with NaN or infinity");
          bad = true;
          break;
        }
        const float nl = pt.m_normalWorldOnB.length();
        if (nl < 0.99f || nl > 1.01f) ++r.badNormals;
        if (pt.getDistance() < r.deepest) r.deepest = pt.getDistance();
      }
    }
    if (touching) ++r.touching;

    // Two frames of the whole step: solver, integration, sleeping
    for (int f = 0; f < 2 && !bad; ++f) {
      w.step();
      for (size_t i = 0; i < w.bodies().size(); ++i) {
        const btRigidBody* rb = w.bodies()[i].rb;
        if (!rb) continue;
        if (!finite3(rb->getWorldTransform().getOrigin()) || !finite3(rb->getLinearVelocity()) ||
            !finite3(rb->getAngularVelocity())) {
          ++r.nonFinite;
          fail(r, seed, p, "body with NaN or infinity after a step");
          bad = true;
          break;
        }
      }
    }
  }
}

BN_NAMESPACE_END

#define BN_FUZZ_CAT2(a, b) a##b
#define BN_FUZZ_CAT(a, b) BN_FUZZ_CAT2(a, b)

void BN_FUZZ_CAT(bnl_fuzz_, BN_TAG)(unsigned seed, int pairs, bnl::FuzzResult& r) { bn::BN_TAG::fuzzPairs(seed, pairs, r); }
