// Bullet NEXT Lab - Project Chrono as a Lab engine (FL, CL.6).
// Chrono's non-smooth contact system (NSC) with its Bullet collision copy carrying Bullet NEXT's
// collision changes (integration/chrono/chrono-cbt.patch, features switched on). It runs the scenes
// it can express - rigid bodies, the shapes, joints, keyframed bodies - and refuses the rest
// (vehicles, characters, soft bodies). It is built outside the Lab with Chrono
// (tools/build_chrono_engine.sh) and writes recordings the viewer plays back:
//   bnl_chrono [--scene NAME|all] [--scenes DIR] [--out DIR]
// Joints are Chrono's lock joints; limits, motors and springs are left out (a yardstick, not a port).
#include "bnl_engine.h"
#include "bnl_mesh.h"
#include "bnl_recording.h"

#include "chrono/collision/ChCollisionShapeBox.h"
#include "chrono/collision/ChCollisionShapeCapsule.h"
#include "chrono/collision/ChCollisionShapeCone.h"
#include "chrono/collision/ChCollisionShapeConvexHull.h"
#include "chrono/collision/ChCollisionShapeCylinder.h"
#include "chrono/collision/ChCollisionShapeSphere.h"
#include "chrono/collision/ChCollisionShapeTriangleMesh.h"
#include "chrono/collision/bullet/ChCollisionSystemBullet.h"
#include "chrono/geometry/ChTriangleMeshSoup.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChSystemNSC.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <memory>

using namespace chrono;

namespace {

ChVector3d C(const bnl::Vec3& v) { return ChVector3d(v.x, v.y, v.z); }
ChQuaterniond C(const bnl::Quat& q) { return ChQuaterniond(q.w, q.x, q.y, q.z); }
bnl::Vec3 B(const ChVector3d& v) { return bnl::Vec3((float)v.x(), (float)v.y(), (float)v.z()); }
bnl::Quat B(const ChQuaterniond& q) { return bnl::Quat((float)q.e1(), (float)q.e2(), (float)q.e3(), (float)q.e0()); }

// Inertia (diagonal, about the centre) for a mass, from the shape's own formula or its bounds.
ChVector3d inertiaOf(const bnl::ShapeDesc& s, double m) {
  switch (s.type) {
    case bnl::ShapeDesc::kBox: {
      const double x = 2 * s.half.x, y = 2 * s.half.y, z = 2 * s.half.z;
      return ChVector3d(m * (y * y + z * z) / 12, m * (x * x + z * z) / 12, m * (x * x + y * y) / 12);
    }
    case bnl::ShapeDesc::kSphere: { const double i = 0.4 * m * s.radius * s.radius; return ChVector3d(i, i, i); }
    case bnl::ShapeDesc::kCylinderZ: {
      const double r = s.half.x, h = 2 * s.half.z, a = m * (3 * r * r + h * h) / 12;
      return ChVector3d(a, a, 0.5 * m * r * r);
    }
    default: {
      bnl::TriMesh t;
      bnl::shapeMesh(s, t);
      bnl::Vec3 lo(1e9f, 1e9f, 1e9f), hi(-1e9f, -1e9f, -1e9f);
      for (const auto& p : t.verts) {
        lo.x = std::min(lo.x, p.x); lo.y = std::min(lo.y, p.y); lo.z = std::min(lo.z, p.z);
        hi.x = std::max(hi.x, p.x); hi.y = std::max(hi.y, p.y); hi.z = std::max(hi.z, p.z);
      }
      const double x = hi.x - lo.x, y = hi.y - lo.y, z = hi.z - lo.z;
      return ChVector3d(m * (y * y + z * z) / 12, m * (x * x + z * z) / 12, m * (x * x + y * y) / 12);
    }
  }
}

void addShape(ChBody& body, const bnl::ShapeDesc& s, const ChFramed& frame, std::shared_ptr<ChContactMaterialNSC> mat) {
  switch (s.type) {
    case bnl::ShapeDesc::kBox:
      body.AddCollisionShape(chrono_types::make_shared<ChCollisionShapeBox>(mat, 2 * s.half.x, 2 * s.half.y, 2 * s.half.z), frame);
      break;
    case bnl::ShapeDesc::kSphere: body.AddCollisionShape(chrono_types::make_shared<ChCollisionShapeSphere>(mat, s.radius), frame); break;
    case bnl::ShapeDesc::kCylinderZ:
      body.AddCollisionShape(chrono_types::make_shared<ChCollisionShapeCylinder>(mat, s.half.x, 2 * s.half.z), frame);
      break;
    case bnl::ShapeDesc::kConeZ: body.AddCollisionShape(chrono_types::make_shared<ChCollisionShapeCone>(mat, s.radius, s.height), frame); break;
    case bnl::ShapeDesc::kCapsuleZ:
      body.AddCollisionShape(chrono_types::make_shared<ChCollisionShapeCapsule>(mat, s.radius, s.height), frame);
      break;
    case bnl::ShapeDesc::kHull: {
      std::vector<ChVector3d> p;
      for (const auto& v : s.points) p.push_back(C(v));
      body.AddCollisionShape(chrono_types::make_shared<ChCollisionShapeConvexHull>(mat, p), frame);
      break;
    }
    case bnl::ShapeDesc::kMesh: {
      bnl::TriMesh t;
      bnl::shapeMesh(s, t);
      auto soup = chrono_types::make_shared<ChTriangleMeshSoup>();
      for (size_t i = 0; i + 2 < t.tris.size(); i += 3)
        soup->AddTriangle(C(t.verts[t.tris[i]]), C(t.verts[t.tris[i + 1]]), C(t.verts[t.tris[i + 2]]));
      const bool fixed = body.IsFixed();
      if (fixed) {
        body.AddCollisionShape(chrono_types::make_shared<ChCollisionShapeTriangleMesh>(mat, soup, true, false, 0.0), frame);
      } else {  // a moving mesh as its hull (Chrono's moving meshes need a sphere-swept radius)
        std::vector<ChVector3d> p;
        for (const auto& v : t.verts) p.push_back(C(v));
        body.AddCollisionShape(chrono_types::make_shared<ChCollisionShapeConvexHull>(mat, p), frame);
      }
      break;
    }
    case bnl::ShapeDesc::kCompound:
      for (const auto& c : s.children) addShape(body, c.shape, frame * ChFramed(C(c.xform.p), C(c.xform.q)), mat);
      break;
  }
}

class ChronoEngine : public bnl::Engine {
 public:
  const char* backend() const override { return "chrono"; }

  bool build(const bnl::SceneDesc& sc, const bnl::EngineOptions& o, std::string& error) override {
    m_scene = sc;
    for (const auto& b : sc.bodies) {
      if (b.kind == bnl::kCharacter || b.kind == bnl::kSoft) { error = "characters and soft bodies are not part of the Chrono engine"; return false; }
      if (b.shape.type == bnl::ShapeDesc::kHeightfield) { error = "heightfields are not part of the Chrono engine"; return false; }
    }
    if (!sc.vehicles.empty()) { error = "vehicles are not part of the Chrono engine"; return false; }
    ChCollisionSystemBullet::EnableNextFeatures(true);
    m_sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    m_sys.SetGravitationalAcceleration(ChVector3d(0, 0, sc.settings.gravityZ + o.gravityOffset));
    if (sc.settings.iterations > 0) m_sys.GetSolver()->AsIterative()->SetMaxIterations(sc.settings.iterations);
    m_fixed = chrono_types::make_shared<ChBody>();
    m_fixed->SetFixed(true);
    m_sys.Add(m_fixed);
    for (const auto& ob : sc.objects) {
      if (ob.type == bnl::ObjectDesc::kBody) addBody(ob.index);
      else if (ob.type == bnl::ObjectDesc::kConstraint) addConstraint(ob.index);
    }
    for (size_t i = 0; i < sc.bodies.size(); ++i) {
      const auto& d = sc.bodies[i];
      if (d.hasLinVel) m_bodies[i]->SetPosDt(C(d.linVel));
      if (d.hasAngVel) m_bodies[i]->SetAngVelParent(C(d.angVel));
    }
    return true;
  }

  void advance() override {
    const int f = m_frame;
    for (const auto& a : m_scene.actions) {
      if (f < a.from || (a.to >= 0 && f > a.to)) continue;
      auto& b = m_bodies[a.target];
      if (a.op == bnl::ActionDesc::kSuspend) b->SetFixed(true);
      else if (a.op == bnl::ActionDesc::kKinematicMove) { b->SetFixed(true); b->SetPos(b->GetPos() + C(a.v)); }
      else if (a.op == bnl::ActionDesc::kLinVel) b->SetPosDt(C(a.v));
    }
    const double dt = 1.0 / m_scene.settings.ticrate * (m_scene.profile == "blender53-rb" ? m_scene.settings.timeScale : 1.0);
    const int n = std::max(1, m_scene.settings.substeps);
    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < n; ++i) m_sys.DoStepDynamics(dt / n);
    m_micros = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
    ++m_frame;
    bnl::u64 h = 1469598103934665603ULL;
    auto feed = [&h](const void* p, size_t k) {
      const unsigned char* c = static_cast<const unsigned char*>(p);
      for (size_t i = 0; i < k; ++i) { h ^= c[i]; h *= 1099511628211ULL; }
    };
    for (int i = 0; i < bodyCount(); ++i) {
      const bnl::BodyState s = body(i);
      const float v[13] = {s.p.x, s.p.y, s.p.z, s.q.x, s.q.y, s.q.z, s.q.w, s.lin.x, s.lin.y, s.lin.z, s.ang.x, s.ang.y, s.ang.z};
      feed(v, sizeof(v));
    }
    const int events = 0;
    feed(&events, sizeof(events));
    m_frameHash = h;
    bnl::u64 fin = m_final;
    const unsigned char* c = reinterpret_cast<const unsigned char*>(&h);
    for (int i = 0; i < 8; ++i) { fin ^= c[i]; fin *= 1099511628211ULL; }
    m_final = fin;
  }

  int frame() const override { return m_frame; }
  double lastStepMicros() const override { return m_micros; }
  int bodyCount() const override { return (int)m_bodies.size(); }
  bnl::BodyState body(int i) const override {
    bnl::BodyState s;
    const auto& b = m_bodies[i];
    s.p = B(b->GetPos());
    s.q = B(b->GetRot());
    s.lin = B(b->GetPosDt());
    s.ang = B(b->GetAngVelParent());
    return s;
  }
  void softNodes(int, std::vector<bnl::Vec3>& out) const override { out.clear(); }
  int vehicleCount() const override { return 0; }
  int wheelCount(int) const override { return 0; }
  bnl::Xform wheel(int, int) const override { return bnl::Xform(); }
  void contacts(std::vector<bnl::Contact>& out) const override { out.clear(); }
  bnl::u64 frameHash() const override { return m_frameHash; }
  bnl::u64 finalHash() const override { return m_final; }
  void setLinearVelocity(int b, const bnl::Vec3& v) override { m_bodies[b]->SetPosDt(C(v)); }
  void setAngularVelocity(int b, const bnl::Vec3& v) override { m_bodies[b]->SetAngVelParent(C(v)); }
  // This engine records scenes; forces from game code are not needed here.
  void applyForce(int, const bnl::Vec3&) override {}
  void applyForceAt(int, const bnl::Vec3&, const bnl::Vec3&) override {}
  void applyImpulse(int b, const bnl::Vec3& j) override { m_bodies[b]->SetPosDt(m_bodies[b]->GetPosDt() + C(j) / m_bodies[b]->GetMass()); }
  void applyImpulseAt(int b, const bnl::Vec3& j, const bnl::Vec3&) override { applyImpulse(b, j); }
  void applyTorque(int, const bnl::Vec3&) override {}
  void applyTorqueImpulse(int, const bnl::Vec3&) override {}
  void setTransform(int b, const bnl::Xform& x) override { m_bodies[b]->SetPos(C(x.p)); m_bodies[b]->SetRot(C(x.q)); }
  void wake(int) override {}
  void setSleeping(int, bool) override {}
  bool raycast(const bnl::Vec3&, const bnl::Vec3&, bnl::RayHit&) const override { return false; }
  void raycastAll(const bnl::Vec3&, const bnl::Vec3&, std::vector<bnl::RayHit>& h) const override { h.clear(); }
  void vehicleControl(int, int, float, float, float) override {}
  void characterWalk(int, const bnl::Vec3&) override {}
  void characterJump(int) override {}

 private:
  void addBody(int i) {
    const bnl::BodyDesc& d = m_scene.bodies[i];
    auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
    mat->SetFriction(d.friction);
    mat->SetRestitution(d.restitution);
    auto b = chrono_types::make_shared<ChBody>();
    const bool moving = (d.kind == bnl::kRigid || d.kind == bnl::kDynamic) && !d.animated;
    b->SetFixed(!moving);
    b->SetPos(C(d.xform.p));
    b->SetRot(C(d.xform.q));
    if (moving) {
      b->SetMass(d.mass);
      b->SetInertiaXX(inertiaOf(d.shape, d.mass) * (d.inertia / 0.4));
    }
    addShape(*b, d.shape, ChFramed(), mat);
    b->EnableCollision(!d.ghost && d.kind != bnl::kSensor);
    m_sys.Add(b);
    m_bodies.push_back(b);
  }

  void addConstraint(int ci) {
    const bnl::ConstraintDesc& c = m_scene.constraints[ci];
    auto A = m_bodies[c.a];
    auto Bb = c.b >= 0 ? m_bodies[c.b] : m_fixed;
    // The joint frame in world space.
    ChFramed f;
    if (c.type >= bnl::ConstraintDesc::kRbFixed) f = ChFramed(C(c.frame.p), C(c.frame.q));
    else if (c.type == bnl::ConstraintDesc::kPointToPoint) f = ChFramed(A->TransformPointLocalToParent(C(c.pivotA)));
    else f = ChFramed(*A) * ChFramed(C(c.frameA.p), C(c.frameA.q));
    std::shared_ptr<ChLinkLock> link;
    switch (c.type) {
      case bnl::ConstraintDesc::kPointToPoint: case bnl::ConstraintDesc::kConeTwist: case bnl::ConstraintDesc::kRbPoint:
        link = chrono_types::make_shared<ChLinkLockSpherical>();
        break;
      case bnl::ConstraintDesc::kHinge: case bnl::ConstraintDesc::kRbHinge: case bnl::ConstraintDesc::kRbMotor:
        link = chrono_types::make_shared<ChLinkLockRevolute>();
        break;
      case bnl::ConstraintDesc::kRbSlider:
        link = chrono_types::make_shared<ChLinkLockPrismatic>();
        f = f * ChFramed(VNULL, QuatFromAngleY(CH_PI_2));  // Blender slides along X, Chrono along Z
        break;
      case bnl::ConstraintDesc::kRbPiston:
        link = chrono_types::make_shared<ChLinkLockCylindrical>();
        f = f * ChFramed(VNULL, QuatFromAngleY(CH_PI_2));
        break;
      default:
        link = chrono_types::make_shared<ChLinkLockLock>();
        break;
    }
    link->Initialize(A, Bb, f);
    m_sys.Add(link);
  }

  bnl::SceneDesc m_scene;
  ChSystemNSC m_sys;
  std::shared_ptr<ChBody> m_fixed;
  std::vector<std::shared_ptr<ChBody> > m_bodies;
  int m_frame = 0;
  double m_micros = 0;
  bnl::u64 m_frameHash = 0, m_final = 0;
};

}  // namespace

int main(int argc, char** argv) {
  std::string which = "all", dir = "lab/scenes", out = ".";
  for (int i = 1; i + 1 < argc; i += 2) {
    if (!std::strcmp(argv[i], "--scene")) which = argv[i + 1];
    else if (!std::strcmp(argv[i], "--scenes")) dir = argv[i + 1];
    else if (!std::strcmp(argv[i], "--out")) out = argv[i + 1];
  }
  int failures = 0;
  for (const std::string& name : bnl::listScenes(dir)) {
    if (which != "all" && which != name) continue;
    bnl::SceneDesc sc;
    std::string err;
    if (!bnl::loadScene(dir + "/" + name + ".json", sc, err)) { std::fprintf(stderr, "%s\n", err.c_str()); return 2; }
    ChronoEngine e;
    if (!e.build(sc, bnl::EngineOptions(), err)) { std::printf("%-18s chrono skipped: %s\n", name.c_str(), err.c_str()); continue; }
    bnl::Recording rec;
    rec.scene = sc.name;
    double ms = 0;
    bool bad = false;
    float vmax = 0;
    for (int f = 0; f < sc.frames; ++f) {
      e.advance();
      ms += e.lastStepMicros() / 1000.0;
      rec.capture(e);
      for (int b = 0; b < e.bodyCount(); ++b) {
        const bnl::BodyState s = e.body(b);
        const float sp = std::sqrt(s.lin.x * s.lin.x + s.lin.y * s.lin.y + s.lin.z * s.lin.z);
        vmax = std::max(vmax, sp);
        if (!std::isfinite(s.p.x + s.p.y + s.p.z + sp) || sp > 1000.0f) bad = true;
      }
    }
    rec.write(out + "/" + name + ".chrono.bnl");
    std::printf("%-18s chrono frames %4d hash %016llx %s ms %9.2f vmax %8.2f\n", name.c_str(), e.frame(),
                (unsigned long long)e.finalHash(), bad ? "BAD" : "ok ", ms, vmax);
    if (bad) ++failures;
  }
  return failures ? 1 : 0;
}
