// Bullet NEXT Lab - Jolt Physics as a Lab engine, the yardstick (FL, CL.6).
// Builds the same scene files as the Bullet engines with Jolt's own features, configured the way a
// Jolt user would: Jolt shapes, constraints, CharacterVirtual, soft bodies and its wheeled vehicle.
// Bullet-only settings (SOR, tau, contact breaking threshold) have no Jolt counterpart and are
// ignored; friction and restitution combine as in Bullet (product), so materials mean the same.
// Built as C++17 (Jolt's language level), apart from Bullet NEXT. Factory: bnl_create_engine_jolt().
#include "bnl_engine.h"
#include "bnl_mesh.h"

#include <Jolt/Jolt.h>

#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Character/CharacterVirtual.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/GroupFilter.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/Constraints/HingeConstraint.h>
#include <Jolt/Physics/Constraints/PointConstraint.h>
#include <Jolt/Physics/Constraints/SixDOFConstraint.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/SoftBody/SoftBodyCreationSettings.h>
#include <Jolt/Physics/SoftBody/SoftBodyMotionProperties.h>
#include <Jolt/Physics/SoftBody/SoftBodySharedSettings.h>
#include <Jolt/Physics/Vehicle/VehicleCollisionTester.h>
#include <Jolt/Physics/Vehicle/VehicleConstraint.h>
#include <Jolt/Physics/Vehicle/WheeledVehicleController.h>
#include <Jolt/RegisterTypes.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <set>

using namespace JPH;

namespace {

const float kPi = 3.14159265358979f;

namespace Layers {
const ObjectLayer kStatic = 0;
const ObjectLayer kMoving = 1;
}  // namespace Layers

class BroadPhaseLayers final : public BroadPhaseLayerInterface {
 public:
  uint GetNumBroadPhaseLayers() const override { return 2; }
  BroadPhaseLayer GetBroadPhaseLayer(ObjectLayer l) const override { return BroadPhaseLayer((BroadPhaseLayer::Type)l); }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
  const char* GetBroadPhaseLayerName(BroadPhaseLayer l) const override { return l.GetValue() ? "moving" : "static"; }
#endif
};

class ObjectVsBroadPhase final : public ObjectVsBroadPhaseLayerFilter {
 public:
  bool ShouldCollide(ObjectLayer a, BroadPhaseLayer b) const override { return a == Layers::kMoving || b.GetValue() == Layers::kMoving; }
};

class ObjectPairs final : public ObjectLayerPairFilter {
 public:
  bool ShouldCollide(ObjectLayer a, ObjectLayer b) const override { return a == Layers::kMoving || b == Layers::kMoving; }
};

// Blender's user collision groups (col_group / col_mask) and Bullet's rule that bodies joined by a
// constraint do not collide. Group ID = scene body index.
class UserGroups final : public GroupFilter {
 public:
  std::vector<std::pair<uint16_t, uint16_t> > masks;
  std::set<std::pair<uint32_t, uint32_t> > linked;
  bool CanCollide(const CollisionGroup& a, const CollisionGroup& b) const override {
    const uint32_t i = a.GetGroupID(), j = b.GetGroupID();
    if (i >= masks.size() || j >= masks.size()) return true;
    if (!((masks[i].first & masks[j].second) && (masks[j].first & masks[i].second))) return false;
    return linked.find(std::make_pair(std::min(i, j), std::max(i, j))) == linked.end();
  }
};

class ContactRecorder final : public ContactListener {
 public:
  std::mutex mutex;
  std::vector<bnl::Contact> contacts;
  const std::map<uint32_t, int>* bodyIndex = nullptr;

  void record(const Body& b1, const Body& b2, const ContactManifold& m) {
    std::lock_guard<std::mutex> lock(mutex);
    for (uint k = 0; k < m.mRelativeContactPointsOn1.size(); ++k) {
      const RVec3 p1 = m.GetWorldSpaceContactPointOn1(k), p2 = m.GetWorldSpaceContactPointOn2(k);
      bnl::Contact c;
      c.a = index(b1); c.b = index(b2);
      c.pa = bnl::Vec3((float)p1.GetX(), (float)p1.GetY(), (float)p1.GetZ());
      c.pb = bnl::Vec3((float)p2.GetX(), (float)p2.GetY(), (float)p2.GetZ());
      const Vec3 n = -m.mWorldSpaceNormal;  // Jolt: from 1 to 2; Bullet's normal on B points to A
      c.n = bnl::Vec3(n.GetX(), n.GetY(), n.GetZ());
      c.dist = -m.mPenetrationDepth;
      contacts.push_back(c);
    }
  }
  int index(const Body& b) const {
    auto it = bodyIndex->find(b.GetID().GetIndexAndSequenceNumber());
    return it == bodyIndex->end() ? -1 : it->second;
  }
  void OnContactAdded(const Body& b1, const Body& b2, const ContactManifold& m, ContactSettings&) override { record(b1, b2, m); }
  void OnContactPersisted(const Body& b1, const Body& b2, const ContactManifold& m, ContactSettings&) override { record(b1, b2, m); }
};

Vec3 J(const bnl::Vec3& v) { return Vec3(v.x, v.y, v.z); }
Quat J(const bnl::Quat& q) { return Quat(q.x, q.y, q.z, q.w).Normalized(); }
bnl::Vec3 B(Vec3Arg v) { return bnl::Vec3(v.GetX(), v.GetY(), v.GetZ()); }
bnl::Quat B(QuatArg q) { return bnl::Quat(q.GetX(), q.GetY(), q.GetZ(), q.GetW()); }

// Jolt's cylinder and capsule run along Y; the scene's run along Z.
RefConst<Shape> alongZ(const Shape* s) {
  return RotatedTranslatedShapeSettings(Vec3::sZero(), Quat::sRotation(Vec3::sAxisX(), 0.5f * kPi), s).Create().Get();
}

RefConst<Shape> hullOf(const std::vector<bnl::Vec3>& pts, float margin) {
  Array<Vec3> p;
  for (const auto& v : pts) p.push_back(J(v));
  ConvexHullShapeSettings s(p, std::max(0.0f, std::min(margin, 0.05f)));
  auto r = s.Create();
  if (r.HasError()) {  // degenerate point set: fall back to a small box around it
    return new BoxShape(Vec3(0.05f, 0.05f, 0.05f));
  }
  return r.Get();
}

RefConst<Shape> makeShape(const bnl::ShapeDesc& s, bool dynamicBody) {
  switch (s.type) {
    case bnl::ShapeDesc::kBox: {
      const float minHalf = std::min(s.half.x, std::min(s.half.y, s.half.z));
      return new BoxShape(J(s.half), std::min(s.margin, 0.9f * minHalf));
    }
    case bnl::ShapeDesc::kSphere: return new SphereShape(s.radius);
    case bnl::ShapeDesc::kCylinderZ: {
      const float minHalf = std::min(s.half.x, s.half.z);
      return alongZ(new CylinderShape(s.half.z, s.half.x, std::min(s.margin, 0.9f * minHalf)));
    }
    case bnl::ShapeDesc::kConeZ: {  // Jolt has no cone: the hull of its outline
      std::vector<bnl::Vec3> p;
      for (int i = 0; i < 32; ++i) {
        const float a = 2 * kPi * i / 32;
        p.push_back(bnl::Vec3(s.radius * std::cos(a), s.radius * std::sin(a), -0.5f * s.height));
      }
      p.push_back(bnl::Vec3(0, 0, 0.5f * s.height));
      return hullOf(p, s.margin);
    }
    case bnl::ShapeDesc::kCapsuleZ: return alongZ(new CapsuleShape(0.5f * s.height, s.radius));
    case bnl::ShapeDesc::kHull: return hullOf(s.points, s.margin);
    case bnl::ShapeDesc::kMesh: {
      bnl::TriMesh m;
      bnl::shapeMesh(s, m);
      if (dynamicBody) return hullOf(m.verts, s.margin);  // Jolt: moving triangle meshes are not supported
      VertexList v;
      for (const auto& p : m.verts) v.push_back(Float3(p.x, p.y, p.z));
      IndexedTriangleList t;
      for (size_t i = 0; i + 2 < m.tris.size(); i += 3) t.push_back(IndexedTriangle(m.tris[i], m.tris[i + 1], m.tris[i + 2]));
      return MeshShapeSettings(v, t).Create().Get();
    }
    case bnl::ShapeDesc::kCompound: {
      StaticCompoundShapeSettings c;
      for (const auto& ch : s.children) c.AddShape(J(ch.xform.p), J(ch.xform.q), makeShape(ch.shape, dynamicBody));
      return c.Create().Get();
    }
  }
  return new SphereShape(0.1f);
}

void hashFloat(bnl::u64& h, float f) {
  unsigned char c[4];
  std::memcpy(c, &f, 4);
  for (int i = 0; i < 4; ++i) { h ^= c[i]; h *= 1099511628211ULL; }
}

struct Slot {
  BodyID id;                         // rigid or soft body
  Ref<CharacterVirtual> character;   // character
  bnl::Vec3 walk;                    // character: displacement per step
  bool jump = false;
};

class JoltEngine final : public bnl::Engine {
 public:
  JoltEngine() {
    static bool registered = false;
    if (!registered) {
      RegisterDefaultAllocator();
      Factory::sInstance = new Factory();
      RegisterTypes();
      registered = true;
    }
  }

  ~JoltEngine() override {
    if (!m_physics) return;
    for (auto& v : m_vehicles) { m_physics->RemoveStepListener(v); m_physics->RemoveConstraint(v); }
    for (auto& c : m_constraints) m_physics->RemoveConstraint(c);
    BodyInterface& bi = m_physics->GetBodyInterface();
    for (auto& s : m_slots)
      if (!s.id.IsInvalid()) { bi.RemoveBody(s.id); bi.DestroyBody(s.id); }
  }

  const char* backend() const override { return "jolt"; }

  bool build(const bnl::SceneDesc& sc, const bnl::EngineOptions& o, std::string& error) override {
    if (m_physics) { error = "engine already built"; return false; }
    m_scene = sc;
    int threads = o.threads;
    if (threads < 0) { const char* e = std::getenv("BN_THREADS"); threads = e ? std::atoi(e) : 0; }
    m_temp.reset(new TempAllocatorImpl(64 * 1024 * 1024));
    if (threads > 1) m_jobs.reset(new JobSystemThreadPool(cMaxPhysicsJobs, cMaxPhysicsBarriers, threads - 1));
    else m_jobs.reset(new JobSystemSingleThreaded(cMaxPhysicsJobs));
    m_physics.reset(new PhysicsSystem());
    m_physics->Init(65536, 0, 65536, 65536, m_bpLayers, m_objVsBp, m_pairs);
    m_physics->SetGravity(Vec3(0, 0, sc.settings.gravityZ + o.gravityOffset));
    PhysicsSettings ps = m_physics->GetPhysicsSettings();
    ps.mTimeBeforeSleep = sc.settings.deactivationTime;
    if (sc.settings.iterations > 0) ps.mNumVelocitySteps = (uint)sc.settings.iterations;
    m_physics->SetPhysicsSettings(ps);
    m_physics->SetCombineFriction([](const Body& a, const SubShapeID&, const Body& b, const SubShapeID&) { return a.GetFriction() * b.GetFriction(); });
    m_physics->SetCombineRestitution([](const Body& a, const SubShapeID&, const Body& b, const SubShapeID&) { return a.GetRestitution() * b.GetRestitution(); });
    m_contacts.bodyIndex = &m_bodyIndex;
    m_physics->SetContactListener(&m_contacts);
    m_groups = new UserGroups();
    for (const auto& b : sc.bodies) m_groups->masks.push_back(std::make_pair(b.userGroup, b.userMask));

    // Parameters belong to the constraint before them; gather them first.
    std::vector<std::vector<bnl::ParamDesc> > params(sc.constraints.size());
    for (const auto& p : sc.params) params[p.constraint].push_back(p);

    BodyInterface& bi = m_physics->GetBodyInterface();
    m_slots.resize(sc.bodies.size());
    int vehicleWheels = 0;
    for (const auto& ob : sc.objects) {
      switch (ob.type) {
        case bnl::ObjectDesc::kBody: addBody(ob.index); break;
        case bnl::ObjectDesc::kConstraint: addConstraint(ob.index, params[ob.index]); break;
        case bnl::ObjectDesc::kVehicle: m_vehicleChassis.push_back(sc.vehicles[ob.index].chassis); break;
        default: break;
      }
    }
    for (size_t v = 0; v < m_vehicleChassis.size(); ++v) addVehicle((int)v);
    (void)vehicleWheels;
    m_physics->OptimizeBroadPhase();
    for (size_t i = 0; i < sc.bodies.size(); ++i) {
      const bnl::BodyDesc& d = sc.bodies[i];
      if (m_slots[i].id.IsInvalid()) continue;
      if (d.hasLinVel) bi.SetLinearVelocity(m_slots[i].id, J(d.linVel));
      if (d.hasAngVel) bi.SetAngularVelocity(m_slots[i].id, J(d.angVel));
    }
    return true;
  }

  void advance() override {
    actions();
    const float dt = 1.0f / m_scene.settings.ticrate * (m_scene.profile == "blender53-rb" ? m_scene.settings.timeScale : 1.0f);
    {
      std::lock_guard<std::mutex> lock(m_contacts.mutex);
      m_contacts.contacts.clear();
    }
    const auto t0 = std::chrono::steady_clock::now();
    m_physics->Update(dt, std::max(1, m_scene.settings.substeps), m_temp.get(), m_jobs.get());
    updateCharacters(dt);
    const auto t1 = std::chrono::steady_clock::now();
    m_micros = std::chrono::duration<double, std::micro>(t1 - t0).count();
    ++m_frame;
    m_frameHash = hashFrame();
    bnl::u64 fin = m_final;
    const unsigned char* c = reinterpret_cast<const unsigned char*>(&m_frameHash);
    for (int i = 0; i < 8; ++i) { fin ^= c[i]; fin *= 1099511628211ULL; }
    m_final = fin;
  }

  int frame() const override { return m_frame; }
  double lastStepMicros() const override { return m_micros; }
  int bodyCount() const override { return (int)m_slots.size(); }

  bnl::BodyState body(int i) const override {
    bnl::BodyState s;
    const Slot& sl = m_slots[i];
    if (sl.character) {
      s.p = B(sl.character->GetPosition());
      s.q = B(sl.character->GetRotation());
      s.lin = B(sl.character->GetLinearVelocity());
      return s;
    }
    const BodyInterface& bi = m_physics->GetBodyInterfaceNoLock();
    if (m_scene.bodies[i].kind == bnl::kSoft) {
      std::vector<bnl::Vec3> n;
      softNodes(i, n);
      for (const auto& p : n) { s.p.x += p.x; s.p.y += p.y; s.p.z += p.z; }
      if (!n.empty()) { s.p.x /= n.size(); s.p.y /= n.size(); s.p.z /= n.size(); }
      s.lin = B(bi.GetLinearVelocity(sl.id));
      return s;
    }
    s.p = B(bi.GetPosition(sl.id));
    s.q = B(bi.GetRotation(sl.id));
    s.lin = B(bi.GetLinearVelocity(sl.id));
    s.ang = B(bi.GetAngularVelocity(sl.id));
    return s;
  }

  void softNodes(int i, std::vector<bnl::Vec3>& out) const override {
    out.clear();
    if (m_scene.bodies[i].kind != bnl::kSoft) return;
    BodyLockRead lock(m_physics->GetBodyLockInterfaceNoLock(), m_slots[i].id);
    if (!lock.Succeeded()) return;
    const Body& b = lock.GetBody();
    const RMat44 com = b.GetCenterOfMassTransform();
    const auto* mp = static_cast<const SoftBodyMotionProperties*>(b.GetMotionPropertiesUnchecked());
    for (const SoftBodyVertex& v : mp->GetVertices()) out.push_back(B(com * v.mPosition));
  }

  int vehicleCount() const override { return (int)m_vehicles.size(); }
  int wheelCount(int v) const override { return (int)m_vehicles[v]->GetWheels().size(); }
  bnl::Xform wheel(int v, int w) const override {
    const RMat44 m = m_vehicles[v]->GetWheelWorldTransform(w, Vec3::sAxisX(), Vec3::sAxisZ());
    bnl::Xform x;
    x.p = B(m.GetTranslation());
    x.q = B(m.GetQuaternion());
    return x;
  }

  void contacts(std::vector<bnl::Contact>& out) const override {
    std::lock_guard<std::mutex> lock(const_cast<std::mutex&>(m_contacts.mutex));
    out = m_contacts.contacts;
  }

  bnl::u64 frameHash() const override { return m_frameHash; }
  bnl::u64 finalHash() const override { return m_final; }

  void setLinearVelocity(int b, const bnl::Vec3& v) override { if (rigid(b)) bi().SetLinearVelocity(id(b), J(v)); }
  void setAngularVelocity(int b, const bnl::Vec3& v) override { if (rigid(b)) bi().SetAngularVelocity(id(b), J(v)); }
  void applyForce(int b, const bnl::Vec3& f) override { if (rigid(b)) bi().AddForce(id(b), J(f)); }
  void applyForceAt(int b, const bnl::Vec3& f, const bnl::Vec3& p) override { if (rigid(b)) bi().AddForce(id(b), J(f), RVec3(J(p))); }
  void applyImpulse(int b, const bnl::Vec3& j) override { if (rigid(b)) bi().AddImpulse(id(b), J(j)); }
  void applyImpulseAt(int b, const bnl::Vec3& j, const bnl::Vec3& p) override { if (rigid(b)) bi().AddImpulse(id(b), J(j), RVec3(J(p))); }
  void applyTorque(int b, const bnl::Vec3& t) override { if (rigid(b)) bi().AddTorque(id(b), J(t)); }
  void applyTorqueImpulse(int b, const bnl::Vec3& t) override { if (rigid(b)) bi().AddAngularImpulse(id(b), J(t)); }
  void setTransform(int b, const bnl::Xform& x) override {
    if (m_slots[b].character) { m_slots[b].character->SetPosition(RVec3(J(x.p))); return; }
    if (!rigid(b)) return;
    if (bi().GetMotionType(id(b)) == EMotionType::Kinematic)
      bi().MoveKinematic(id(b), RVec3(J(x.p)), J(x.q), 1.0f / m_scene.settings.ticrate);
    else
      bi().SetPositionAndRotation(id(b), RVec3(J(x.p)), J(x.q), EActivation::Activate);
  }
  void wake(int b) override { if (!id(b).IsInvalid()) bi().ActivateBody(id(b)); }
  void setSleeping(int, bool) override {}  // Jolt: per-body sleeping is chosen at creation

  bool raycast(const bnl::Vec3& a, const bnl::Vec3& b, bnl::RayHit& hit) const override {
    RRayCast ray(RVec3(J(a)), J(b) - J(a));
    RayCastResult r;
    if (!m_physics->GetNarrowPhaseQuery().CastRay(ray, r)) return false;
    fillHit(ray, r, hit);
    return true;
  }
  void raycastAll(const bnl::Vec3& a, const bnl::Vec3& b, std::vector<bnl::RayHit>& hits) const override {
    hits.clear();
    RRayCast ray(RVec3(J(a)), J(b) - J(a));
    AllHitCollisionCollector<CastRayCollector> c;
    m_physics->GetNarrowPhaseQuery().CastRay(ray, RayCastSettings(), c);
    c.Sort();
    for (const auto& r : c.mHits) { bnl::RayHit h; fillHit(ray, r, h); hits.push_back(h); }
  }

  void vehicleControl(int v, int, float engine, float brake, float steer) override {
    // Jolt's controller takes driver input for the whole car: the strongest request wins.
    auto* c = static_cast<WheeledVehicleController*>(m_vehicles[v]->GetController());
    c->SetDriverInput(std::max(-1.0f, std::min(1.0f, engine / 1500.0f)), -steer / 0.3f, std::min(1.0f, brake / 40.0f), 0.0f);
    bi().ActivateBody(m_vehicles[v]->GetVehicleBody()->GetID());
  }
  void characterWalk(int b, const bnl::Vec3& step) override { m_slots[b].walk = step; }
  void characterJump(int b) override { m_slots[b].jump = true; }

 private:
  BodyInterface& bi() { return m_physics->GetBodyInterfaceNoLock(); }
  BodyID id(int b) const { return m_slots[b].id; }
  bool rigid(int b) const { return !m_slots[b].id.IsInvalid() && m_scene.bodies[b].kind != bnl::kSoft; }

  void fillHit(const RRayCast& ray, const RayCastResult& r, bnl::RayHit& h) const {
    const RVec3 p = ray.GetPointOnRay(r.mFraction);
    h.p = B(p);
    h.t = r.mFraction;
    auto it = m_bodyIndex.find(r.mBodyID.GetIndexAndSequenceNumber());
    h.body = it == m_bodyIndex.end() ? -1 : it->second;
    BodyLockRead lock(m_physics->GetBodyLockInterfaceNoLock(), r.mBodyID);
    if (lock.Succeeded()) h.n = B(lock.GetBody().GetWorldSpaceSurfaceNormal(r.mSubShapeID2, p));
  }

  void addBody(int i) {
    const bnl::BodyDesc& d = m_scene.bodies[i];
    BodyInterface& b = bi();
    if (d.kind == bnl::kCharacter) {
      Ref<CharacterVirtualSettings> s = new CharacterVirtualSettings();
      s->mUp = Vec3::sAxisZ();
      s->mShape = makeShape(d.shape, true);
      s->mSupportingVolume = Plane(Vec3::sAxisZ(), -d.shape.radius);
      m_slots[i].character = new CharacterVirtual(s, RVec3(J(d.xform.p)), J(d.xform.q), 0, m_physics.get());
      return;
    }
    if (d.kind == bnl::kSoft) {
      bnl::TriMesh m;
      bnl::bodyMesh(d, m);
      Ref<SoftBodySharedSettings> s = new SoftBodySharedSettings();
      const float invMass = m.verts.empty() ? 1.0f : float(m.verts.size()) / std::max(1e-3f, d.mass);
      for (const auto& p : m.verts) s->mVertices.push_back(SoftBodySharedSettings::Vertex(Float3(p.x, p.y, p.z), Float3(0, 0, 0), invMass));
      for (size_t t = 0; t + 2 < m.tris.size(); t += 3) s->AddFace(SoftBodySharedSettings::Face(m.tris[t], m.tris[t + 1], m.tris[t + 2]));
      SoftBodySharedSettings::VertexAttributes attr(1.0e-4f, 1.0e-4f, d.bending ? 1.0e-3f : FLT_MAX);
      s->CreateConstraints(&attr, 1, d.bending ? SoftBodySharedSettings::EBendType::Distance : SoftBodySharedSettings::EBendType::None);
      s->Optimize();
      SoftBodyCreationSettings cs(s, RVec3(J(d.xform.p)), J(d.xform.q), Layers::kMoving);
      cs.mFriction = d.friction;
      if (d.shapeMatching) cs.mPressure = 2000.0f;  // a closed soft shape keeps its volume (Bullet: shape matching)
      cs.mCollisionGroup = CollisionGroup(m_groups, (CollisionGroup::GroupID)i, 0);
      m_slots[i].id = b.CreateAndAddSoftBody(cs, EActivation::Activate);
      m_bodyIndex[m_slots[i].id.GetIndexAndSequenceNumber()] = i;
      return;
    }
    const bool moving = (d.kind == bnl::kRigid || d.kind == bnl::kDynamic) && !d.animated;
    BodyCreationSettings cs(makeShape(d.shape, moving), RVec3(J(d.xform.p)), J(d.xform.q),
                            moving ? EMotionType::Dynamic : EMotionType::Static, moving ? Layers::kMoving : Layers::kStatic);
    cs.mFriction = d.friction;
    cs.mRestitution = d.restitution;
    cs.mLinearDamping = d.linDamp;
    cs.mAngularDamping = d.angDamp;
    cs.mCollisionGroup = CollisionGroup(m_groups, (CollisionGroup::GroupID)i, 0);
    if (d.kind == bnl::kSensor) { cs.mIsSensor = true; cs.mMotionType = EMotionType::Kinematic; cs.mObjectLayer = Layers::kMoving; }
    if (d.animated) { cs.mMotionType = EMotionType::Kinematic; cs.mObjectLayer = Layers::kMoving; }  // Blender: keyframed passive
    if (moving) {
      cs.mOverrideMassProperties = EOverrideMassProperties::CalculateInertia;
      cs.mMassPropertiesOverride.mMass = d.mass;
      cs.mInertiaMultiplier = d.inertia / 0.4f;
      cs.mMotionQuality = EMotionQuality::LinearCast;
      cs.mIsSensor = d.ghost;
      if (d.clampVelMax > 0) cs.mMaxLinearVelocity = d.clampVelMax;
      if (d.clampAngMax > 0) cs.mMaxAngularVelocity = d.clampAngMax;
      if (d.noSleep) cs.mAllowSleeping = false;
      int dofs = 0;
      const float lf[3] = {d.linFactor.x, d.linFactor.y, d.linFactor.z}, af[3] = {d.angFactor.x, d.angFactor.y, d.angFactor.z};
      for (int k = 0; k < 3; ++k) {
        if (lf[k] != 0) dofs |= 1 << k;
        if (d.kind == bnl::kRigid && af[k] != 0) dofs |= 8 << k;
      }
      if (!(dofs & 7)) dofs |= 7;
      cs.mAllowedDOFs = EAllowedDOFs(dofs);
    } else {
      cs.mAllowDynamicOrKinematic = false;
    }
    m_slots[i].id = b.CreateAndAddBody(cs, moving || d.animated ? EActivation::Activate : EActivation::DontActivate);
    m_bodyIndex[m_slots[i].id.GetIndexAndSequenceNumber()] = i;
  }

  Body* lockBody(int i) {
    if (i < 0) return &Body::sFixedToWorld;
    return m_physics->GetBodyLockInterfaceNoLock().TryGetBody(m_slots[i].id);
  }

  RMat44 worldOf(int i) const {
    const BodyInterface& b = m_physics->GetBodyInterfaceNoLock();
    return RMat44::sRotationTranslation(b.GetRotation(m_slots[i].id), b.GetPosition(m_slots[i].id));
  }

  void addConstraint(int ci, const std::vector<bnl::ParamDesc>& params) {
    const bnl::ConstraintDesc& c = m_scene.constraints[ci];
    if (m_slots[c.a].id.IsInvalid() || (c.b >= 0 && m_slots[c.b].id.IsInvalid())) return;
    Body* A = lockBody(c.a);
    Body* Bb = lockBody(c.b);
    if (c.b >= 0) m_groups->linked.insert(std::make_pair((uint32_t)std::min(c.a, c.b), (uint32_t)std::max(c.a, c.b)));
    const RMat44 wa = worldOf(c.a);
    auto frame = [&](const bnl::Xform& f) { return wa * RMat44::sRotationTranslation(J(f.q), J(f.p)); };
    Ref<TwoBodyConstraint> made;
    switch (c.type) {
      case bnl::ConstraintDesc::kPointToPoint: {
        PointConstraintSettings s;
        s.mSpace = EConstraintSpace::WorldSpace;
        s.mPoint1 = s.mPoint2 = wa * J(c.pivotA);
        made = static_cast<TwoBodyConstraint*>(s.Create(*A, *Bb));
        break;
      }
      case bnl::ConstraintDesc::kHinge: {
        const RMat44 f = frame(c.frameA);
        HingeConstraintSettings s;
        s.mSpace = EConstraintSpace::WorldSpace;
        s.mPoint1 = s.mPoint2 = f.GetTranslation();
        s.mHingeAxis1 = s.mHingeAxis2 = f.GetAxisZ();
        s.mNormalAxis1 = s.mNormalAxis2 = f.GetAxisX();
        for (const auto& p : params)
          if (p.index == 3 && p.v0 <= p.v1) { s.mLimitsMin = p.v0; s.mLimitsMax = p.v1; }
        made = static_cast<TwoBodyConstraint*>(s.Create(*A, *Bb));
        break;
      }
      case bnl::ConstraintDesc::kConeTwist: {
        const RMat44 f = frame(c.frameA);
        SwingTwistConstraintSettings s;
        s.mSpace = EConstraintSpace::WorldSpace;
        s.mPosition1 = s.mPosition2 = f.GetTranslation();
        s.mTwistAxis1 = s.mTwistAxis2 = f.GetAxisX();
        s.mPlaneAxis1 = s.mPlaneAxis2 = f.GetAxisY();
        float span1 = kPi, span2 = kPi, twist = kPi;
        for (const auto& p : params) {
          const float v = p.v1 < 0 ? kPi : p.v1;
          if (p.index == 3) span1 = v; else if (p.index == 4) span2 = v; else if (p.index == 5) twist = v;
        }
        s.mNormalHalfConeAngle = span1;
        s.mPlaneHalfConeAngle = span2;
        s.mTwistMinAngle = -twist;
        s.mTwistMaxAngle = twist;
        made = static_cast<TwoBodyConstraint*>(s.Create(*A, *Bb));
        break;
      }
      case bnl::ConstraintDesc::kGeneric6DofSpring: {
        const RMat44 f = frame(c.frameA);
        SixDOFConstraintSettings s;
        s.mSpace = EConstraintSpace::WorldSpace;
        s.mPosition1 = s.mPosition2 = f.GetTranslation();
        s.mAxisX1 = s.mAxisX2 = f.GetAxisX();
        s.mAxisY1 = s.mAxisY2 = f.GetAxisY();
        for (int k = 0; k < 6; ++k) s.MakeFixedAxis((SixDOFConstraintSettings::EAxis)k);  // Bullet's 6DOF starts locked
        int velocityMotor[6] = {0, 0, 0, 0, 0, 0}, springMotor[6] = {0, 0, 0, 0, 0, 0};
        float target[6] = {0, 0, 0, 0, 0, 0};
        for (const auto& p : params) {
          if (p.index <= 5) {
            const auto ax = (SixDOFConstraintSettings::EAxis)p.index;
            if (p.v0 > p.v1) s.MakeFreeAxis(ax);
            else if (p.v0 == p.v1) s.MakeFixedAxis(ax);
            else s.SetLimitedAxis(ax, p.v0, p.v1);
          } else if (p.index <= 11) {
            const int ax = p.index - 6;
            if (p.v1 > 0) {
              velocityMotor[ax] = 1;
              target[ax] = p.v0;
              MotorSettings& m = s.mMotorSettings[ax];
              if (ax < 3) m.SetForceLimit(p.v1); else m.SetTorqueLimit(p.v1);
            }
          } else if (p.index <= 17) {
            const int ax = p.index - 12;
            if (p.v0 != 0) {
              springMotor[ax] = 1;
              s.mMotorSettings[ax].mSpringSettings = SpringSettings(ESpringMode::StiffnessAndDamping, p.v0, p.v1);
            }
          }
        }
        auto* six = static_cast<SixDOFConstraint*>(s.Create(*A, *Bb));
        Vec3 lin(target[0], target[1], target[2]), ang(target[3], target[4], target[5]);
        for (int ax = 0; ax < 6; ++ax) {
          if (velocityMotor[ax]) six->SetMotorState((SixDOFConstraintSettings::EAxis)ax, EMotorState::Velocity);
          else if (springMotor[ax]) six->SetMotorState((SixDOFConstraintSettings::EAxis)ax, EMotorState::Position);
        }
        six->SetTargetVelocityCS(lin);
        six->SetTargetAngularVelocityCS(ang);
        made = six;
        break;
      }
    }
    if (made) {
      m_physics->AddConstraint(made);
      m_constraints.push_back(made);
    }
  }

  void addVehicle(int v) {
    const int chassis = m_vehicleChassis[v];
    VehicleConstraintSettings vs;
    vs.mUp = Vec3::sAxisZ();
    vs.mForward = Vec3::sAxisY();
    auto* ctrl = new WheeledVehicleControllerSettings();
    vs.mController = ctrl;
    std::vector<int> driven;
    float radius = 0.4f;
    float stiffnessPerMass = 20.0f;
    for (const auto& w : m_scene.wheels) {
      if (w.vehicle != v) continue;
      auto* ws = new WheelSettingsWV();
      ws->mPosition = J(w.pos);
      ws->mSuspensionDirection = J(w.down).Normalized();
      ws->mSteeringAxis = -ws->mSuspensionDirection;
      ws->mWheelUp = Vec3::sAxisZ();
      ws->mWheelForward = Vec3::sAxisY();
      ws->mSuspensionMinLength = 0.05f;
      ws->mSuspensionMaxLength = w.restLength;
      if (w.tuning.set) stiffnessPerMass = w.tuning.suspensionStiffness;
      ws->mSuspensionSpring = SpringSettings(ESpringMode::FrequencyAndDamping, std::sqrt(stiffnessPerMass) / (2 * kPi), 0.5f);
      ws->mRadius = w.radius;
      ws->mWidth = 0.5f * w.radius;
      ws->mMaxSteerAngle = w.steering ? 0.3f : 0.0f;
      ws->mMaxBrakeTorque = 1500.0f;
      ws->mMaxHandBrakeTorque = 0.0f;
      radius = w.radius;
      if (!w.steering) driven.push_back((int)vs.mWheels.size());
      vs.mWheels.push_back(ws);
    }
    if (driven.size() >= 2) {
      VehicleDifferentialSettings diff;
      diff.mLeftWheel = driven[0];
      diff.mRightWheel = driven[1];
      diff.mDifferentialRatio = 1.0f;
      ctrl->mDifferentials.push_back(diff);
    }
    ctrl->mEngine.mMaxTorque = 1500.0f * radius * float(std::max<size_t>(1, driven.size()));
    ctrl->mEngine.mMaxRPM = 20000.0f;
    ctrl->mTransmission.mMode = ETransmissionMode::Manual;
    ctrl->mTransmission.mGearRatios = {1.0f};
    ctrl->mTransmission.mReverseGearRatios = {-1.0f};
    Body* body = lockBody(chassis);
    Ref<VehicleConstraint> vc = new VehicleConstraint(*body, vs);
    vc->SetVehicleCollisionTester(new VehicleCollisionTesterRay(Layers::kMoving, Vec3::sAxisZ()));
    static_cast<WheeledVehicleController*>(vc->GetController())->GetTransmission().Set(1, 1.0f);
    m_physics->AddConstraint(vc);
    m_physics->AddStepListener(vc);
    m_vehicles.push_back(vc);
  }

  void actions() {
    const int f = m_frame;
    for (size_t v = 0; v < m_vehicles.size(); ++v) {
      float engine = 0, brake = 0, steer = 0;
      bool driven = false;
      for (const auto& a : m_scene.actions) {
        if (a.op != bnl::ActionDesc::kVehicle || a.target != (int)v) continue;
        driven = true;
        if (f < a.from || (a.to >= 0 && f > a.to)) continue;
        for (float e : a.engine) engine = std::max(engine, e);
        for (float b : a.brake) brake = std::max(brake, b);
        for (float s : a.steer) if (s != 0) steer = s;
      }
      if (driven) vehicleControl((int)v, 0, engine, brake, steer);
    }
    for (const auto& a : m_scene.actions) {
      if (a.op == bnl::ActionDesc::kVehicle) continue;
      if (f < a.from || (a.to >= 0 && f > a.to)) continue;
      const int b = a.target;
      switch (a.op) {
        case bnl::ActionDesc::kWalk: characterWalk(b, a.v); break;
        case bnl::ActionDesc::kJump: characterJump(b); break;
        case bnl::ActionDesc::kSuspend:
          if (rigid(b)) { bi().SetMotionType(id(b), EMotionType::Kinematic, EActivation::Activate); bi().SetLinearAndAngularVelocity(id(b), Vec3::sZero(), Vec3::sZero()); }
          break;
        case bnl::ActionDesc::kKinematicMove:
          if (rigid(b) && bi().GetMotionType(id(b)) != EMotionType::Static) bi().MoveKinematic(id(b), bi().GetPosition(id(b)) + J(a.v), bi().GetRotation(id(b)), 1.0f / m_scene.settings.ticrate);
          break;
        case bnl::ActionDesc::kLinVel: setLinearVelocity(b, a.v); bi().ActivateBody(id(b)); break;
        case bnl::ActionDesc::kImpulse: applyImpulse(b, a.v); break;
        default: break;
      }
    }
  }

  void updateCharacters(float dt) {
    for (size_t i = 0; i < m_slots.size(); ++i) {
      Slot& s = m_slots[i];
      if (!s.character) continue;
      const bnl::BodyDesc& d = m_scene.bodies[i];
      Vec3 v = s.character->GetLinearVelocity();
      const bool grounded = s.character->GetGroundState() == CharacterBase::EGroundState::OnGround;
      // Bullet's walk direction is a displacement per step; Jolt's character takes a velocity.
      Vec3 h = J(s.walk) / dt;
      float vz = grounded ? 0.0f : v.GetZ();
      if (s.jump && grounded) vz = d.jumpSpeed;
      s.jump = false;
      vz += m_physics->GetGravity().GetZ() * dt;
      vz = std::max(vz, -d.fallSpeed);
      s.character->SetLinearVelocity(Vec3(h.GetX(), h.GetY(), vz));
      CharacterVirtual::ExtendedUpdateSettings us;
      us.mStickToFloorStepDown = Vec3(0, 0, -0.5f * d.stepHeight);
      us.mWalkStairsStepUp = Vec3(0, 0, d.stepHeight);
      s.character->ExtendedUpdate(dt, m_physics->GetGravity(), us,
                                  m_physics->GetDefaultBroadPhaseLayerFilter(Layers::kMoving),
                                  m_physics->GetDefaultLayerFilter(Layers::kMoving), {}, {}, *m_temp);
    }
  }

  bnl::u64 hashFrame() const {
    bnl::u64 h = 1469598103934665603ULL;
    std::vector<bnl::Vec3> nodes;
    for (int i = 0; i < bodyCount(); ++i) {
      const bnl::BodyState s = body(i);
      const float f[13] = {s.p.x, s.p.y, s.p.z, s.q.x, s.q.y, s.q.z, s.q.w, s.lin.x, s.lin.y, s.lin.z, s.ang.x, s.ang.y, s.ang.z};
      for (float x : f) hashFloat(h, x);
      softNodes(i, nodes);
      for (const auto& p : nodes) { hashFloat(h, p.x); hashFloat(h, p.y); hashFloat(h, p.z); }
    }
    const int events = 0;  // collision callbacks are a Bullet/BGE notion; not modelled here
    unsigned char c[4];
    std::memcpy(c, &events, 4);
    for (int i = 0; i < 4; ++i) { h ^= c[i]; h *= 1099511628211ULL; }
    return h;
  }

  bnl::SceneDesc m_scene;
  BroadPhaseLayers m_bpLayers;
  ObjectVsBroadPhase m_objVsBp;
  ObjectPairs m_pairs;
  Ref<UserGroups> m_groups;
  ContactRecorder m_contacts;
  std::unique_ptr<TempAllocator> m_temp;
  std::unique_ptr<JobSystem> m_jobs;
  std::unique_ptr<PhysicsSystem> m_physics;
  std::vector<Slot> m_slots;
  std::map<uint32_t, int> m_bodyIndex;
  std::vector<Ref<TwoBodyConstraint> > m_constraints;
  std::vector<int> m_vehicleChassis;
  std::vector<Ref<VehicleConstraint> > m_vehicles;
  int m_frame = 0;
  double m_micros = 0;
  bnl::u64 m_frameHash = 0, m_final = 0;
};

}  // namespace

bnl::Engine* bnl_create_engine_jolt() { return new JoltEngine(); }
