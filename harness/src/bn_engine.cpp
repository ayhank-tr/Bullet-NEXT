// Bullet NEXT Lab - the Lab engine interface over the harness World (FL, CL.2).
// Compiled once per Bullet backend, inside that backend's namespace, so a program can link the
// legacy and Bullet NEXT engines together. The factory is bnl_create_engine_<tag>().
#include "bnl_engine.h"

#include "bn_load.h"
#include "bn_record.h"

#include <chrono>

BN_NAMESPACE_BEGIN

namespace {

bnl::Vec3 fromBt(const btVector3& v) { return bnl::Vec3((float)v.x(), (float)v.y(), (float)v.z()); }
bnl::Quat fromBt(const btQuaternion& q) { return bnl::Quat((float)q.x(), (float)q.y(), (float)q.z(), (float)q.w()); }

class RayAll : public btCollisionWorld::AllHitsRayResultCallback {
 public:
  RayAll(const btVector3& a, const btVector3& b) : btCollisionWorld::AllHitsRayResultCallback(a, b) {}
};

class EngineImpl : public bnl::Engine {
 public:
  ~EngineImpl() { delete m_world; }

  const char* backend() const { return BN_BACKEND_NAME; }

  bool build(const bnl::SceneDesc& s, const bnl::EngineOptions& o, std::string& error) {
    if (m_world) { error = "engine already built"; return false; }
    m_scene = s;
    Settings st = settingsOf(s);
    st.gravityZ += o.gravityOffset;
    st.threads = o.threads;
    m_world = new World(st);
    bn::build(*m_world, m_scene);
    return true;
  }

  void advance() {
    act(*m_world, m_scene, m_world->frame());
    const auto t0 = std::chrono::steady_clock::now();
    m_world->step();
    const auto t1 = std::chrono::steady_clock::now();
    m_micros = std::chrono::duration<double, std::micro>(t1 - t0).count();
    m_frameHash = hashFrame(*m_world);
    m_final = chainHash(m_final, m_frameHash);
  }

  int frame() const { return m_world ? m_world->frame() : 0; }
  double lastStepMicros() const { return m_micros; }

  int bodyCount() const { return m_world ? (int)m_world->bodies().size() : 0; }

  bnl::BodyState body(int i) const {
    bnl::BodyState s;
    const BodyRec& b = m_world->bodies()[i];
    if (b.soft) {
      btVector3 p(0, 0, 0), v(0, 0, 0);
      const int n = b.soft->m_nodes.size();
      for (int k = 0; k < n; ++k) { p += b.soft->m_nodes[k].m_x; v += b.soft->m_nodes[k].m_v; }
      if (n) { p /= btScalar(n); v /= btScalar(n); }
      s.p = fromBt(p); s.lin = fromBt(v);
      return s;
    }
    const btTransform& t = b.obj->getWorldTransform();
    s.p = fromBt(t.getOrigin());
    s.q = fromBt(t.getRotation());
    if (b.rb) { s.lin = fromBt(b.rb->getLinearVelocity()); s.ang = fromBt(b.rb->getAngularVelocity()); }
    return s;
  }

  void softNodes(int i, std::vector<bnl::Vec3>& out) const {
    out.clear();
    const BodyRec& b = m_world->bodies()[i];
    if (!b.soft) return;
    for (int k = 0; k < b.soft->m_nodes.size(); ++k) out.push_back(fromBt(b.soft->m_nodes[k].m_x));
  }

  int vehicleCount() const { return (int)m_scene.vehicles.size(); }
  int wheelCount(int v) const { return const_cast<World*>(m_world)->vehicle(v)->getNumWheels(); }
  bnl::Xform wheel(int v, int w) const {
    const btTransform& t = const_cast<World*>(m_world)->vehicle(v)->getWheelInfo(w).m_worldTransform;
    bnl::Xform x;
    x.p = fromBt(t.getOrigin());
    x.q = fromBt(t.getRotation());
    return x;
  }

  void contacts(std::vector<bnl::Contact>& out) const {
    out.clear();
    btDispatcher* d = m_world->dyn()->getDispatcher();
    for (int i = 0; i < d->getNumManifolds(); ++i) {
      btPersistentManifold* m = d->getManifoldByIndexInternal(i);
      const int a = static_cast<const btCollisionObject*>(m->getBody0())->getUserIndex();
      const int b = static_cast<const btCollisionObject*>(m->getBody1())->getUserIndex();
      for (int k = 0; k < m->getNumContacts(); ++k) {
        const btManifoldPoint& cp = m->getContactPoint(k);
        bnl::Contact c;
        c.a = a; c.b = b;
        c.pa = fromBt(cp.m_positionWorldOnA);
        c.pb = fromBt(cp.m_positionWorldOnB);
        c.n = fromBt(cp.m_normalWorldOnB);
        c.dist = (float)cp.getDistance();
        out.push_back(c);
      }
    }
  }

  bnl::u64 frameHash() const { return m_frameHash; }
  bnl::u64 finalHash() const { return m_final; }

  void setLinearVelocity(int b, const bnl::Vec3& v) { if (btRigidBody* r = rb(b)) { r->setLinearVelocity(toBt(v)); r->activate(true); } }
  void setAngularVelocity(int b, const bnl::Vec3& v) { if (btRigidBody* r = rb(b)) { r->setAngularVelocity(toBt(v)); r->activate(true); } }
  void applyForce(int b, const bnl::Vec3& f) { if (btRigidBody* r = rb(b)) { r->applyCentralForce(toBt(f)); r->activate(true); } }
  void applyForceAt(int b, const bnl::Vec3& f, const bnl::Vec3& p) {
    if (btRigidBody* r = rb(b)) { r->applyForce(toBt(f), toBt(p) - r->getCenterOfMassPosition()); r->activate(true); }
  }
  void applyImpulse(int b, const bnl::Vec3& j) { if (btRigidBody* r = rb(b)) { r->applyCentralImpulse(toBt(j)); r->activate(true); } }
  void applyImpulseAt(int b, const bnl::Vec3& j, const bnl::Vec3& p) {
    if (btRigidBody* r = rb(b)) { r->applyImpulse(toBt(j), toBt(p) - r->getCenterOfMassPosition()); r->activate(true); }
  }
  void applyTorque(int b, const bnl::Vec3& t) { if (btRigidBody* r = rb(b)) { r->applyTorque(toBt(t)); r->activate(true); } }
  void applyTorqueImpulse(int b, const bnl::Vec3& t) { if (btRigidBody* r = rb(b)) { r->applyTorqueImpulse(toBt(t)); r->activate(true); } }

  void setTransform(int b, const bnl::Xform& x) {
    BodyRec& r = m_world->body(b);
    const btTransform t = toBt(x);
    if (r.rb && r.rb->isKinematicObject()) { m_world->setKinematicTransform(b, t); return; }
    if (r.rb) {
      r.rb->setWorldTransform(t);
      r.rb->setCenterOfMassTransform(t);
      if (r.rb->getMotionState()) r.rb->getMotionState()->setWorldTransform(t);
      r.rb->activate(true);
    } else if (r.obj) {
      r.obj->setWorldTransform(t);
    }
  }

  void wake(int b) { if (btCollisionObject* o = m_world->body(b).obj) o->activate(true); }
  void setSleeping(int b, bool allowed) {
    btCollisionObject* o = m_world->body(b).obj;
    if (!o) return;
    if (allowed) o->forceActivationState(ACTIVE_TAG);
    else o->forceActivationState(DISABLE_DEACTIVATION);
  }

  bool raycast(const bnl::Vec3& a, const bnl::Vec3& b, bnl::RayHit& hit) const {
    btCollisionWorld::ClosestRayResultCallback cb(toBt(a), toBt(b));
    m_world->dyn()->rayTest(toBt(a), toBt(b), cb);
    if (!cb.hasHit()) return false;
    hit.body = cb.m_collisionObject->getUserIndex();
    hit.p = fromBt(cb.m_hitPointWorld);
    hit.n = fromBt(cb.m_hitNormalWorld);
    hit.t = (float)cb.m_closestHitFraction;
    return true;
  }

  void raycastAll(const bnl::Vec3& a, const bnl::Vec3& b, std::vector<bnl::RayHit>& hits) const {
    hits.clear();
    RayAll cb(toBt(a), toBt(b));
    m_world->dyn()->rayTest(toBt(a), toBt(b), cb);
    for (int i = 0; i < cb.m_collisionObjects.size(); ++i) {
      bnl::RayHit h;
      h.body = cb.m_collisionObjects[i]->getUserIndex();
      h.p = fromBt(cb.m_hitPointWorld[i]);
      h.n = fromBt(cb.m_hitNormalWorld[i]);
      h.t = (float)cb.m_hitFractions[i];
      hits.push_back(h);
    }
  }

  void vehicleControl(int v, int w, float engine, float brake, float steer) {
    btRaycastVehicle* rv = m_world->vehicle(v);
    rv->applyEngineForce(engine, w);
    rv->setBrake(brake, w);
    rv->setSteeringValue(steer, w);
  }

  void characterWalk(int b, const bnl::Vec3& step) {
    if (btKinematicCharacterController* c = m_world->body(b).character) c->setWalkDirection(toBt(step));
  }
  void characterJump(int b) {
    if (btKinematicCharacterController* c = m_world->body(b).character) c->jump();
  }

 private:
  btRigidBody* rb(int b) { return m_world->body(b).rb; }

  bnl::SceneDesc m_scene;
  World* m_world = nullptr;
  double m_micros = 0;
  bnl::u64 m_frameHash = 0, m_final = 0;
};

}  // namespace

BN_NAMESPACE_END

#define BN_CAT2(a, b) a##b
#define BN_CAT(a, b) BN_CAT2(a, b)

bnl::Engine* BN_CAT(bnl_create_engine_, BN_TAG)() { return new bn::BN_TAG::EngineImpl(); }
