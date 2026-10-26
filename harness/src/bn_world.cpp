// Bullet NEXT golden harness - BGE 2.79b world factory (commits C0.4, C0.5).
#include "bn_world.h"

namespace bn {

// ---------------------------------------------------------------------------------------
// CcdOverlapFilterCallBack: Bullet group/mask test + user col_group/col_mask test.
// Sensor broadphase callbacks are game-engine side; sensors here always pass.
class BgeOverlapFilter : public btOverlapFilterCallback {
 public:
  explicit BgeOverlapFilter(World* w) : m_w(w) {}
  virtual bool needBroadphaseCollision(btBroadphaseProxy* p0, btBroadphaseProxy* p1) const {
    bool c = (p0->m_collisionFilterGroup & p1->m_collisionFilterMask) != 0;
    c = c && (p1->m_collisionFilterGroup & p0->m_collisionFilterMask);
    btCollisionObject* o0 = static_cast<btCollisionObject*>(p0->m_clientObject);
    btCollisionObject* o1 = static_cast<btCollisionObject*>(p1->m_clientObject);
    if (c && o0 && o1 && o0->getUserIndex() >= 0 && o1->getUserIndex() >= 0)
      c = m_w->collides(o0->getUserIndex(), o1->getUserIndex());
    return c;
  }
 private:
  World* m_w;
};

// BlenderVehicleRaycaster: closest hit, sub-simplex raytest, contact-response bodies only.
class BgeVehicleRaycaster : public btDefaultVehicleRaycaster {
 public:
  explicit BgeVehicleRaycaster(btDynamicsWorld* w) : btDefaultVehicleRaycaster(w), m_dw(w) {}
  virtual void* castRay(const btVector3& from, const btVector3& to, btVehicleRaycasterResult& r) {
    btCollisionWorld::ClosestRayResultCallback cb(from, to);
    cb.m_flags |= btTriangleRaycastCallback::kF_UseSubSimplexConvexCastRaytest;
    m_dw->rayTest(from, to, cb);
    if (cb.hasHit()) {
      const btRigidBody* body = btRigidBody::upcast(cb.m_collisionObject);
      if (body && body->hasContactResponse()) {
        r.m_hitPointInWorld = cb.m_hitPointWorld;
        r.m_hitNormalInWorld = cb.m_hitNormalWorld;
        r.m_hitNormalInWorld.normalize();
        r.m_distFraction = cb.m_closestHitFraction;
        return (void*)body;
      }
    }
    return 0;
  }
 private:
  btDynamicsWorld* m_dw;
};

// BlenderBulletCharacterController: multi-jump counter on top of Bullet's controller.
class BgeCharacter : public btKinematicCharacterController {
 public:
  BgeCharacter(btPairCachingGhostObject* g, btConvexShape* s, float stepHeight)
#if BN_NEW_API
      : btKinematicCharacterController(g, s, stepHeight, btVector3(0, 0, 1)) {}
#else
      : btKinematicCharacterController(g, s, stepHeight, 2) {}
#endif
  virtual void updateAction(btCollisionWorld* w, btScalar dt) {
    if (onGround()) m_jumps = 0;
    btKinematicCharacterController::updateAction(w, dt);
  }
  int m_jumps = 0, m_maxJumps = 1;
};

static void tickCallback(btDynamicsWorld* w, btScalar dt) {
  static_cast<World*>(w->getWorldUserInfo())->tick(dt);
}

// ---------------------------------------------------------------------------------------
World::World(const Settings& s) : m_settings(s) {
  m_config = new btSoftBodyRigidBodyCollisionConfiguration();
  m_dispatcher = new btCollisionDispatcher(m_config);
  btGImpactCollisionAlgorithm::registerAlgorithm(m_dispatcher);
  m_broadphase = new btDbvtBroadphase();
  m_filter = new BgeOverlapFilter(this);
  m_ghostCb = new btGhostPairCallback();
  m_broadphase->getOverlappingPairCache()->setOverlapFilterCallback(m_filter);
  m_broadphase->getOverlappingPairCache()->setInternalGhostPairCallback(m_ghostCb);
  m_solver = new btSequentialImpulseConstraintSolver();  // SetSolverType(1)
  m_world = new btSoftRigidDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_config);
  m_world->setInternalTickCallback(&tickCallback, this);
  m_world->setGravity(btVector3(0, 0, s.gravityZ));
  m_world->getWorldInfo().m_gravity = btVector3(0, 0, s.gravityZ);
  if (s.setSolverParams) {
    m_world->getSolverInfo().m_sor = s.sor;
    m_world->getSolverInfo().m_tau = s.tau;
    m_world->getSolverInfo().m_damping = s.damping;
  }
}

World::~World() {
  for (size_t i = 0; i < m_vehicles.size(); ++i) { m_world->removeVehicle(m_vehicles[i]); delete m_vehicles[i]; }
  for (size_t i = 0; i < m_raycasters.size(); ++i) delete m_raycasters[i];
  for (size_t i = 0; i < m_tunings.size(); ++i) delete m_tunings[i];
  for (size_t i = 0; i < m_constraints.size(); ++i) { m_world->removeConstraint(m_constraints[i]); delete m_constraints[i]; }
  for (size_t i = 0; i < m_bodies.size(); ++i) {
    BodyRec& b = m_bodies[i];
    if (b.character) { m_world->removeAction(b.character); delete b.character; }
    if (b.soft) m_world->removeSoftBody(b.soft);
    else if (b.rb) { m_world->removeRigidBody(b.rb); delete b.rb->getMotionState(); }
    else if (b.obj) m_world->removeCollisionObject(b.obj);
    delete b.obj;
    if (!b.soft) delete b.opts.shape;
  }
  for (size_t i = 0; i < m_meshes.size(); ++i) delete m_meshes[i];
  delete m_fixed;
  delete m_world; delete m_solver; delete m_ghostCb; delete m_filter;
  delete m_broadphase; delete m_dispatcher; delete m_config;
}

bool World::collides(int a, int b) const {
  const BodyOpts& A = m_bodies[a].opts;
  const BodyOpts& B = m_bodies[b].opts;
  return (A.userGroup & B.userMask) && (B.userGroup & A.userMask);
}

void World::tick(btScalar) {
  // CcdPhysicsController::SimulationTick, in creation order.
  for (size_t i = 0; i < m_bodies.size(); ++i) {
    BodyRec& b = m_bodies[i];
    btRigidBody* body = b.rb;
    if (!body || body->isStaticObject()) continue;
    const BodyOpts& o = b.opts;
    if (o.clampVelMax > 0.0f || o.clampVelMin > 0.0f) {
      const btVector3& v = body->getLinearVelocity();
      btScalar len = v.length();
      if (o.clampVelMax > 0.0f && len > o.clampVelMax) body->setLinearVelocity(v * (o.clampVelMax / len));
      else if (o.clampVelMin > 0.0f && !btFuzzyZero(len) && len < o.clampVelMin) body->setLinearVelocity(v * (o.clampVelMin / len));
    }
    if (o.clampAngMax > 0.0f || o.clampAngMin > 0.0f) {
      const btVector3& w = body->getAngularVelocity();
      btScalar len = w.length();
      if (o.clampAngMax > 0.0f && len > o.clampAngMax) body->setAngularVelocity(w * (o.clampAngMax / len));
      else if (o.clampAngMin > 0.0f && !btFuzzyZero(len) && len < o.clampAngMin) body->setAngularVelocity(w * (o.clampAngMin / len));
    }
  }
}

int World::add(const BodyOpts& o) {
  BodyRec rec;
  rec.opts = o;
  const bool dyn = (o.kind == kDynamic || o.kind == kRigid);
  rec.mass = dyn ? o.mass : 0.0f;
  btVector3 inertia(0, 0, 0);
  if (rec.mass) o.shape->calculateLocalInertia(rec.mass, inertia);
  btRigidBody::btRigidBodyConstructionInfo ci(rec.mass, new btDefaultMotionState(o.xform), o.shape,
                                              inertia * (o.inertia / 0.4f));
  ci.m_linearDamping = o.linDamp;
  ci.m_angularDamping = o.angDamp;
  ci.m_friction = o.friction;
  ci.m_restitution = o.restitution;
  btRigidBody* body = new btRigidBody(ci);
  int flags = 0;
  if (o.kind == kStatic) flags |= btCollisionObject::CF_STATIC_OBJECT;
  if (o.ghost || o.kind == kSensor) flags |= btCollisionObject::CF_NO_CONTACT_RESPONSE;
  if (o.kind == kSensor) { body->setCollisionFlags(0); body->setActivationState(DISABLE_DEACTIVATION); }
  body->setCollisionFlags(body->getCollisionFlags() | flags);
  body->setGravity(btVector3(0, 0, m_settings.gravityZ));
  body->setDamping(o.linDamp, o.angDamp);
  if (o.kind == kDynamic) body->setAngularFactor(0.f);
  if (o.kind == kRigid) { body->setLinearFactor(o.linFactor); body->setAngularFactor(o.angFactor); }
  body->setSleepingThresholds(m_settings.linSleep, m_settings.angSleep);
  rec.group = o.kind == kSensor ? kSensorFilter : dyn ? kDefaultFilter : kStaticFilter;
  rec.mask = o.kind == kSensor ? (kAllFilter ^ kSensorFilter) : dyn ? kAllFilter : (kAllFilter ^ kStaticFilter);
  rec.obj = body;
  rec.rb = body;
  const int index = (int)m_bodies.size();
  body->setUserIndex(index);
  m_bodies.push_back(rec);
  m_world->addRigidBody(body, rec.group, rec.mask);
  if (body->isStaticOrKinematicObject()) body->setActivationState(ISLAND_SLEEPING);
  return index;
}

int World::addCharacter(const BodyOpts& o, float stepHeight, float jumpSpeed, float fallSpeed) {
  BodyRec rec;
  rec.opts = o;
  rec.opts.kind = kCharacter;
  btPairCachingGhostObject* ghost = new btPairCachingGhostObject();
  ghost->setCollisionShape(o.shape);
  ghost->setCollisionFlags(btCollisionObject::CF_CHARACTER_OBJECT);
  ghost->setWorldTransform(o.xform);
  BgeCharacter* ch = new BgeCharacter(ghost, static_cast<btConvexShape*>(o.shape), stepHeight);
  ch->setJumpSpeed(jumpSpeed);
  ch->setFallSpeed(fallSpeed);
  rec.obj = ghost;
  rec.character = ch;
  rec.group = kCharacterFilter;
  rec.mask = kAllFilter;
  const int index = (int)m_bodies.size();
  ghost->setUserIndex(index);
  m_bodies.push_back(rec);
  m_world->addCollisionObject(ghost, rec.group, rec.mask);
  m_world->addAction(ch);
  return index;
}

int World::addSoftFromTriMesh(const std::vector<btVector3>& verts, const std::vector<int>& tris,
                              const BodyOpts& o, bool bending, bool shapeMatching) {
  // CcdPhysicsController::CreateSoftbody (triangle-mesh path) with BGE's default settings.
  std::vector<btScalar> flat;
  for (size_t i = 0; i < verts.size(); ++i) {
    btVector3 p = o.xform * verts[i];
    flat.push_back(p.x()); flat.push_back(p.y()); flat.push_back(p.z());
  }
  btSoftBody* psb = btSoftBodyHelpers::CreateFromTriMesh(m_world->getWorldInfo(), &flat[0], &tris[0],
                                                         (int)tris.size() / 3);
  psb->m_cfg.collisions = btSoftBody::fCollision::SDF_RS | btSoftBody::fCollision::VF_SS;
  psb->m_cfg.kDF = o.friction;
  psb->m_cfg.piterations = 2;
  psb->m_materials[0]->m_kLST = 0.5f;
  psb->m_materials[0]->m_kAST = 0.5f;
  psb->m_materials[0]->m_kVST = 0.5f;
  if (bending) psb->generateBendingConstraints(2, psb->m_materials[0]);
  psb->getCollisionShape()->setMargin(0.1f);
  if (shapeMatching) psb->setPose(false, true); else psb->setPose(true, false);
  psb->randomizeConstraints();
  psb->setTotalMass(o.mass);
  BodyRec rec;
  rec.opts = o;
  rec.opts.kind = kSoft;
  rec.opts.shape = nullptr;
  rec.obj = psb;
  rec.soft = psb;
  rec.mass = o.mass;
  const int index = (int)m_bodies.size();
  psb->setUserIndex(index);
  m_bodies.push_back(rec);
  m_world->addSoftBody(psb);
  return index;
}

btRigidBody* World::rigidOf(int i) { return i < 0 ? &fixedBody() : m_bodies[i].rb; }

btRigidBody& World::fixedBody() {
  if (!m_fixed) {
    m_fixed = new btRigidBody(0, 0, 0);  // static btRigidBody s_fixedObject2(0, 0, 0)
  }
  return *m_fixed;
}

int World::addPointToPoint(int a, int b, const btVector3& pa, const btVector3& pb) {
  // BGE uses the single-body constructor for point-to-point (pivotB ignored).
  btTypedConstraint* c = b < 0 ? new btPoint2PointConstraint(*rigidOf(a), pa)
                               : new btPoint2PointConstraint(*rigidOf(a), *rigidOf(b), pa, pb);
  c->setUserConstraintType(1);
  m_world->addConstraint(c, true);
  m_constraints.push_back(c);
  return (int)m_constraints.size() - 1;
}

int World::addHinge(int a, int b, const btTransform& fa, const btTransform& fb, bool angularOnly) {
  // No second object: BGE attaches to a static s_fixedObject2 with frameB in world space.
  btHingeConstraint* c = b < 0 ? new btHingeConstraint(*rigidOf(a), fixedBody(), fa, rigidOf(a)->getCenterOfMassTransform() * fa)
                               : new btHingeConstraint(*rigidOf(a), *rigidOf(b), fa, fb);
  c->setAngularOnly(angularOnly);
  c->setUserConstraintType(angularOnly ? 3 : 2);
  m_world->addConstraint(c, true);
  m_constraints.push_back(c);
  return (int)m_constraints.size() - 1;
}

int World::addConeTwist(int a, int b, const btTransform& fa, const btTransform& fb) {
  btConeTwistConstraint* c = b < 0 ? new btConeTwistConstraint(*rigidOf(a), fixedBody(), fa, rigidOf(a)->getCenterOfMassTransform() * fa)
                                   : new btConeTwistConstraint(*rigidOf(a), *rigidOf(b), fa, fb);
  c->setUserConstraintType(4);
  m_world->addConstraint(c, true);
  m_constraints.push_back(c);
  return (int)m_constraints.size() - 1;
}

int World::addGeneric6DofSpring(int a, int b, const btTransform& fa, const btTransform& fb) {
  btGeneric6DofSpringConstraint* c =
      new btGeneric6DofSpringConstraint(*rigidOf(a), *rigidOf(b), fa, fb, true);
  c->setUserConstraintType(12);
  m_world->addConstraint(c, true);
  m_constraints.push_back(c);
  return (int)m_constraints.size() - 1;
}

void World::setParam(int ci, int param, float v0, float v1) {
  btTypedConstraint* tc = m_constraints[ci];
  switch (tc->getUserConstraintType()) {
    case 12: {
      btGeneric6DofSpringConstraint* g = static_cast<btGeneric6DofSpringConstraint*>(tc);
      if (param <= 5) g->setLimit(param, v0, v1);
      else if (param <= 8) {
        btTranslationalLimitMotor* m = g->getTranslationalLimitMotor();
        m->m_targetVelocity[param - 6] = v0;
        m->m_maxMotorForce[param - 6] = v1;
        m->m_enableMotor[param - 6] = (v1 > 0.f);
      } else if (param <= 11) {
        btRotationalLimitMotor* m = g->getRotationalLimitMotor(param - 9);
        m->m_enableMotor = (v1 > 0.f);
        m->m_targetVelocity = v0;
        m->m_maxMotorForce = v1;
      } else if (param <= 17) {
        int s = param - 12;
        if (v0 != 0.f) { g->setStiffness(s, v0); g->setDamping(s, v1); g->enableSpring(s, true); g->setEquilibriumPoint(s); }
        else g->enableSpring(s, false);
      }
      break;
    }
    case 4:
      if (param >= 3 && param <= 5)
        static_cast<btConeTwistConstraint*>(tc)->setLimit(param, v1 < 0.0f ? btScalar(BT_LARGE_FLOAT) : v1);
      break;
    case 2: case 3:
      if (param == 3) static_cast<btHingeConstraint*>(tc)->setLimit(v0, v1);
      break;
  }
}

int World::addVehicle(int chassis) {
  btRaycastVehicle::btVehicleTuning* tuning = new btRaycastVehicle::btVehicleTuning();
  btVehicleRaycaster* rc = new BgeVehicleRaycaster(m_world);
  btRaycastVehicle* v = new btRaycastVehicle(*tuning, m_bodies[chassis].rb, rc);
  m_world->addVehicle(v);
  m_tunings.push_back(tuning);
  m_raycasters.push_back(rc);
  m_vehicles.push_back(v);
  return (int)m_vehicles.size() - 1;
}

void World::addWheel(int vi, const btVector3& pos, const btVector3& down, const btVector3& axle,
                     float restLen, float radius, bool steering) {
  static btRaycastVehicle::btVehicleTuning gTuning;  // BGE's static gTuning
  btRaycastVehicle* v = m_vehicles[vi];
  btWheelInfo& info = v->addWheel(pos, down.normalized(), axle.normalized(), restLen, radius, gTuning, steering);
  info.m_clientInfo = 0;
}

void World::suspendDynamics(int i) {
  // CcdPhysicsController::SuspendDynamics(ghost=false)
  btRigidBody* body = m_bodies[i].rb;
  if (!body) return;
  btBroadphaseProxy* h = body->getBroadphaseHandle();
  short group = h->m_collisionFilterGroup, mask = h->m_collisionFilterMask;
  m_world->removeRigidBody(body);
  body->setMassProps(0, btVector3(0, 0, 0));
  body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);
  body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
  m_world->addRigidBody(body, group, mask);
  body->setActivationState(DISABLE_DEACTIVATION);
}

void World::setKinematicTransform(int i, const btTransform& t) {
  btRigidBody* body = m_bodies[i].rb;
  body->getMotionState()->setWorldTransform(t);
  body->setWorldTransform(t);
  body->activate(true);
}

void World::step() {
  // ProceedDeltaTime(curTime, timeStep, interval) with a fixed frame rate.
  gDeactivationTime = m_settings.deactivationTime;
  gContactBreakingThreshold = m_settings.contactBreaking;
  const float timeStep = (float)(1.0 / (double)m_settings.ticrate);
  const float interval = timeStep;
  const float subStep = timeStep / float(m_settings.substeps);
  m_world->stepSimulation(interval, 25, subStep);
  for (size_t v = 0; v < m_vehicles.size(); ++v)
    for (int w = 0; w < m_vehicles[v]->getNumWheels(); ++w)
      m_vehicles[v]->updateWheelTransform(w, false);

  // CallbackTriggers: point 0 of every manifold touching a registered object.
  m_events.clear();
  btDispatcher* d = m_world->getDispatcher();
  const int n = d->getNumManifolds();
  for (int i = 0; i < n; ++i) {
    btPersistentManifold* m = d->getManifoldByIndexInternal(i);
    if (!m->getNumContacts()) continue;
    const btCollisionObject* o0 = static_cast<const btCollisionObject*>(m->getBody0());
    const btCollisionObject* o1 = static_cast<const btCollisionObject*>(m->getBody1());
    const int i0 = o0->getUserIndex(), i1 = o1->getUserIndex();
    bool first = true, use = false;
    if (i0 >= 0 && m_bodies[i0].opts.reportCollisions) use = true;
    else if (i1 >= 0 && m_bodies[i1].opts.reportCollisions) { first = false; use = true; }
    if (use) {
      const btManifoldPoint& cp = m->getContactPoint(0);
      ContactEvent e;
      e.a = i0; e.b = i1;
      e.p1 = first ? cp.m_positionWorldOnA : cp.m_positionWorldOnB;
      e.p2 = first ? cp.m_positionWorldOnB : cp.m_positionWorldOnA;
      e.normal = first ? -cp.m_normalWorldOnB : cp.m_normalWorldOnB;
      m_events.push_back(e);
    }
    if (!d->needsResponse(o0, o1)) m->clearManifold();
  }
  ++m_frame;
}

// ---------------------------------------------------------------------------------------
btCollisionShape* makeBox(const btVector3& half, float margin) {
  btCollisionShape* s = new btBoxShape(half); s->setMargin(margin); return s;
}
btCollisionShape* makeSphere(float r, float margin) {
  btCollisionShape* s = new btSphereShape(r); s->setMargin(margin); return s;
}
btCollisionShape* makeCylinderZ(const btVector3& half, float margin) {
  btCollisionShape* s = new btCylinderShapeZ(half); s->setMargin(margin); return s;
}
btCollisionShape* makeConeZ(float r, float h, float margin) {
  btCollisionShape* s = new btConeShapeZ(r, h); s->setMargin(margin); return s;
}
btCollisionShape* makeCapsuleZ(float r, float h, float margin) {
  btCollisionShape* s = new btCapsuleShapeZ(r, h); s->setMargin(margin); return s;
}
btCollisionShape* makeHull(const std::vector<btVector3>& pts, float margin) {
  std::vector<btScalar> flat;
  for (size_t i = 0; i < pts.size(); ++i) { flat.push_back(pts[i].x()); flat.push_back(pts[i].y()); flat.push_back(pts[i].z()); }
  btCollisionShape* s = new btConvexHullShape(&flat[0], (int)pts.size(), 3 * sizeof(btScalar));
  s->setMargin(margin);
  return s;
}

namespace {
struct MeshStore {  // keeps vertex/index memory alive for btTriangleIndexVertexArray
  std::vector<std::vector<btScalar> > v;
  std::vector<std::vector<int> > t;
};
MeshStore& store() { static MeshStore s; return s; }
}  // namespace

btCollisionShape* makeMesh(const std::vector<btVector3>& verts, const std::vector<int>& tris,
                           bool dynamic, const btVector3& scale, float margin) {
  MeshStore& st = store();
  st.v.push_back(std::vector<btScalar>());
  st.t.push_back(tris);
  std::vector<btScalar>& fv = st.v.back();
  for (size_t i = 0; i < verts.size(); ++i) { fv.push_back(verts[i].x()); fv.push_back(verts[i].y()); fv.push_back(verts[i].z()); }
  btTriangleIndexVertexArray* arr = new btTriangleIndexVertexArray(
      (int)tris.size() / 3, &st.t.back()[0], 3 * sizeof(int), (int)verts.size(), &fv[0], 3 * sizeof(btScalar));
  if (dynamic) {
    btGImpactMeshShape* g = new btGImpactMeshShape(arr);
    g->setMargin(margin);
    g->setLocalScaling(scale);
    g->updateBound();
    return g;
  }
  btBvhTriangleMeshShape* bvh = new btBvhTriangleMeshShape(arr, true);
  bvh->setMargin(margin);
  // BGE sets the margin on the scaled wrapper too (CcdShapeConstructionInfo::CreateBulletShape).
  btScaledBvhTriangleMeshShape* scaled = new btScaledBvhTriangleMeshShape(bvh, scale);
  scaled->setMargin(margin);
  return scaled;
}

void gridMesh(int nx, int ny, float size, std::vector<btVector3>& v, std::vector<int>& t) {
  v.clear(); t.clear();
  for (int j = 0; j <= ny; ++j)
    for (int i = 0; i <= nx; ++i)
      v.push_back(btVector3(-size + 2 * size * i / nx, -size + 2 * size * j / ny, 0));
  for (int j = 0; j < ny; ++j)
    for (int i = 0; i < nx; ++i) {
      int a = j * (nx + 1) + i, b = a + 1, c = a + nx + 1, d = c + 1;
      t.push_back(a); t.push_back(b); t.push_back(d);
      t.push_back(a); t.push_back(d); t.push_back(c);
    }
}

void boxMesh(const btVector3& h, std::vector<btVector3>& v, std::vector<int>& t) {
  v.clear(); t.clear();
  for (int i = 0; i < 8; ++i)
    v.push_back(btVector3(i & 1 ? h.x() : -h.x(), i & 2 ? h.y() : -h.y(), i & 4 ? h.z() : -h.z()));
  const int f[36] = {0, 2, 1, 1, 2, 3, 4, 5, 6, 5, 7, 6, 0, 1, 4, 1, 5, 4,
                     2, 6, 3, 3, 6, 7, 0, 4, 2, 2, 4, 6, 1, 3, 5, 3, 7, 5};
  t.assign(f, f + 36);
}

}  // namespace bn
