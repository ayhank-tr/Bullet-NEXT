// Bullet NEXT golden harness - BGE 2.79b world factory (commits C0.4, C0.5).
#include "bn_world.h"
#include "bn_rbmath.h"

#include <cfloat>

#include <cstdlib>

BN_NAMESPACE_BEGIN

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

// rbFilterCallback (Blender 5.3 intern/rigidbody): Bullet group/mask + collision collections.
class RbOverlapFilter : public btOverlapFilterCallback {
 public:
  explicit RbOverlapFilter(World* w) : m_w(w) {}
  virtual bool needBroadphaseCollision(btBroadphaseProxy* p0, btBroadphaseProxy* p1) const {
    bool c = (p0->m_collisionFilterGroup & p1->m_collisionFilterMask) != 0;
    c = c && (p1->m_collisionFilterGroup & p0->m_collisionFilterMask);
    btCollisionObject* o0 = static_cast<btCollisionObject*>(p0->m_clientObject);
    btCollisionObject* o1 = static_cast<btCollisionObject*>(p1->m_clientObject);
    c = c && (m_w->bodies()[o0->getUserIndex()].opts.colGroups & m_w->bodies()[o1->getUserIndex()].opts.colGroups);
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

// Gate switch (B1.2 feature cost): BN_FEATURES=<bitmask> keeps only the listed Bullet NEXT features
// (1 world config, 2 linked collision, 4 hull clipping, 8 active edges, 16 CCD). Default: all on.
static bool envFeatureOn(int bit) {
  static int mask = -2;
  if (mask == -2) { const char* e = std::getenv("BN_FEATURES"); mask = e ? std::atoi(e) : -1; }
  return (mask & bit) != 0;
}

bool World::featureOn(int bit) const {
  return m_settings.features >= 0 ? (m_settings.features & bit) != 0 : envFeatureOn(bit);
}

static void tickCallback(btDynamicsWorld* w, btScalar dt) {
  static_cast<World*>(w->getWorldUserInfo())->tick(dt);
}

// ---------------------------------------------------------------------------------------
#if BN_NEXT_FEATURES
// Gate switch (T1.2, B1.1): BN_THREADS=<n> runs the threaded soft-rigid world on n threads, as
// BGE NEXT does (C1.9). Unset or 0 keeps the classic single-threaded world.
static int threadCount(int fromSettings) {
  if (fromSettings >= 0) return fromSettings;
  const char* e = std::getenv("BN_THREADS");
  return e ? std::atoi(e) : 0;
}
#endif

#if BN_NEXT_LIB
// Soft Step solver (F3c): the scene's settings over the solver's defaults.
btSoftStepConstraintSolver* World::newSoftStepSolver() {
  btSoftStepSettings ss;
  if (m_settings.solverSubsteps > 0) ss.m_substeps = m_settings.solverSubsteps;
  if (m_settings.substepRate > 0) ss.m_substepRate = m_settings.substepRate;
  if (m_settings.solverIterations > 0) ss.m_iterations = m_settings.solverIterations;
  if (m_settings.relaxIterations >= 0) ss.m_relaxIterations = m_settings.relaxIterations;
  if (m_settings.manifoldIterations > 0) ss.m_manifoldIterations = m_settings.manifoldIterations;
  if (m_settings.jointIterations > 0) ss.m_jointIterations = m_settings.jointIterations;
  if (m_settings.jointWarmStart >= 0) ss.m_jointWarmStart = m_settings.jointWarmStart;
  if (m_settings.jointHertz >= 0) ss.m_jointHertz = m_settings.jointHertz;
  if (m_settings.jointDampingRatio >= 0) ss.m_jointDampingRatio = m_settings.jointDampingRatio;
  if (m_settings.contactHertz >= 0) ss.m_contactHertz = m_settings.contactHertz;
  if (m_settings.contactDampingRatio >= 0) ss.m_contactDampingRatio = m_settings.contactDampingRatio;
  if (m_settings.contactPushSpeed >= 0) ss.m_contactPushMaxVelocity = m_settings.contactPushSpeed;
  btSoftStepConstraintSolver* solver = new btSoftStepConstraintSolver();
  solver->setSettings(ss);
  m_softStepSolvers.push_back(solver);
  return solver;
}
#endif

World::World(const Settings& s) : m_settings(s) {
  if (s.profile == kBlenderRb) {
    // RB_dworld_new (Blender 5.3 intern/rigidbody/rb_bullet_api.cpp)
    m_config = new btDefaultCollisionConfiguration();
    m_dispatcher = new btCollisionDispatcher(m_config);
    btGImpactCollisionAlgorithm::registerAlgorithm(m_dispatcher);
    m_broadphase = new btDbvtBroadphase();
    m_filter = new RbOverlapFilter(this);
    m_broadphase->getOverlappingPairCache()->setOverlapFilterCallback(m_filter);
#if BN_NEXT_LIB
    if (s.softStep) m_solver = newSoftStepSolver();
    else
#endif
    m_solver = new btSequentialImpulseConstraintSolver();
    m_world = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_config);
    m_world->setGravity(btVector3(0, 0, s.gravityZ));
#if BN_NEXT_FEATURES
    if (featureOn(1)) btNextConfigureWorld(m_world);
    m_meshReduction = featureOn(1);
#endif
    applyGlobals();
    return;
  }
  m_config = new btSoftBodyRigidBodyCollisionConfiguration();
  m_broadphase = new btDbvtBroadphase();
  m_filter = new BgeOverlapFilter(this);
  m_ghostCb = new btGhostPairCallback();
  m_broadphase->getOverlappingPairCache()->setOverlapFilterCallback(m_filter);
  m_broadphase->getOverlappingPairCache()->setInternalGhostPairCallback(m_ghostCb);
#if BN_NEXT_FEATURES
  const int threads = threadCount(s.threads);
  if (threads > 0) {
    static bool scheduler = false;
    if (!scheduler) {
      btITaskScheduler* ts = btCreateDefaultTaskScheduler();
      ts->setNumThreads(threads);
      btSetTaskScheduler(ts);
      scheduler = true;
    }
    m_dispatcher = new btCollisionDispatcherMt(m_config, 40);
    btGImpactCollisionAlgorithm::registerAlgorithm(m_dispatcher);
    if (s.softStep) {
      // one Soft Step solver per thread; every island goes to the pool (no large-island solver)
      std::vector<btConstraintSolver*> solvers;
      for (int i = 0; i < threads; ++i) solvers.push_back(newSoftStepSolver());
      m_solverPool = new btConstraintSolverPoolMt(&solvers[0], threads);
      m_solver = nullptr;
    } else {
      m_solverPool = new btConstraintSolverPoolMt(threads);
      m_solver = new btSequentialImpulseConstraintSolverMt();
    }
    m_softMt = new btSoftRigidDynamicsWorldMt(m_dispatcher, m_broadphase, m_solverPool, m_solver, m_config);
    m_world = m_softMt;
  } else
#endif
  {
    m_dispatcher = new btCollisionDispatcher(m_config);
    btGImpactCollisionAlgorithm::registerAlgorithm(m_dispatcher);
#if BN_NEXT_LIB
    if (s.softStep) m_solver = newSoftStepSolver();       // F3c: Solver "Soft Step"
    else if (s.nncg) m_solver = new btNNCGConstraintSolver();  // BGE NEXT: Solver "NNCG"
    else
#endif
    m_solver = new btSequentialImpulseConstraintSolver();  // SetSolverType(1)
    m_soft = new btSoftRigidDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_config);
    m_world = m_soft;
  }
  m_world->setInternalTickCallback(&tickCallback, this);
#if BN_NEXT_FEATURES
  if (featureOn(1)) {
    btNextConfigureWorld(m_world);                               // C1.3: ordered pairs, SAT for triangles
    // Bullet 2.84 (Blender 2.79b) recovered deep penetration with erp2 = 0.8; upstream lowered it
    // to 0.2, which leaves resting bodies up to 4x deeper. The BGE world keeps 2.79b's value.
    m_world->getSolverInfo().m_erp2 = btScalar(0.8);
  }
  m_meshReduction = featureOn(1);                                // C1.6b, part of the world configuration
  m_linkedCollision = featureOn(2);                              // C1.1: Blender's linked-collision semantics
#endif
  applyGlobals();
  m_world->setGravity(btVector3(0, 0, s.gravityZ));
  softInfo().m_gravity = btVector3(0, 0, s.gravityZ);
  if (s.setSolverParams) {
    m_world->getSolverInfo().m_sor = s.sor;
    m_world->getSolverInfo().m_tau = s.tau;
    m_world->getSolverInfo().m_damping = s.damping;
  }
  if (s.iterations > 0) m_world->getSolverInfo().m_numIterations = s.iterations;
  if (s.erp >= 0) m_world->getSolverInfo().m_erp = s.erp;
  if (s.erp2 >= 0) m_world->getSolverInfo().m_erp2 = s.erp2;
  if (s.cfm >= 0) m_world->getSolverInfo().m_globalCfm = s.cfm;
}

btSoftBodyWorldInfo& World::softInfo() {
#if BN_NEXT_FEATURES
  if (m_softMt) return m_softMt->getWorldInfo();
#endif
  return m_soft->getWorldInfo();
}

void World::addSoft(btSoftBody* b) {
#if BN_NEXT_FEATURES
  if (m_softMt) { m_softMt->addSoftBody(b); return; }
#endif
  m_soft->addSoftBody(b);
}

void World::removeSoft(btSoftBody* b) {
#if BN_NEXT_FEATURES
  if (m_softMt) { m_softMt->removeSoftBody(b); return; }
#endif
  m_soft->removeSoftBody(b);
}

World::~World() {
  // The soft-body distance-field cache lives in the world info; Bullet 2.84's has no destructor
  // that frees its cells (Bullet NEXT's has), so empty it here.
#if BN_NEXT_FEATURES
  if (m_softMt) m_softMt->getWorldInfo().m_sparsesdf.Reset();
#endif
  if (m_soft) m_soft->getWorldInfo().m_sparsesdf.Reset();  // (the Blender profile has no soft world)
  for (size_t i = 0; i < m_vehicles.size(); ++i) { m_world->removeVehicle(m_vehicles[i]); delete m_vehicles[i]; }
  for (size_t i = 0; i < m_raycasters.size(); ++i) delete m_raycasters[i];
  for (size_t i = 0; i < m_tunings.size(); ++i) delete m_tunings[i];
  for (size_t i = 0; i < m_constraints.size(); ++i) { m_world->removeConstraint(m_constraints[i]); delete m_constraints[i]; }
  for (size_t i = 0; i < m_bodies.size(); ++i) {
    BodyRec& b = m_bodies[i];
    if (b.character) { m_world->removeAction(b.character); delete b.character; }
    if (b.soft) removeSoft(b.soft);
    else if (b.rb) { m_world->removeRigidBody(b.rb); delete b.rb->getMotionState(); }
    else if (b.obj) m_world->removeCollisionObject(b.obj);
    delete b.obj;
    if (!b.soft) deleteShape(b.opts.shape);
  }
  for (size_t i = 0; i < m_meshes.size(); ++i) delete m_meshes[i];
  delete m_fixed;
  delete m_world; delete m_solver; delete m_ghostCb; delete m_filter;
#if BN_NEXT_FEATURES
  delete m_solverPool;
#endif
  delete m_broadphase; delete m_dispatcher; delete m_config;
}

// Process-wide switches of Bullet NEXT, set for this world before it creates constraints and
// before every step, so that worlds with and without the features can share one program (the Lab).
void World::applyGlobals() {
#if BN_NEXT_FEATURES || BN_NEXT_LIB
  btNextSetLinkedCollisionWhileEnabled(m_linkedCollision);
  gBnMeshContactReduction = m_meshReduction;
#endif
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
  if (m_settings.profile == kBlenderRb) return addRb(o);
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
#if BN_NEXT_FEATURES
  if (featureOn(4)) btNextEnableHullClipping(o.shape);          // C1.5
  if (!dyn && featureOn(8)) btNextEnableActiveEdges(body);      // C1.6 (meshes and heightfields only)
  if (dyn && featureOn(16)) btNextConfigureCcd(body, true);     // C1.7, C1.8
#endif
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
  btSoftBody* psb = btSoftBodyHelpers::CreateFromTriMesh(softInfo(), &flat[0], &tris[0],
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
  addSoft(psb);
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
  applyGlobals();
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
  applyGlobals();
  m_world->addConstraint(c, true);
  m_constraints.push_back(c);
  return (int)m_constraints.size() - 1;
}

int World::addConeTwist(int a, int b, const btTransform& fa, const btTransform& fb) {
  btConeTwistConstraint* c = b < 0 ? new btConeTwistConstraint(*rigidOf(a), fixedBody(), fa, rigidOf(a)->getCenterOfMassTransform() * fa)
                                   : new btConeTwistConstraint(*rigidOf(a), *rigidOf(b), fa, fb);
  c->setUserConstraintType(4);
  applyGlobals();
  m_world->addConstraint(c, true);
  m_constraints.push_back(c);
  return (int)m_constraints.size() - 1;
}

int World::addGeneric6DofSpring(int a, int b, const btTransform& fa, const btTransform& fb) {
  btGeneric6DofSpringConstraint* c =
      new btGeneric6DofSpringConstraint(*rigidOf(a), *rigidOf(b), fa, fb, true);
  c->setUserConstraintType(12);
  applyGlobals();
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

void World::moveKinematic(int i, const btVector3& delta) {
  BodyRec& r = m_bodies[i];
  if (m_settings.profile != kBlenderRb) {
    btTransform t = r.obj->getWorldTransform();
    t.setOrigin(t.getOrigin() + delta);
    setKinematicTransform(i, t);
    return;
  }
  // rigidbody_create_substep_data: old = the body now, new = the keyframed object this frame
  // (here: old + delta; the actions only translate, so the rotation is kept).
  const float d[3] = {(float)delta.x(), (float)delta.y(), (float)delta.z()};
  for (size_t k = 0; k < m_targets.size(); ++k)
    if (m_targets[k].body == i) {
      for (int c = 0; c < 3; ++c) m_targets[k].to[c] += d[c];
      return;
    }
  KinematicTarget t;
  t.body = i;
  const btTransform& cur = r.rb->getWorldTransform();
  const btQuaternion q = cur.getRotation();
  for (int c = 0; c < 3; ++c) { t.from[c] = (float)cur.getOrigin()[c]; t.to[c] = t.from[c] + d[c]; }
  t.rot[0] = (float)q.getW(); t.rot[1] = (float)q.getX(); t.rot[2] = (float)q.getY(); t.rot[3] = (float)q.getZ();
  m_targets.push_back(t);
}

void World::step() {
  if (m_settings.profile == kBlenderRb) { stepRb(); return; }
  // ProceedDeltaTime(curTime, timeStep, interval) with a fixed frame rate.
  gDeactivationTime = m_settings.deactivationTime;
  gContactBreakingThreshold = m_settings.contactBreaking;
  applyGlobals();
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

void deleteShape(btCollisionShape* s) {
  if (!s) return;
  const btStridingMeshInterface* mesh = 0;
  switch (s->getShapeType()) {
    case COMPOUND_SHAPE_PROXYTYPE: {
      btCompoundShape* c = static_cast<btCompoundShape*>(s);
      for (int i = c->getNumChildShapes() - 1; i >= 0; --i) deleteShape(c->getChildShape(i));
      break;
    }
    case SCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE:
      deleteShape(static_cast<btScaledBvhTriangleMeshShape*>(s)->getChildShape());
      break;
    case TRIANGLE_MESH_SHAPE_PROXYTYPE: mesh = static_cast<btBvhTriangleMeshShape*>(s)->getMeshInterface(); break;
    case GIMPACT_SHAPE_PROXYTYPE: mesh = static_cast<btGImpactMeshShape*>(s)->getMeshInterface(); break;
    default: break;
  }
  delete s;
  delete mesh;
}

btCollisionShape* makeMesh(const std::vector<btVector3>& verts, const std::vector<int>& tris,
                           bool dynamic, const btVector3& scale, float margin) {
  std::vector<btScalar> fv;
  for (size_t i = 0; i < verts.size(); ++i) { fv.push_back(verts[i].x()); fv.push_back(verts[i].y()); fv.push_back(verts[i].z()); }
  btTriangleIndexVertexArray* arr = new OwnedMesh(fv, tris);
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


// ---------------------------------------------------------------------------------------
// Blender rigid body world (profile kBlenderRb, FL CL.7): what Blender 5.3 does with Bullet,
// rebuilt here so it runs on every Lab backend. Bodies: rigidbody_validate_sim_object and the
// RB_body_* calls it makes; constraints: rigidbody_validate_sim_constraint and RB_constraint_*;
// stepping: BKE_rigidbody_do_simulation. lab/reference/bnl_rbref.cpp drives Blender's own
// rb_bullet_api.cpp the same way; on Blender's Bullet copy both must give the same hashes (TL.5).

int World::addRb(const BodyOpts& o) {
  BodyRec rec;
  rec.opts = o;
  const bool active = (o.kind == kRigid || o.kind == kDynamic) && !o.animated;
  // RB_body_new
  btTransform trans;
  trans.setIdentity();
  trans.setOrigin(o.rbLoc);
  trans.setRotation(o.rbRot);
  btDefaultMotionState* ms = new btDefaultMotionState(trans);
  btRigidBody::btRigidBodyConstructionInfo ci(1.0f, ms, o.shape);
  btRigidBody* body = new btRigidBody(ci);
  body->setFriction(o.friction);
  body->setRestitution(o.restitution);
  body->setDamping(o.linDamp, o.angDamp);
  body->setSleepingThresholds(o.rbLinSleep, o.rbAngSleep);
  if (o.useDeactivation) body->forceActivationState(ACTIVE_TAG);
  else body->setActivationState(DISABLE_DEACTIVATION);
  if (!active || o.startDeactivated) body->setActivationState(ISLAND_SLEEPING);
  body->setLinearFactor(o.linFactor);
  body->setAngularFactor(o.angFactor);
  // RB_body_set_mass (RBO_GET_MASS: 0 for passive, keyframed or disabled bodies)
  rec.mass = active ? o.mass : 0.0f;
  btVector3 localInertia(0, 0, 0);
  if (rec.mass) body->getCollisionShape()->calculateLocalInertia(rec.mass, localInertia);
  body->setMassProps(rec.mass, localInertia);
  body->updateInertiaTensor();
  // RB_body_set_kinematic_state
  if (o.animated) body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
  else body->setCollisionFlags(body->getCollisionFlags() & ~btCollisionObject::CF_KINEMATIC_OBJECT);
  rec.obj = body;
  rec.rb = body;
  const int index = (int)m_bodies.size();
  body->setUserIndex(index);
  m_bodies.push_back(rec);
  m_world->addRigidBody(body);  // RB_dworld_add_body
#if BN_NEXT_FEATURES
  if (featureOn(4)) btNextEnableHullClipping(o.shape);
  if (!active && featureOn(8)) btNextEnableActiveEdges(body);
  if (active && featureOn(16)) btNextConfigureCcd(body, true);
#endif
  return index;
}

int World::addRbJoint(const RbJoint& j) {
  btRigidBody* b1 = m_bodies[j.a].rb;
  btRigidBody* b2 = m_bodies[j.b].rb;
  // make_constraint_transforms
  const btTransform t1 = b1->getWorldTransform().inverse() * j.frame;
  const btTransform t2 = b2->getWorldTransform().inverse() * j.frame;
  btTypedConstraint* con = 0;
  enum { kFixed = 4, kPoint, kHinge, kSlider, kPiston, kGeneric, kGenericSpring, kMotor };
  switch (j.type) {
    case kPoint: {
      const btVector3 p = j.frame.getOrigin();
      con = new btPoint2PointConstraint(*b1, *b2, b1->getWorldTransform().inverse() * p, b2->getWorldTransform().inverse() * p);
      break;
    }
    case kFixed: con = new btFixedConstraint(*b1, *b2, t1, t2); break;
    case kHinge: {
      btHingeConstraint* h = new btHingeConstraint(*b1, *b2, t1, t2);
      if (j.useLimit[5]) h->setLimit(j.lower[5], j.upper[5], 0.9f, 0.3f, 1.0f);
      else h->setLimit(0.0f, -1.0f, 0.9f, 0.3f, 1.0f);
      con = h;
      break;
    }
    case kSlider: {
      btSliderConstraint* s = new btSliderConstraint(*b1, *b2, t1, t2, true);
      if (j.useLimit[0]) { s->setLowerLinLimit(j.lower[0]); s->setUpperLinLimit(j.upper[0]); }
      else { s->setLowerLinLimit(0.0f); s->setUpperLinLimit(-1.0f); }
      con = s;
      break;
    }
    case kPiston: {
      btSliderConstraint* s = new btSliderConstraint(*b1, *b2, t1, t2, true);
      s->setUpperAngLimit(-1.0f);
      s->setLowerLinLimit(j.useLimit[0] ? j.lower[0] : 0.0f);
      s->setUpperLinLimit(j.useLimit[0] ? j.upper[0] : -1.0f);
      s->setLowerAngLimit(j.useLimit[3] ? j.lower[3] : 0.0f);
      s->setUpperAngLimit(j.useLimit[3] ? j.upper[3] : -1.0f);
      con = s;
      break;
    }
    case kGeneric: {
      btGeneric6DofConstraint* g = new btGeneric6DofConstraint(*b1, *b2, t1, t2, true);
      for (int k = 0; k < 6; ++k) g->setLimit(k, j.useLimit[k] ? j.lower[k] : 0.0f, j.useLimit[k] ? j.upper[k] : -1.0f);
      con = g;
      break;
    }
    case kGenericSpring:
      if (j.springType == 2) {
        btGeneric6DofSpring2Constraint* g = new btGeneric6DofSpring2Constraint(*b1, *b2, t1, t2);
        for (int k = 0; k < 6; ++k) {
          g->enableSpring(k, j.useSpring[k]);
          g->setStiffness(k, j.stiffness[k]);
          g->setDamping(k, j.damping[k]);
        }
        g->setEquilibriumPoint();
        for (int k = 0; k < 6; ++k) g->setLimit(k, j.useLimit[k] ? j.lower[k] : 0.0f, j.useLimit[k] ? j.upper[k] : -1.0f);
        con = g;
      } else {
        btGeneric6DofSpringConstraint* g = new btGeneric6DofSpringConstraint(*b1, *b2, t1, t2, true);
        for (int k = 0; k < 6; ++k) {
          g->enableSpring(k, j.useSpring[k]);
          g->setStiffness(k, j.stiffness[k]);
          const float d = j.damping[k] > 1.0f ? 0.0f : 1.0f - j.damping[k];  // Blender inverts the range
          g->setDamping(k, d);
        }
        g->setEquilibriumPoint();
        for (int k = 0; k < 6; ++k) g->setLimit(k, j.useLimit[k] ? j.lower[k] : 0.0f, j.useLimit[k] ? j.upper[k] : -1.0f);
        con = g;
      }
      break;
    case kMotor: {
      btGeneric6DofConstraint* g = new btGeneric6DofConstraint(*b1, *b2, t1, t2, true);
      for (int k = 0; k < 6; ++k) g->setLimit(k, 0.0f, -1.0f);
      g->getTranslationalLimitMotor()->m_upperLimit.setValue(-1.0f, -1.0f, -1.0f);
      g->getTranslationalLimitMotor()->m_enableMotor[0] = j.useMotorLin;
      g->getRotationalLimitMotor(0)->m_enableMotor = j.useMotorAng;
      g->getTranslationalLimitMotor()->m_maxMotorForce.setX(j.motorLinMaxImpulse);
      g->getRotationalLimitMotor(0)->m_maxMotorForce = j.motorAngMaxImpulse;
      g->getTranslationalLimitMotor()->m_targetVelocity.setX(j.motorLinVelocity);
      g->getRotationalLimitMotor(0)->m_targetVelocity = j.motorAngVelocity;
      con = g;
      break;
    }
  }
  if (!con) return -1;
  con->setEnabled(j.enabled);
  con->setBreakingImpulseThreshold(j.breakingThreshold >= 0 ? j.breakingThreshold : FLT_MAX);
  con->setOverrideNumSolverIterations(j.iterations);
  con->setUserConstraintType(100 + j.type);
  applyGlobals();
  m_world->addConstraint(con, j.disableCollisions);
  m_constraints.push_back(con);
  return (int)m_constraints.size() - 1;
}

void World::stepRb() {
  // BKE_rigidbody_do_simulation for one frame: world settings, then the substeps with the
  // keyframed bodies interpolated towards this frame's targets.
  // Blender never sets these two Bullet globals, so they keep Bullet's defaults. A BGE-profile
  // world that ran before in the same program (another scene, another engine) may have changed
  // them; put the defaults back (found by the Lab: rb scenes depended on the scene before).
  gContactBreakingThreshold = btScalar(0.02);
  gDeactivationTime = btScalar(2.);
  const float timestep = 1.0f / m_settings.ticrate * 1.0f * m_settings.timeScale;
  const float substep = timestep / float(m_settings.substeps);
  const float interpStep = 1.0f / float(m_settings.substeps);
  float interp = interpStep;
  m_world->setGravity(btVector3(0, 0, m_settings.gravityZ));      // rigidbody_update_sim_world
  if (m_settings.iterations > 0) m_world->getSolverInfo().m_numIterations = m_settings.iterations;
  m_world->getSolverInfo().m_splitImpulse = m_settings.splitImpulse;
  applyGlobals();
  for (int i = 0; i < m_settings.substeps; ++i) {
    for (size_t k = 0; k < m_targets.size(); ++k) {  // rigidbody_update_kinematic_obj_substep
      const KinematicTarget& t = m_targets[k];
      float loc[3], rot[4];
      bnrb::interpV3(loc, t.from, t.to, interp);
      bnrb::interpQt(rot, t.rot, t.rot, interp);
      btRigidBody* body = m_bodies[t.body].rb;
      body->setActivationState(ACTIVE_TAG);                          // RB_body_activate
      btTransform trans;                                            // RB_body_set_loc_rot
      trans.setIdentity();
      trans.setOrigin(btVector3(loc[0], loc[1], loc[2]));
      trans.setRotation(btQuaternion(rot[1], rot[2], rot[3], rot[0]));
      body->getMotionState()->setWorldTransform(trans);
    }
    m_world->stepSimulation(substep, 0, substep);                   // RB_dworld_step_simulation
    interp += interpStep;
  }
  m_targets.clear();
  m_events.clear();  // Blender's rigid body world has no collision callbacks
  ++m_frame;
}

BN_NAMESPACE_END  // namespace bn
