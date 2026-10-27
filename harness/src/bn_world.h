// Bullet NEXT golden harness - BGE 2.79b world factory (commits C0.4, C0.5).
// Rebuilds the physics setup of source/gameengine/Physics/Bullet/CcdPhysicsEnvironment.cpp
// (Blender 2.79b) without the rest of the game engine.
#ifndef BN_WORLD_H
#define BN_WORLD_H

#include "bn_common.h"

BN_NAMESPACE_BEGIN

// BGE CcdConstructionInfo::CollisionFilterGroups
enum FilterGroup {
  kDefaultFilter = 1, kStaticFilter = 2, kKinematicFilter = 4, kDebrisFilter = 8,
  kSensorFilter = 16, kCharacterFilter = 32,
  kAllFilter = 1 | 2 | 4 | 8 | 16 | 32
};

// Host profiles (FL, CL.7): how the host program drives Bullet.
enum Profile { kBge279 = 0, kBlenderRb = 1 };

// BGE physics types (Object > Physics > Physics Type)
enum BodyKind { kStatic, kDynamic, kRigid, kSensor, kCharacter, kSoft };

struct Settings {
  float ticrate = 60.0f;            // GameData.ticrate
  int substeps = 1;                 // GameData.physubstep / setNumTimeSubSteps
  float gravityZ = -9.81f;          // GameData.gravity (Z only)
  float contactBreaking = 0.02f;    // setContactBreakingTreshold (global)
  float deactivationTime = 2.0f;    // setDeactivationTime (global)
  float linSleep = 0.8f;            // setDeactivationLinearTreshold
  float angSleep = 1.0f;            // setDeactivationAngularTreshold
  bool setSolverParams = false;     // setSorConstant / setSolverTau / setSolverDamping
  float sor = 1.0f, tau = 0.6f, damping = 1.0f;
  int threads = -1;                 // Bullet NEXT threaded world: -1 = BN_THREADS variable, 0 = off
  // Scene-file solver settings (FL, CL.3); negative or empty = the engine's own default.
  int iterations = -1;              // setNumIterations
  int nncg = 0;                     // Bullet NEXT: 1 = NNCG solver (setSolverType), single-threaded world
  float erp = -1, erp2 = -1, cfm = -1;  // joint ERP, contact ERP, CFM (UPBGE names)
  int features = -1;                // Bullet NEXT features (bits as BN_FEATURES); -1 = BN_FEATURES or all
  // Soft Step solver (Bullet NEXT, F3c); -1 = the solver's default.
  int softStep = 0;
  int solverSubsteps = -1, solverIterations = -1, relaxIterations = -1, manifoldIterations = -1, jointIterations = -1;
  float jointWarmStart = -1, jointHertz = -1, jointDampingRatio = -1;
  float substepRate = -1;
  float contactHertz = -1, contactDampingRatio = -1, contactPushSpeed = -1;
  int profile = kBge279;
  bool splitImpulse = false;        // Blender rigid body world settings
  float timeScale = 1.0f;
};

struct BodyOpts {
  BodyKind kind = kRigid;
  btCollisionShape* shape = nullptr;   // ownership passes to the World
  btTransform xform = btTransform::getIdentity();
  float mass = 1.0f;
  float friction = 0.5f;               // material friction
  float restitution = 0.0f;            // material elasticity ("reflect")
  float linDamp = 0.04f, angDamp = 0.1f;
  float inertia = 0.4f;                // Blender "form factor"; BGE uses inertia/0.4
  btVector3 linFactor = btVector3(1, 1, 1), angFactor = btVector3(1, 1, 1);  // axis locks
  bool ghost = false;                  // Ghost flag: no contact response
  unsigned short userGroup = 0xffff, userMask = 0xffff;  // col_group / col_mask
  float clampVelMin = 0, clampVelMax = 0, clampAngMin = 0, clampAngMax = 0;
  bool reportCollisions = false;       // a Collision sensor / callback is registered
  std::string name;
  // Blender rigid body world (profile kBlenderRb)
  bool animated = false, useDeactivation = false, startDeactivated = false;
  float rbLinSleep = 0.4f, rbAngSleep = 0.5f;
  int colGroups = 1;
  btVector3 rbLoc = btVector3(0, 0, 0);       // RB_body_new: location and rotation as given
  btQuaternion rbRot = btQuaternion(0, 0, 0, 1);
};

// A Blender rigid body constraint (profile kBlenderRb), in world space between two bodies.
struct RbJoint {
  int type = 0;                        // bnl::ConstraintDesc::Type, kRbFixed and up
  int a = 0, b = 0;
  btTransform frame = btTransform::getIdentity();
  bool enabled = true, disableCollisions = true;
  float breakingThreshold = -1;
  int iterations = -1;
  bool useLimit[6] = {false, false, false, false, false, false};
  float lower[6] = {0, 0, 0, 0, 0, 0}, upper[6] = {0, 0, 0, 0, 0, 0};
  bool useSpring[6] = {false, false, false, false, false, false};
  float stiffness[6] = {0, 0, 0, 0, 0, 0}, damping[6] = {0, 0, 0, 0, 0, 0};
  int springType = 2;
  bool useMotorLin = false, useMotorAng = false;
  float motorLinVelocity = 0, motorAngVelocity = 0, motorLinMaxImpulse = 0, motorAngMaxImpulse = 0;
};

struct ContactEvent {
  int a, b;                 // body indices (a = object reported on)
  btVector3 p1, p2, normal; // BGE PHY_CollData (point 0 of the manifold)
};

struct BodyRec {
  BodyOpts opts;
  btCollisionObject* obj = nullptr;
  btRigidBody* rb = nullptr;
  btSoftBody* soft = nullptr;
  btKinematicCharacterController* character = nullptr;
  short group = 0, mask = 0;
  float mass = 0;
};

class World {
 public:
  explicit World(const Settings& s);
  ~World();

  int add(const BodyOpts& o);
  int addSoftFromTriMesh(const std::vector<btVector3>& verts, const std::vector<int>& tris,
                         const BodyOpts& o, bool bending, bool shapeMatching);
  int addCharacter(const BodyOpts& o, float stepHeight, float jumpSpeed, float fallSpeed);

  // Constraints, following CcdPhysicsEnvironment::CreateConstraint. bodyB = -1 attaches to
  // BGE's static fixed object. Frames are given directly in each body's local space.
  int addPointToPoint(int a, int b, const btVector3& pivotA, const btVector3& pivotB);
  int addHinge(int a, int b, const btTransform& frameA, const btTransform& frameB, bool angularOnly);
  int addConeTwist(int a, int b, const btTransform& frameA, const btTransform& frameB);
  int addGeneric6DofSpring(int a, int b, const btTransform& frameA, const btTransform& frameB);
  // KX_ConstraintWrapper.setParam(index, value0, value1) semantics for 6DOF / cone / hinge
  void setParam(int constraint, int index, float v0, float v1);
  // Blender rigid body constraints, as BKE_rigidbody / rb_bullet_api create them.
  int addRbJoint(const RbJoint& j);

  int addVehicle(int chassis);
  void addWheel(int vehicle, const btVector3& pos, const btVector3& down, const btVector3& axle,
                float restLen, float radius, bool steering);
  btRaycastVehicle* vehicle(int i) { return m_vehicles[i]; }

  void step();  // one logic frame: CcdPhysicsEnvironment::ProceedDeltaTime
  void suspendDynamics(int body);  // switch to static/kinematic as BGE does
  void setKinematicTransform(int body, const btTransform& t);
  // Scripted kinematic motion: BGE moves the body now; Blender's rigid body world moves it to
  // the target over the next frame's substeps (rigidbody_update_kinematic_obj_substep).
  void moveKinematic(int body, const btVector3& delta);
  // Stage 0, CS.3: moves the whole world by d (btNextShiftWorld on Bullet NEXT; the same steps
  // by hand on the older copies), soft bodies included
  void shiftOrigin(const btVector3& d);

  const std::vector<BodyRec>& bodies() const { return m_bodies; }
  BodyRec& body(int i) { return m_bodies[i]; }
  const std::vector<ContactEvent>& events() const { return m_events; }
  const std::vector<btTypedConstraint*>& constraints() const { return m_constraints; }
  btDiscreteDynamicsWorld* dyn() { return m_world; }
  const Settings& settings() const { return m_settings; }
  int frame() const { return m_frame; }
  float gravityZ() const { return m_settings.gravityZ; }

  bool collides(int a, int b) const;  // KX_GameObject::CheckCollision (user groups)
  void tick(btScalar dt);             // per-substep velocity clamping
  void applyGlobals();
  bool featureOn(int bit) const;    // Bullet NEXT feature switch (scene file, else BN_FEATURES)
#if BN_NEXT_LIB
  // Soft Step solvers of this world (one, or one per thread); owned by the world or the pool.
  const std::vector<btSoftStepConstraintSolver*>& softStepSolvers() const { return m_softStepSolvers; }
#endif

 private:
  btRigidBody* rigidOf(int i);
  btRigidBody& fixedBody();

  Settings m_settings;
  btDefaultCollisionConfiguration* m_config = nullptr;
  struct KinematicTarget { int body; float from[3], to[3], rot[4]; };  // floats, as Blender keeps them
  std::vector<KinematicTarget> m_targets;  // Blender profile: this frame's kinematic moves
  int addRb(const BodyOpts& o);
  void stepRb();
  btCollisionDispatcher* m_dispatcher = nullptr;
  btDbvtBroadphase* m_broadphase = nullptr;
  btOverlapFilterCallback* m_filter = nullptr;
  btGhostPairCallback* m_ghostCb = nullptr;
  btConstraintSolver* m_solver = nullptr;
#if BN_NEXT_LIB
  btSoftStepConstraintSolver* newSoftStepSolver();
  std::vector<btSoftStepConstraintSolver*> m_softStepSolvers;
#endif
  btDiscreteDynamicsWorld* m_world = nullptr;
  btSoftRigidDynamicsWorld* m_soft = nullptr;
#if BN_NEXT_FEATURES
  btSoftRigidDynamicsWorldMt* m_softMt = nullptr;
  btConstraintSolverPoolMt* m_solverPool = nullptr;
#endif
  btSoftBodyWorldInfo& softInfo();
  void addSoft(btSoftBody* b);
  void removeSoft(btSoftBody* b);
  btRigidBody* m_fixed = nullptr;
  bool m_linkedCollision = false, m_meshReduction = false;  // Bullet NEXT globals, set per step
  std::vector<BodyRec> m_bodies;
  std::vector<btTypedConstraint*> m_constraints;
  std::vector<btRaycastVehicle*> m_vehicles;
  std::vector<btVehicleRaycaster*> m_raycasters;
  std::vector<btRaycastVehicle::btVehicleTuning*> m_tunings;
  std::vector<btTriangleIndexVertexArray*> m_meshes;
  std::vector<ContactEvent> m_events;
  int m_frame = 0;
};

// Shape helpers (CcdShapeConstructionInfo::CreateBulletShape)
// A triangle mesh that owns its vertex and index memory; it goes with its shape (deleteShape).
struct MeshMemory {
  std::vector<btScalar> v;
  std::vector<int> t;
  MeshMemory(const std::vector<btScalar>& vv, const std::vector<int>& tt) : v(vv), t(tt) {}
};
class OwnedMesh : private MeshMemory, public btTriangleIndexVertexArray {
 public:
  OwnedMesh(const std::vector<btScalar>& v, const std::vector<int>& t)
      : MeshMemory(v, t),
        btTriangleIndexVertexArray((int)MeshMemory::t.size() / 3, &MeshMemory::t[0], 3 * sizeof(int),
                                   (int)MeshMemory::v.size() / 3, &MeshMemory::v[0], 3 * sizeof(btScalar)) {}
};
// A heightfield that keeps its own heights (Stage 0, CS.2): samplesX by samplesY floats, Z up,
// `spacing` metres apart. Bullet centres it on its bounding box: makeHeightfield returns the
// height of that centre, which the body's origin must be raised by.
struct HeightMemory {
  std::vector<float> h;
  explicit HeightMemory(const std::vector<float>& hh) : h(hh) {}
};
class OwnedHeightfield : private HeightMemory, public btHeightfieldTerrainShape {
 public:
  OwnedHeightfield(int nx, int ny, const std::vector<float>& heights, float minH, float maxH)
      : HeightMemory(heights), btHeightfieldTerrainShape(nx, ny, &HeightMemory::h[0], 1.0f, minH, maxH, 2, PHY_FLOAT, false) {}
};
btCollisionShape* makeHeightfield(int nx, int ny, float spacing, const std::vector<float>& heights, float margin, float* centreZ);
// Deletes a shape with everything the Lab made for it: compound children, the BVH inside the
// scaled wrapper, mesh memory.
void deleteShape(btCollisionShape* s);

btCollisionShape* makeBox(const btVector3& half, float margin = 0.04f);
btCollisionShape* makeSphere(float r, float margin = 0.04f);
btCollisionShape* makeCylinderZ(const btVector3& half, float margin = 0.04f);
btCollisionShape* makeConeZ(float r, float h, float margin = 0.04f);
btCollisionShape* makeCapsuleZ(float r, float h, float margin = 0.04f);
btCollisionShape* makeHull(const std::vector<btVector3>& pts, float margin = 0.04f);
// Static triangle mesh: BVH mesh wrapped in a scaled shape; dynamic: GImpact.
btCollisionShape* makeMesh(const std::vector<btVector3>& verts, const std::vector<int>& tris,
                           bool dynamic, const btVector3& scale, float margin = 0.04f);
void gridMesh(int nx, int ny, float size, std::vector<btVector3>& v, std::vector<int>& t);
void boxMesh(const btVector3& half, std::vector<btVector3>& v, std::vector<int>& t);

BN_NAMESPACE_END  // namespace bn

#endif
