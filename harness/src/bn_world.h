// Bullet NEXT golden harness - BGE 2.79b world factory (commits C0.4, C0.5).
// Rebuilds the physics setup of source/gameengine/Physics/Bullet/CcdPhysicsEnvironment.cpp
// (Blender 2.79b) without the rest of the game engine.
#ifndef BN_WORLD_H
#define BN_WORLD_H

#include "bn_common.h"

namespace bn {

// BGE CcdConstructionInfo::CollisionFilterGroups
enum FilterGroup {
  kDefaultFilter = 1, kStaticFilter = 2, kKinematicFilter = 4, kDebrisFilter = 8,
  kSensorFilter = 16, kCharacterFilter = 32,
  kAllFilter = 1 | 2 | 4 | 8 | 16 | 32
};

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

  int addVehicle(int chassis);
  void addWheel(int vehicle, const btVector3& pos, const btVector3& down, const btVector3& axle,
                float restLen, float radius, bool steering);
  btRaycastVehicle* vehicle(int i) { return m_vehicles[i]; }

  void step();  // one logic frame: CcdPhysicsEnvironment::ProceedDeltaTime
  void suspendDynamics(int body);  // switch to static/kinematic as BGE does
  void setKinematicTransform(int body, const btTransform& t);

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

 private:
  btRigidBody* rigidOf(int i);
  btRigidBody& fixedBody();

  Settings m_settings;
  btSoftBodyRigidBodyCollisionConfiguration* m_config = nullptr;
  btCollisionDispatcher* m_dispatcher = nullptr;
  btDbvtBroadphase* m_broadphase = nullptr;
  btOverlapFilterCallback* m_filter = nullptr;
  btGhostPairCallback* m_ghostCb = nullptr;
  btConstraintSolver* m_solver = nullptr;
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

}  // namespace bn

#endif
