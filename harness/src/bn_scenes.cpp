// Bullet NEXT golden harness - scene library (commit C0.8).
// All scenes are made for this project (no outside files). Coverage of the 12-point
// compatibility surface:
//   1 stepping  2 Python setters  3 body types  4 shapes/margins  5 filtering  6 contacts
//   7 constraints  8 vehicle  9 character  10 soft body  11 material/inertia/damping/sleep
//   12 numerics (every scene)
#include "bn_scenes.h"

#include <cmath>

namespace bn {

namespace {

const float kPi = 3.14159265358979f;

btTransform at(float x, float y, float z) {
  btTransform t = btTransform::getIdentity();
  t.setOrigin(btVector3(x, y, z));
  return t;
}

btTransform atRot(float x, float y, float z, const btVector3& axis, float angle) {
  btTransform t(btQuaternion(axis, angle), btVector3(x, y, z));
  return t;
}

int ground(World& w, float half = 20.0f) {
  BodyOpts o;
  o.kind = kStatic;
  o.shape = makeBox(btVector3(half, half, 0.5f));
  o.xform = at(0, 0, -0.5f);
  o.name = "ground";
  return w.add(o);
}

int meshGround(World& w, int cells, float half) {
  std::vector<btVector3> v; std::vector<int> t;
  gridMesh(cells, cells, half, v, t);
  BodyOpts o;
  o.kind = kStatic;
  o.shape = makeMesh(v, t, false, btVector3(1, 1, 1));
  o.name = "mesh_ground";
  return w.add(o);
}

int box(World& w, const btTransform& t, const btVector3& half, float mass, BodyKind kind = kRigid) {
  BodyOpts o;
  o.kind = kind;
  o.shape = makeBox(half);
  o.xform = t;
  o.mass = mass;
  return w.add(o);
}

// --- 1. stack_10: ten boxes in a column -------------------------------------------------
void buildStack(World& w) {
  ground(w);
  for (int i = 0; i < 10; ++i) box(w, at(0, 0, 0.5f + i * 1.0f), btVector3(0.5f, 0.5f, 0.5f), 1.0f);
}

// --- 2. pyramid_10: 55 boxes ---------------------------------------------------------------
void buildPyramid(World& w) {
  ground(w);
  const int rows = 10;
  for (int r = 0; r < rows; ++r)
    for (int i = 0; i < rows - r; ++i)
      box(w, at(-0.5f * (rows - r - 1) + i * 1.0f, 0, 0.5f + r * 1.0f), btVector3(0.48f, 0.48f, 0.48f), 1.0f);
}

// --- 3. shape_zoo: every BGE shape on a static BVH mesh -----------------------------------
void buildShapeZoo(World& w) {
  meshGround(w, 16, 12.0f);
  BodyOpts o;
  o.kind = kRigid;
  o.mass = 1.0f;
  o.friction = 0.6f;
  o.restitution = 0.2f;
  o.shape = makeBox(btVector3(0.5f, 0.5f, 0.5f)); o.xform = atRot(-6, 0, 3, btVector3(1, 1, 0).normalized(), 0.4f); w.add(o);
  o.shape = makeSphere(0.5f); o.xform = at(-4, 0, 3); w.add(o);
  o.shape = makeCylinderZ(btVector3(0.5f, 0.5f, 0.7f)); o.xform = atRot(-2, 0, 3, btVector3(1, 0, 0), 1.2f); w.add(o);
  o.shape = makeConeZ(0.5f, 1.2f); o.xform = atRot(0, 0, 3, btVector3(0, 1, 0), 0.7f); w.add(o);
  o.shape = makeCapsuleZ(0.4f, 1.0f); o.xform = atRot(2, 0, 3, btVector3(1, 0, 0), kPi / 2); w.add(o);
  std::vector<btVector3> hull;
  for (int i = 0; i < 12; ++i) {
    float a = i * 2 * kPi / 12;
    hull.push_back(btVector3(0.6f * std::cos(a), 0.6f * std::sin(a), i % 2 ? 0.5f : -0.5f));
  }
  o.shape = makeHull(hull); o.xform = atRot(4, 0, 3, btVector3(0, 1, 1).normalized(), 0.3f); o.inertia = 0.6f; w.add(o);
  o.inertia = 0.4f;
  std::vector<btVector3> mv; std::vector<int> mt;
  boxMesh(btVector3(0.6f, 0.4f, 0.3f), mv, mt);
  o.shape = makeMesh(mv, mt, true, btVector3(1, 1, 1)); o.xform = atRot(6, 0, 3, btVector3(0, 0, 1), 0.5f); w.add(o);
  btCompoundShape* comp = new btCompoundShape();
  comp->addChildShape(at(0, 0, 0), makeBox(btVector3(0.6f, 0.15f, 0.15f)));
  comp->addChildShape(at(0, 0, 0.45f), makeSphere(0.3f));
  o.shape = comp; o.xform = atRot(0, 3, 3, btVector3(1, 0, 0), 0.3f); w.add(o);
}

// --- 4. joints_six: every BGE rigid-body joint type ---------------------------------------
void buildJoints(World& w) {
  ground(w);
  // Point-to-point chain hanging from the fixed object.
  int prev = -1;
  for (int i = 0; i < 5; ++i) {
    int b = box(w, at(-6, 0, 9.0f - i * 1.0f), btVector3(0.1f, 0.1f, 0.4f), 1.0f);
    if (prev < 0) w.addPointToPoint(b, -1, btVector3(0, 0, 0.5f), btVector3(-6, 0, 9.5f));
    else w.addPointToPoint(prev, b, btVector3(0, 0, -0.5f), btVector3(0, 0, 0.5f));
    prev = b;
  }
  if (prev >= 0) w.body(prev).rb->setLinearVelocity(btVector3(3, 0, 0));
  // Line hinge (door) with limit.
  int post = box(w, at(-2, 0, 2), btVector3(0.1f, 0.1f, 2), 0, kStatic);
  int door = box(w, at(-1.2f, 0, 2), btVector3(0.7f, 0.05f, 1.5f), 2.0f);
  btTransform fa = at(0.1f, 0, 0), fb = at(-0.8f, 0, 0);
  fa.setRotation(btQuaternion(btVector3(1, 0, 0), kPi / 2)); fb.setRotation(fa.getRotation());
  int h = w.addHinge(post, door, fa, fb, false);
  w.setParam(h, 3, -1.0f, 1.0f);
  w.body(door).rb->setAngularVelocity(btVector3(0, 0, 4));
  // Angular-only hinge (spinning wheel).
  int wheel = box(w, at(1, 0, 3), btVector3(0.6f, 0.6f, 0.1f), 1.0f);
  int hw = w.addHinge(wheel, -1, at(0, 0, 0), at(1, 0, 3), true);
  w.setParam(hw, 3, 1.0f, -1.0f);  // min > max: free
  w.body(wheel).rb->setAngularVelocity(btVector3(0, 0, 5));
  // Cone twist (shoulder).
  int arm = box(w, at(4, 0, 3), btVector3(0.15f, 0.15f, 0.8f), 1.0f);
  int ct = w.addConeTwist(arm, -1, at(0, 0, 0.8f), at(4, 0, 3.8f));
  w.setParam(ct, 3, 0, 0.5f); w.setParam(ct, 4, 0, 0.7f); w.setParam(ct, 5, 0, -1.0f);
  w.body(arm).rb->setAngularVelocity(btVector3(2, 1, 0));
  // Generic 6DOF with limits, motors and springs (parameter indices 0-17).
  int baseB = box(w, at(7, 0, 0.5f), btVector3(0.5f, 0.5f, 0.5f), 0, kStatic);
  int slider = box(w, at(7, 0, 2), btVector3(0.3f, 0.3f, 0.3f), 1.0f);
  int g = w.addGeneric6DofSpring(baseB, slider, at(0, 0, 1.5f), at(0, 0, 0));
  w.setParam(g, 0, -1.0f, 1.0f);   // X limited
  w.setParam(g, 1, 0.0f, 0.0f);    // Y locked
  w.setParam(g, 2, -0.5f, 0.5f);   // Z limited
  w.setParam(g, 3, 1.0f, -1.0f);   // rot X free
  w.setParam(g, 4, 0.0f, 0.0f);
  w.setParam(g, 5, 0.0f, 0.0f);
  w.setParam(g, 6, 0.8f, 5.0f);    // X motor
  w.setParam(g, 9, 2.0f, 2.0f);    // rot X motor
  w.setParam(g, 14, 40.0f, 0.5f);  // Z spring
}

// --- 5. vehicle: raycast car driven by a script -----------------------------------------
void buildVehicle(World& w) {
  meshGround(w, 20, 40.0f);
  int ramp = box(w, atRot(0, 14, 0.2f, btVector3(1, 0, 0), 0.15f), btVector3(3, 3, 0.4f), 0, kStatic);
  (void)ramp;
  BodyOpts c;
  c.kind = kRigid;
  c.mass = 800.0f;
  c.shape = makeBox(btVector3(0.9f, 2.0f, 0.4f));
  c.xform = at(0, 0, 1.2f);
  c.linDamp = 0.04f; c.angDamp = 0.1f;
  int chassis = w.add(c);
  w.body(chassis).rb->setActivationState(DISABLE_DEACTIVATION);
  int v = w.addVehicle(chassis);
  const btVector3 down(0, 0, -1), axle(-1, 0, 0);
  w.addWheel(v, btVector3(0.9f, 1.4f, 0), down, axle, 0.5f, 0.4f, true);
  w.addWheel(v, btVector3(-0.9f, 1.4f, 0), down, axle, 0.5f, 0.4f, true);
  w.addWheel(v, btVector3(0.9f, -1.4f, 0), down, axle, 0.5f, 0.4f, false);
  w.addWheel(v, btVector3(-0.9f, -1.4f, 0), down, axle, 0.5f, 0.4f, false);
  btRaycastVehicle* rv = w.vehicle(v);
  for (int i = 0; i < 4; ++i) {  // typical BGE script values
    btWheelInfo& wi = rv->getWheelInfo(i);
    wi.m_suspensionStiffness = 20.0f;
    wi.m_wheelsDampingCompression = 2.0f;
    wi.m_wheelsDampingRelaxation = 3.0f;
    wi.m_frictionSlip = 2.0f;
    wi.m_rollInfluence = 0.1f;
  }
}
void driveVehicle(World& w, int f) {
  btRaycastVehicle* rv = w.vehicle(0);
  const float engine = f < 240 ? 1500.0f : 0.0f;
  const float steer = (f > 120 && f < 300) ? 0.3f : 0.0f;
  const float brake = f > 360 ? 40.0f : 0.0f;
  for (int i = 0; i < 4; ++i) {
    rv->applyEngineForce(i >= 2 ? engine : 0.0f, i);
    rv->setBrake(brake, i);
    if (i < 2) rv->setSteeringValue(steer, i);
  }
}

// --- 6. character: walking up steps, jumping ---------------------------------------------
void buildCharacter(World& w) {
  ground(w);
  for (int s = 0; s < 6; ++s) box(w, at(0, 3.0f + s * 0.6f, 0.1f + s * 0.2f), btVector3(2, 0.3f, 0.1f + s * 0.2f), 0, kStatic);
  BodyOpts o;
  o.shape = makeCapsuleZ(0.4f, 1.0f);
  o.xform = at(0, 0, 1.0f);
  w.addCharacter(o, 0.35f, 6.0f, 55.0f);
}
void driveCharacter(World& w, int f) {
  btKinematicCharacterController* ch = 0;
  for (size_t i = 0; i < w.bodies().size(); ++i) if (w.bodies()[i].character) ch = w.bodies()[i].character;
  if (!ch) return;
  ch->setWalkDirection(f < 300 ? btVector3(0, 0.05f, 0) : btVector3(0.03f, -0.02f, 0));
  if (f == 330 || f == 400) ch->jump();
}

// --- 7. soft_body: soft cube and cloth-like sheet -----------------------------------------
void buildSoft(World& w) {
  ground(w);
  std::vector<btVector3> v; std::vector<int> t;
  gridMesh(6, 6, 1.0f, v, t);  // 2 x 2 m sheet, 72 triangles
  BodyOpts o;
  o.mass = 1.0f; o.friction = 0.5f;
  o.xform = atRot(0, 0, 3, btVector3(1, 0, 0), 0.4f);
  w.addSoftFromTriMesh(v, t, o, true, false);
  boxMesh(btVector3(0.5f, 0.5f, 0.5f), v, t);
  o.xform = at(3, 0, 2);
  w.addSoftFromTriMesh(v, t, o, false, true);
  box(w, at(-3, 0, 1), btVector3(0.5f, 0.5f, 0.5f), 1.0f);
}

// --- 8. sensors_filters: sensors, ghosts, user groups, callbacks --------------------------
void buildSensors(World& w) {
  ground(w);
  BodyOpts s;
  s.kind = kSensor; s.shape = makeBox(btVector3(1.5f, 1.5f, 1.5f)); s.xform = at(0, 0, 1.5f);
  s.reportCollisions = true; s.name = "sensor";
  w.add(s);
  int falling = box(w, at(0, 0, 6), btVector3(0.4f, 0.4f, 0.4f), 1.0f);
  w.body(falling).opts.reportCollisions = true;
  BodyOpts gh;
  gh.kind = kRigid; gh.ghost = true; gh.shape = makeSphere(0.5f); gh.xform = at(3, 0, 4); gh.reportCollisions = true;
  w.add(gh);  // ghost falls through the ground but still reports contacts
  // User collision groups: A (group 1) ignores B (group 2) but hits C.
  BodyOpts a;
  a.kind = kRigid; a.shape = makeBox(btVector3(0.4f, 0.4f, 0.4f)); a.xform = at(-3, 0, 6);
  a.userGroup = 1; a.userMask = 0xffff ^ 2; a.reportCollisions = true;
  w.add(a);
  BodyOpts b = a;
  b.shape = makeBox(btVector3(1, 1, 0.2f)); b.kind = kStatic; b.xform = at(-3, 0, 3); b.userGroup = 2; b.userMask = 0xffff;
  b.reportCollisions = false;
  w.add(b);
  BodyOpts c = b;
  c.shape = makeBox(btVector3(1, 1, 0.2f)); c.xform = at(-3, 0, 1); c.userGroup = 4;
  w.add(c);
}

// --- 9. projectile: fast sphere at a thin wall (no CCD in 2.79b) --------------------------
void buildProjectile(World& w) {
  ground(w);
  box(w, at(10, 0, 2), btVector3(0.025f, 3, 2), 0, kStatic);
  for (int i = 0; i < 5; ++i) {
    BodyOpts o;
    o.kind = kRigid; o.mass = 0.1f; o.shape = makeSphere(0.1f); o.xform = at(0, -2.0f + i, 2);
    int b = w.add(o);
    w.body(b).rb->setLinearVelocity(btVector3(60.0f + 60.0f * i, 0, 0));
  }
}

// --- 10. mesh_slide: box sliding across internal triangle edges ---------------------------
void buildMeshSlide(World& w) {
  meshGround(w, 40, 20.0f);
  BodyOpts o;
  o.kind = kRigid; o.mass = 1.0f; o.friction = 0.1f; o.shape = makeBox(btVector3(0.3f, 0.3f, 0.3f));
  o.xform = at(-15, 0.37f, 0.31f);
  int b = w.add(o);
  w.body(b).rb->setLinearVelocity(btVector3(8, 0, 0));
  o.shape = makeSphere(0.3f); o.xform = at(-15, -3.13f, 0.31f);
  int s = w.add(o);
  w.body(s).rb->setLinearVelocity(btVector3(8, 0, 0));
}

// --- 11. body_types: Dynamic (no rotation), axis locks, clamps, damping -------------------
void buildBodyTypes(World& w) {
  ground(w);
  BodyOpts o;
  o.kind = kDynamic; o.shape = makeBox(btVector3(0.5f, 0.5f, 0.5f)); o.xform = atRot(-4, 0, 3, btVector3(1, 0, 0), 0.6f);
  int d = w.add(o);
  w.body(d).rb->setAngularVelocity(btVector3(5, 0, 0));
  o.kind = kRigid; o.shape = makeBox(btVector3(0.5f, 0.5f, 0.5f)); o.xform = atRot(-1, 0, 3, btVector3(1, 1, 0).normalized(), 0.6f);
  o.linFactor = btVector3(1, 0, 1); o.angFactor = btVector3(0, 0, 1);
  int l = w.add(o);
  w.body(l).rb->setLinearVelocity(btVector3(1, 3, 0));
  o.linFactor = btVector3(1, 1, 1); o.angFactor = btVector3(1, 1, 1);
  o.shape = makeBox(btVector3(0.5f, 0.5f, 0.5f)); o.xform = at(2, 0, 1); o.clampVelMax = 2.0f; o.clampAngMax = 1.0f;
  int c = w.add(o);
  w.body(c).rb->setLinearVelocity(btVector3(10, 0, 5));
  w.body(c).rb->setAngularVelocity(btVector3(0, 0, 10));
  o.shape = makeBox(btVector3(0.5f, 0.5f, 0.5f)); o.clampVelMax = 0; o.clampAngMax = 0; o.xform = at(5, 0, 1); o.linDamp = 0.6f; o.angDamp = 0.6f;
  int dm = w.add(o);
  w.body(dm).rb->setLinearVelocity(btVector3(0, 6, 4));
}

// --- 12. kinematic_switch: suspendDynamics and scripted motion ----------------------------
void buildKinematic(World& w) {
  ground(w);
  box(w, at(0, 0, 4), btVector3(1.5f, 1.5f, 0.2f), 5.0f);
  for (int i = 0; i < 3; ++i) box(w, at(-0.5f + i * 0.5f, 0, 5 + i), btVector3(0.2f, 0.2f, 0.2f), 0.5f);
}
void driveKinematic(World& w, int f) {
  if (f == 60) w.suspendDynamics(1);
  if (f > 60) {
    btTransform t = w.body(1).obj->getWorldTransform();
    t.setOrigin(t.getOrigin() + btVector3(0.02f, 0, f < 150 ? 0.01f : -0.01f));
    w.setKinematicTransform(1, t);
  }
}

// --- 13. settings: solver setters, breaking threshold, deactivation, substeps -------------
void buildSettings(World& w) { buildPyramid(w); }

Settings settingsVariant() {
  Settings s;
  s.ticrate = 50.0f;
  s.substeps = 3;
  s.contactBreaking = 0.05f;
  s.deactivationTime = 0.5f;
  s.linSleep = 0.5f;
  s.angSleep = 0.8f;
  s.setSolverParams = true;
  s.sor = 1.2f; s.tau = 0.4f; s.damping = 0.9f;
  return s;
}

}  // namespace

const std::vector<Scene>& scenes() {
  static std::vector<Scene> list;
  if (list.empty()) {
    Settings d;
    list.push_back({"stack_10", "1,11,12", 300, d, buildStack, 0});
    list.push_back({"pyramid_10", "1,11,12", 300, d, buildPyramid, 0});
    list.push_back({"shape_zoo", "4,11,12", 300, d, buildShapeZoo, 0});
    list.push_back({"joints_six", "7,12", 480, d, buildJoints, 0});
    list.push_back({"vehicle", "8,12", 480, d, buildVehicle, driveVehicle});
    list.push_back({"character", "3,9,12", 480, d, buildCharacter, driveCharacter});
    list.push_back({"soft_body", "3,10,12", 300, d, buildSoft, 0});
    list.push_back({"sensors_filters", "3,5,6,12", 240, d, buildSensors, 0});
    list.push_back({"projectile", "4,12", 120, d, buildProjectile, 0});
    list.push_back({"mesh_slide", "4,6,12", 240, d, buildMeshSlide, 0});
    list.push_back({"body_types", "3,11,12", 240, d, buildBodyTypes, 0});
    list.push_back({"kinematic_switch", "3,11,12", 240, d, buildKinematic, driveKinematic});
    list.push_back({"settings", "1,2,11,12", 300, settingsVariant(), buildSettings, 0});
  }
  return list;
}

const Scene* findScene(const std::string& name) {
  for (size_t i = 0; i < scenes().size(); ++i)
    if (name == scenes()[i].name) return &scenes()[i];
  return 0;
}

}  // namespace bn
