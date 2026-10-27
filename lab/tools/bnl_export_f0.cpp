// Bullet NEXT Lab - one-time export of the 16 F0/F1 harness scenes to scene files (FL, CL.5).
// The builders below are the C++ scenes of harness/src/bn_scenes.cpp (removed in CL.5), with
// every World call turned into a scene entry. Built against Blender 2.79b's Bullet 2.84 copy so
// that rotations and normalised axes carry exactly the floats the F0 goldens were made with.
//   bnl_export_f0 <out_dir>
#include "bnl_scene.h"

#include "btBulletDynamicsCommon.h"

#include <cmath>
#include <cstdio>

using namespace bnl;

namespace {

const float kPi = 3.14159265358979f;

Vec3 V(const btVector3& v) { return Vec3(v.x(), v.y(), v.z()); }
Quat Q(const btQuaternion& q) { return Quat(q.x(), q.y(), q.z(), q.w()); }

Xform at(float x, float y, float z) { Xform t; t.p = Vec3(x, y, z); return t; }
Xform atRot(float x, float y, float z, const btVector3& axis, float angle) {
  Xform t; t.p = Vec3(x, y, z); t.q = Q(btQuaternion(axis, angle)); return t;
}

ShapeDesc box(const Vec3& h) { ShapeDesc s; s.type = ShapeDesc::kBox; s.half = h; return s; }
ShapeDesc sphere(float r) { ShapeDesc s; s.type = ShapeDesc::kSphere; s.radius = r; return s; }
ShapeDesc cylZ(const Vec3& h) { ShapeDesc s; s.type = ShapeDesc::kCylinderZ; s.half = h; return s; }
ShapeDesc coneZ(float r, float h) { ShapeDesc s; s.type = ShapeDesc::kConeZ; s.radius = r; s.height = h; return s; }
ShapeDesc capZ(float r, float h) { ShapeDesc s; s.type = ShapeDesc::kCapsuleZ; s.radius = r; s.height = h; return s; }
MeshDesc grid(int nx, int ny, float size) { MeshDesc m; m.gen = MeshDesc::kGrid; m.nx = nx; m.ny = ny; m.size = size; return m; }
MeshDesc boxM(const Vec3& h) { MeshDesc m; m.gen = MeshDesc::kBox; m.half = h; return m; }
ShapeDesc mesh(const MeshDesc& m, bool dynamic) { ShapeDesc s; s.type = ShapeDesc::kMesh; s.mesh = m; s.dynamicMesh = dynamic; return s; }

int ground(SceneDesc& w, float half = 20.0f) {
  BodyDesc o;
  o.kind = kStatic; o.shape = box(Vec3(half, half, 0.5f)); o.xform = at(0, 0, -0.5f); o.name = "ground";
  return w.addBody(o);
}

int meshGround(SceneDesc& w, int cells, float half) {
  BodyDesc o;
  o.kind = kStatic; o.shape = mesh(grid(cells, cells, half), false); o.name = "mesh_ground";
  return w.addBody(o);
}

int boxBody(SceneDesc& w, const Xform& t, const Vec3& half, float mass, BodyKind kind = kRigid) {
  BodyDesc o;
  o.kind = kind; o.shape = box(half); o.xform = t; o.mass = mass;
  return w.addBody(o);
}

int p2p(SceneDesc& w, int a, int b, const Vec3& pa, const Vec3& pb) {
  ConstraintDesc c; c.type = ConstraintDesc::kPointToPoint; c.a = a; c.b = b; c.pivotA = pa; c.pivotB = pb;
  return w.addConstraint(c);
}
int framed(SceneDesc& w, ConstraintDesc::Type type, int a, int b, const Xform& fa, const Xform& fb, bool angularOnly = false) {
  ConstraintDesc c; c.type = type; c.a = a; c.b = b; c.frameA = fa; c.frameB = fb; c.angularOnly = angularOnly;
  return w.addConstraint(c);
}

void linVel(SceneDesc& w, int b, const Vec3& v) { w.bodies[b].hasLinVel = true; w.bodies[b].linVel = v; }
void angVel(SceneDesc& w, int b, const Vec3& v) { w.bodies[b].hasAngVel = true; w.bodies[b].angVel = v; }

ActionDesc action(ActionDesc::Op op, int from, int to, int target) {
  ActionDesc a; a.op = op; a.from = from; a.to = to; a.target = target; return a;
}

// --- 1. stack_10
void buildStack(SceneDesc& w) {
  ground(w);
  for (int i = 0; i < 10; ++i) boxBody(w, at(0, 0, 0.5f + i * 1.0f), Vec3(0.5f, 0.5f, 0.5f), 1.0f);
}

// --- 2. pyramid_10
void buildPyramid(SceneDesc& w) {
  ground(w);
  const int rows = 10;
  for (int r = 0; r < rows; ++r)
    for (int i = 0; i < rows - r; ++i)
      boxBody(w, at(-0.5f * (rows - r - 1) + i * 1.0f, 0, 0.5f + r * 1.0f), Vec3(0.48f, 0.48f, 0.48f), 1.0f);
}

// --- 3. shape_zoo
void buildShapeZoo(SceneDesc& w) {
  meshGround(w, 16, 12.0f);
  BodyDesc o;
  o.kind = kRigid; o.mass = 1.0f; o.friction = 0.6f; o.restitution = 0.2f;
  o.shape = box(Vec3(0.5f, 0.5f, 0.5f)); o.xform = atRot(-6, 0, 3, btVector3(1, 1, 0).normalized(), 0.4f); w.addBody(o);
  o.shape = sphere(0.5f); o.xform = at(-4, 0, 3); w.addBody(o);
  o.shape = cylZ(Vec3(0.5f, 0.5f, 0.7f)); o.xform = atRot(-2, 0, 3, btVector3(1, 0, 0), 1.2f); w.addBody(o);
  o.shape = coneZ(0.5f, 1.2f); o.xform = atRot(0, 0, 3, btVector3(0, 1, 0), 0.7f); w.addBody(o);
  o.shape = capZ(0.4f, 1.0f); o.xform = atRot(2, 0, 3, btVector3(1, 0, 0), kPi / 2); w.addBody(o);
  ShapeDesc hull; hull.type = ShapeDesc::kHull;
  for (int i = 0; i < 12; ++i) {
    float a = i * 2 * kPi / 12;
    hull.points.push_back(Vec3(0.6f * std::cos(a), 0.6f * std::sin(a), i % 2 ? 0.5f : -0.5f));
  }
  o.shape = hull; o.xform = atRot(4, 0, 3, btVector3(0, 1, 1).normalized(), 0.3f); o.inertia = 0.6f; w.addBody(o);
  o.inertia = 0.4f;
  o.shape = mesh(boxM(Vec3(0.6f, 0.4f, 0.3f)), true); o.xform = atRot(6, 0, 3, btVector3(0, 0, 1), 0.5f); w.addBody(o);
  ShapeDesc comp; comp.type = ShapeDesc::kCompound;
  ChildDesc c1; c1.xform = at(0, 0, 0); c1.shape = box(Vec3(0.6f, 0.15f, 0.15f)); comp.children.push_back(c1);
  ChildDesc c2; c2.xform = at(0, 0, 0.45f); c2.shape = sphere(0.3f); comp.children.push_back(c2);
  o.shape = comp; o.xform = atRot(0, 3, 3, btVector3(1, 0, 0), 0.3f); w.addBody(o);
}

// --- 4. joints_six
void buildJoints(SceneDesc& w) {
  ground(w);
  int prev = -1;
  for (int i = 0; i < 5; ++i) {
    int b = boxBody(w, at(-6, 0, 9.0f - i * 1.0f), Vec3(0.1f, 0.1f, 0.4f), 1.0f);
    if (prev < 0) p2p(w, b, -1, Vec3(0, 0, 0.5f), Vec3(-6, 0, 9.5f));
    else p2p(w, prev, b, Vec3(0, 0, -0.5f), Vec3(0, 0, 0.5f));
    prev = b;
  }
  if (prev >= 0) linVel(w, prev, Vec3(3, 0, 0));
  int post = boxBody(w, at(-2, 0, 2), Vec3(0.1f, 0.1f, 2), 0, kStatic);
  int door = boxBody(w, at(-1.2f, 0, 2), Vec3(0.7f, 0.05f, 1.5f), 2.0f);
  Xform fa = at(0.1f, 0, 0), fb = at(-0.8f, 0, 0);
  fa.q = Q(btQuaternion(btVector3(1, 0, 0), kPi / 2)); fb.q = fa.q;
  int h = framed(w, ConstraintDesc::kHinge, post, door, fa, fb, false);
  w.addParam(h, 3, -1.0f, 1.0f);
  angVel(w, door, Vec3(0, 0, 4));
  int wheel = boxBody(w, at(1, 0, 3), Vec3(0.6f, 0.6f, 0.1f), 1.0f);
  int hw = framed(w, ConstraintDesc::kHinge, wheel, -1, at(0, 0, 0), at(1, 0, 3), true);
  w.addParam(hw, 3, 1.0f, -1.0f);
  angVel(w, wheel, Vec3(0, 0, 5));
  int arm = boxBody(w, at(4, 0, 3), Vec3(0.15f, 0.15f, 0.8f), 1.0f);
  int ct = framed(w, ConstraintDesc::kConeTwist, arm, -1, at(0, 0, 0.8f), at(4, 0, 3.8f));
  w.addParam(ct, 3, 0, 0.5f); w.addParam(ct, 4, 0, 0.7f); w.addParam(ct, 5, 0, -1.0f);
  angVel(w, arm, Vec3(2, 1, 0));
  int baseB = boxBody(w, at(7, 0, 0.5f), Vec3(0.5f, 0.5f, 0.5f), 0, kStatic);
  int slider = boxBody(w, at(7, 0, 2), Vec3(0.3f, 0.3f, 0.3f), 1.0f);
  int g = framed(w, ConstraintDesc::kGeneric6DofSpring, baseB, slider, at(0, 0, 1.5f), at(0, 0, 0));
  w.addParam(g, 0, -1.0f, 1.0f);
  w.addParam(g, 1, 0.0f, 0.0f);
  w.addParam(g, 2, -0.5f, 0.5f);
  w.addParam(g, 3, 1.0f, -1.0f);
  w.addParam(g, 4, 0.0f, 0.0f);
  w.addParam(g, 5, 0.0f, 0.0f);
  w.addParam(g, 6, 0.8f, 5.0f);
  w.addParam(g, 9, 2.0f, 2.0f);
  w.addParam(g, 14, 40.0f, 0.5f);
}

// --- 5. vehicle
void buildVehicle(SceneDesc& w) {
  meshGround(w, 20, 40.0f);
  boxBody(w, atRot(0, 14, 0.2f, btVector3(1, 0, 0), 0.15f), Vec3(3, 3, 0.4f), 0, kStatic);
  BodyDesc c;
  c.kind = kRigid; c.mass = 800.0f; c.shape = box(Vec3(0.9f, 2.0f, 0.4f)); c.xform = at(0, 0, 1.2f);
  c.linDamp = 0.04f; c.angDamp = 0.1f; c.noSleep = true;
  int chassis = w.addBody(c);
  int v = w.addVehicle(chassis);
  const Vec3 down(0, 0, -1), axle(-1, 0, 0);
  const Vec3 pos[4] = {Vec3(0.9f, 1.4f, 0), Vec3(-0.9f, 1.4f, 0), Vec3(0.9f, -1.4f, 0), Vec3(-0.9f, -1.4f, 0)};
  for (int i = 0; i < 4; ++i) {
    WheelDesc wd;
    wd.vehicle = v; wd.pos = pos[i]; wd.down = down; wd.axle = axle; wd.restLength = 0.5f; wd.radius = 0.4f; wd.steering = i < 2;
    wd.tuning.set = true;
    wd.tuning.suspensionStiffness = 20.0f; wd.tuning.dampingCompression = 2.0f; wd.tuning.dampingRelaxation = 3.0f;
    wd.tuning.frictionSlip = 2.0f; wd.tuning.rollInfluence = 0.1f;
    w.addWheel(wd);
  }
  ActionDesc e = action(ActionDesc::kVehicle, 0, 239, v); e.engine = {0.0f, 0.0f, 1500.0f, 1500.0f}; w.actions.push_back(e);
  ActionDesc s = action(ActionDesc::kVehicle, 121, 299, v); s.steer = {0.3f, 0.3f}; w.actions.push_back(s);
  ActionDesc b = action(ActionDesc::kVehicle, 361, -1, v); b.brake = {40.0f, 40.0f, 40.0f, 40.0f}; w.actions.push_back(b);
}

// --- 6. character
void buildCharacter(SceneDesc& w) {
  ground(w);
  for (int s = 0; s < 6; ++s) boxBody(w, at(0, 3.0f + s * 0.6f, 0.1f + s * 0.2f), Vec3(2, 0.3f, 0.1f + s * 0.2f), 0, kStatic);
  BodyDesc o;
  o.kind = kCharacter; o.shape = capZ(0.4f, 1.0f); o.xform = at(0, 0, 1.0f);
  o.stepHeight = 0.35f; o.jumpSpeed = 6.0f; o.fallSpeed = 55.0f;
  int ch = w.addBody(o);
  ActionDesc a = action(ActionDesc::kWalk, 0, 299, ch); a.v = Vec3(0, 0.05f, 0); w.actions.push_back(a);
  ActionDesc b = action(ActionDesc::kWalk, 300, -1, ch); b.v = Vec3(0.03f, -0.02f, 0); w.actions.push_back(b);
  w.actions.push_back(action(ActionDesc::kJump, 330, 330, ch));
  w.actions.push_back(action(ActionDesc::kJump, 400, 400, ch));
}

// --- 7. soft_body
void buildSoft(SceneDesc& w) {
  ground(w);
  BodyDesc o;
  o.kind = kSoft; o.mass = 1.0f; o.friction = 0.5f;
  o.softMesh = grid(6, 6, 1.0f);
  o.xform = atRot(0, 0, 3, btVector3(1, 0, 0), 0.4f);
  o.bending = true; o.shapeMatching = false;
  w.addBody(o);
  o.softMesh = boxM(Vec3(0.5f, 0.5f, 0.5f));
  o.xform = at(3, 0, 2);
  o.bending = false; o.shapeMatching = true;
  w.addBody(o);
  boxBody(w, at(-3, 0, 1), Vec3(0.5f, 0.5f, 0.5f), 1.0f);
}

// --- 8. sensors_filters
void buildSensors(SceneDesc& w) {
  ground(w);
  BodyDesc s;
  s.kind = kSensor; s.shape = box(Vec3(1.5f, 1.5f, 1.5f)); s.xform = at(0, 0, 1.5f); s.reportCollisions = true; s.name = "sensor";
  w.addBody(s);
  int falling = boxBody(w, at(0, 0, 6), Vec3(0.4f, 0.4f, 0.4f), 1.0f);
  w.bodies[falling].reportCollisions = true;
  BodyDesc gh;
  gh.kind = kRigid; gh.ghost = true; gh.shape = sphere(0.5f); gh.xform = at(3, 0, 4); gh.reportCollisions = true;
  w.addBody(gh);
  BodyDesc a;
  a.kind = kRigid; a.shape = box(Vec3(0.4f, 0.4f, 0.4f)); a.xform = at(-3, 0, 6);
  a.userGroup = 1; a.userMask = 0xffff ^ 2; a.reportCollisions = true;
  w.addBody(a);
  BodyDesc b = a;
  b.shape = box(Vec3(1, 1, 0.2f)); b.kind = kStatic; b.xform = at(-3, 0, 3); b.userGroup = 2; b.userMask = 0xffff;
  b.reportCollisions = false;
  w.addBody(b);
  BodyDesc c = b;
  c.shape = box(Vec3(1, 1, 0.2f)); c.xform = at(-3, 0, 1); c.userGroup = 4;
  w.addBody(c);
}

// --- 9. projectile
void buildProjectile(SceneDesc& w) {
  ground(w);
  boxBody(w, at(10, 0, 2), Vec3(0.025f, 3, 2), 0, kStatic);
  for (int i = 0; i < 5; ++i) {
    BodyDesc o;
    o.kind = kRigid; o.mass = 0.1f; o.shape = sphere(0.1f); o.xform = at(0, -2.0f + i, 2);
    int b = w.addBody(o);
    linVel(w, b, Vec3(60.0f + 60.0f * i, 0, 0));
  }
}

// --- 9b. tunnel_100
void buildTunnel(SceneDesc& w) {
  ground(w);
  boxBody(w, at(10, 0, 3), Vec3(0.025f, 4, 3), 0, kStatic);
  for (int j = 0; j < 10; ++j)
    for (int k = 0; k < 10; ++k) {
      BodyDesc o;
      o.kind = kRigid; o.mass = 0.05f; o.shape = sphere(0.1f);
      o.xform = at(0.37f, -2.7f + 0.6f * j, 0.4f + 0.55f * k);
      int b = w.addBody(o);
      linVel(w, b, Vec3(300, 0, 0));
    }
}

// --- 10. mesh_slide
void buildMeshSlide(SceneDesc& w) {
  meshGround(w, 40, 20.0f);
  BodyDesc o;
  o.kind = kRigid; o.mass = 1.0f; o.friction = 0.1f; o.shape = box(Vec3(0.3f, 0.3f, 0.3f));
  o.xform = at(-15, 0.37f, 0.31f);
  int b = w.addBody(o);
  linVel(w, b, Vec3(8, 0, 0));
  o.shape = sphere(0.3f); o.xform = at(-15, -3.13f, 0.31f);
  int s = w.addBody(o);
  linVel(w, s, Vec3(8, 0, 0));
}

// --- 11. body_types
void buildBodyTypes(SceneDesc& w) {
  ground(w);
  BodyDesc o;
  o.kind = kDynamic; o.shape = box(Vec3(0.5f, 0.5f, 0.5f)); o.xform = atRot(-4, 0, 3, btVector3(1, 0, 0), 0.6f);
  int d = w.addBody(o);
  angVel(w, d, Vec3(5, 0, 0));
  o.kind = kRigid; o.shape = box(Vec3(0.5f, 0.5f, 0.5f)); o.xform = atRot(-1, 0, 3, btVector3(1, 1, 0).normalized(), 0.6f);
  o.linFactor = Vec3(1, 0, 1); o.angFactor = Vec3(0, 0, 1);
  int l = w.addBody(o);
  linVel(w, l, Vec3(1, 3, 0));
  o.linFactor = Vec3(1, 1, 1); o.angFactor = Vec3(1, 1, 1);
  o.shape = box(Vec3(0.5f, 0.5f, 0.5f)); o.xform = at(2, 0, 1); o.clampVelMax = 2.0f; o.clampAngMax = 1.0f;
  int c = w.addBody(o);
  linVel(w, c, Vec3(10, 0, 5));
  angVel(w, c, Vec3(0, 0, 10));
  o.shape = box(Vec3(0.5f, 0.5f, 0.5f)); o.clampVelMax = 0; o.clampAngMax = 0; o.xform = at(5, 0, 1); o.linDamp = 0.6f; o.angDamp = 0.6f;
  int dm = w.addBody(o);
  linVel(w, dm, Vec3(0, 6, 4));
}

// --- 12. kinematic_switch
void buildKinematic(SceneDesc& w) {
  ground(w);
  boxBody(w, at(0, 0, 4), Vec3(1.5f, 1.5f, 0.2f), 5.0f);
  for (int i = 0; i < 3; ++i) boxBody(w, at(-0.5f + i * 0.5f, 0, 5 + i), Vec3(0.2f, 0.2f, 0.2f), 0.5f);
  w.actions.push_back(action(ActionDesc::kSuspend, 60, 60, 1));
  ActionDesc up = action(ActionDesc::kKinematicMove, 61, 149, 1); up.v = Vec3(0.02f, 0, 0.01f); w.actions.push_back(up);
  ActionDesc dn = action(ActionDesc::kKinematicMove, 150, -1, 1); dn.v = Vec3(0.02f, 0, -0.01f); w.actions.push_back(dn);
}

// --- 14. pile_1000
void buildPile(SceneDesc& w) {
  ground(w);
  for (int side = 0; side < 4; ++side) {
    const float x = side < 2 ? (side ? 6.0f : -6.0f) : 0, y = side < 2 ? 0 : (side == 3 ? 6.0f : -6.0f);
    boxBody(w, at(x, y, 4), side < 2 ? Vec3(0.25f, 6.25f, 4) : Vec3(6.25f, 0.25f, 4), 0, kStatic);
  }
  for (int i = 0; i < 1000; ++i) {
    const int ix = i % 10, iy = (i / 10) % 10, iz = i / 100;
    BodyDesc o;
    o.kind = kRigid; o.mass = 1.0f; o.friction = 0.6f;
    switch (i % 3) {
      case 0: o.shape = box(Vec3(0.25f, 0.25f, 0.25f)); break;
      case 1: o.shape = sphere(0.25f); break;
      default: o.shape = capZ(0.18f, 0.3f); break;
    }
    o.xform = atRot(-4.5f + ix * 1.0f + 0.05f * (iz % 2), -4.5f + iy * 1.0f, 1.0f + iz * 0.8f,
                    btVector3(1, 0, 0), 0.3f * (i % 5));
    w.addBody(o);
  }
}

// --- 15. joint_grid
void buildJointGrid(SceneDesc& w) {
  ground(w);
  const int n = 20;
  const float s = 0.5f;
  std::vector<int> id(n * n);
  for (int j = 0; j < n; ++j)
    for (int i = 0; i < n; ++i) {
      BodyDesc o;
      o.kind = kRigid; o.mass = 0.2f; o.shape = sphere(0.12f);
      o.xform = at(-0.5f * s * (n - 1) + i * s, -0.5f * s * (n - 1) + j * s, 8.0f);
      id[j * n + i] = w.addBody(o);
    }
  const btVector3 half(0.5f * s, 0, 0), halfY(0, 0.5f * s, 0);
  for (int j = 0; j < n; ++j)
    for (int i = 0; i < n; ++i) {
      const int b = id[j * n + i];
      if (i + 1 < n) p2p(w, b, id[j * n + i + 1], V(half), V(-half));
      if (j + 1 < n) p2p(w, b, id[(j + 1) * n + i], V(halfY), V(-halfY));
    }
  const int corners[4] = {0, n - 1, n * (n - 1), n * n - 1};
  for (int c : corners) p2p(w, id[c], -1, Vec3(0, 0, 0), w.bodies[id[c]].xform.p);  // centre of mass = origin
  for (int k = 0; k < 4; ++k) boxBody(w, at(-1.0f + k * 0.7f, 0.3f * k, 10.0f + k), Vec3(0.3f, 0.3f, 0.3f), 2.0f);
}

void buildSettings(SceneDesc& w) { buildPyramid(w); }

SettingsDesc settingsVariant() {
  SettingsDesc s;
  s.ticrate = 50.0f; s.substeps = 3; s.contactBreaking = 0.05f; s.deactivationTime = 0.5f;
  s.linSleep = 0.5f; s.angSleep = 0.8f;
  s.setSolverParams = true; s.sor = 1.2f; s.tau = 0.4f; s.damping = 0.9f;
  return s;
}

struct Entry { const char* name; const char* covers; int frames; bool variant; void (*build)(SceneDesc&); };

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) { std::fprintf(stderr, "usage: %s <out_dir>\n", argv[0]); return 2; }
  const std::string dir = argv[1];
  const Entry list[] = {
      {"stack_10", "1,11,12", 300, false, buildStack},
      {"pyramid_10", "1,11,12", 300, false, buildPyramid},
      {"shape_zoo", "4,11,12", 300, false, buildShapeZoo},
      {"joints_six", "7,12", 480, false, buildJoints},
      {"vehicle", "8,12", 480, false, buildVehicle},
      {"character", "3,9,12", 480, false, buildCharacter},
      {"soft_body", "3,10,12", 300, false, buildSoft},
      {"sensors_filters", "3,5,6,12", 240, false, buildSensors},
      {"projectile", "4,12", 120, false, buildProjectile},
      {"tunnel_100", "4,12", 60, false, buildTunnel},
      {"mesh_slide", "4,6,12", 240, false, buildMeshSlide},
      {"body_types", "3,11,12", 240, false, buildBodyTypes},
      {"kinematic_switch", "3,11,12", 240, false, buildKinematic},
      {"pile_1000", "1,4,11,12", 300, false, buildPile},
      {"joint_grid", "7,12", 300, false, buildJointGrid},
      {"settings", "1,2,11,12", 300, true, buildSettings},
  };
  std::string index = "{\n  \"scenes\": [\n";
  const int n = sizeof(list) / sizeof(list[0]);
  for (int i = 0; i < n; ++i) {
    SceneDesc s;
    s.name = list[i].name; s.covers = list[i].covers; s.frames = list[i].frames;
    if (list[i].variant) s.settings = settingsVariant();
    list[i].build(s);
    const std::string path = dir + "/" + s.name + ".json";
    if (!saveScene(path, s)) { std::fprintf(stderr, "cannot write %s\n", s.name.c_str()); return 1; }
    // Round trip: every float must read back to itself (each float has its own decimal form).
    SceneDesc back;
    std::string err;
    if (!loadScene(path, back, err) || sceneToJson(back) != sceneToJson(s)) {
      std::fprintf(stderr, "%s does not read back unchanged %s\n", path.c_str(), err.c_str());
      return 1;
    }
    index += std::string("    \"") + s.name + "\"" + (i + 1 < n ? ",\n" : "\n");
    std::printf("%-18s %zu objects, %zu actions\n", s.name.c_str(), s.objects.size(), s.actions.size());
  }
  index += "  ]\n}\n";
  FILE* f = std::fopen((dir + "/index.json").c_str(), "w");
  if (!f) return 1;
  std::fputs(index.c_str(), f);
  std::fclose(f);
  return 0;
}
