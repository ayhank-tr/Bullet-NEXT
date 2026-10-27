// Bullet NEXT Lab - scene description (FL, CL.5).
// One scene file is the single source of a test scene: the harness, every Lab engine and the
// HARFANG host all build from it. The description has no physics-engine types, so it is compiled
// once and shared by all backends. Objects are kept in creation order, because creation order
// is part of a scene's exact result.
#ifndef BNL_SCENE_H
#define BNL_SCENE_H

#include <cstdint>
#include <string>
#include <vector>

namespace bnl {

struct Vec3 {
  float x = 0, y = 0, z = 0;
  Vec3() {}
  Vec3(float a, float b, float c) : x(a), y(b), z(c) {}
};

struct Quat {
  float x = 0, y = 0, z = 0, w = 1;
  Quat() {}
  Quat(float a, float b, float c, float d) : x(a), y(b), z(c), w(d) {}
};

struct Xform {
  Vec3 p;
  Quat q;
};

// A triangle mesh, given explicitly or generated (the generators are part of every backend).
struct MeshDesc {
  enum Gen { kExplicit, kGrid, kBox };
  Gen gen = kExplicit;
  int nx = 0, ny = 0;    // grid: cells
  float size = 0;        // grid: half size
  Vec3 half;             // box: half extents
  std::vector<Vec3> verts;
  std::vector<int> tris;
};

struct ShapeDesc;
struct ChildDesc;

struct ShapeDesc {
  enum Type { kBox, kSphere, kCylinderZ, kConeZ, kCapsuleZ, kHull, kMesh, kCompound };
  Type type = kBox;
  Vec3 half;                      // box, cylinder
  float radius = 0, height = 0;   // sphere, cone, capsule
  float margin = 0.04f;
  std::vector<Vec3> points;       // hull
  MeshDesc mesh;                  // mesh
  bool dynamicMesh = false;       // mesh: GImpact (dynamic) or BVH (static)
  Vec3 scale = Vec3(1, 1, 1);     // mesh
  std::vector<ChildDesc> children;  // compound
};

struct ChildDesc {
  Xform xform;
  ShapeDesc shape;
};

// The game-engine physics types (Blender 2.79b names).
enum BodyKind { kStatic, kDynamic, kRigid, kSensor, kCharacter, kSoft };

struct BodyDesc {
  std::string name;
  BodyKind kind = kRigid;
  ShapeDesc shape;
  Xform xform;
  float mass = 1.0f;
  float friction = 0.5f;
  float restitution = 0.0f;
  float linDamp = 0.04f, angDamp = 0.1f;
  float inertia = 0.4f;           // Blender "form factor"
  Vec3 linFactor = Vec3(1, 1, 1), angFactor = Vec3(1, 1, 1);
  bool ghost = false;
  uint16_t userGroup = 0xffff, userMask = 0xffff;
  float clampVelMin = 0, clampVelMax = 0, clampAngMin = 0, clampAngMax = 0;
  bool reportCollisions = false;
  // Applied after every object of the scene exists.
  bool hasLinVel = false, hasAngVel = false;
  Vec3 linVel, angVel;
  bool noSleep = false;
  // Blender rigid body world (profile "blender53-rb"): passive bodies are kStatic, active kRigid.
  bool animated = false;           // passive and keyframed: a kinematic body moved by actions
  bool useDeactivation = false, startDeactivated = false;
  float linSleep = 0.4f, angSleep = 0.5f;
  int colGroups = 1;               // collision collections (bit mask)
  // Character
  float stepHeight = 0, jumpSpeed = 0, fallSpeed = 0;
  // Soft body
  MeshDesc softMesh;
  bool bending = false, shapeMatching = false;
};

struct ConstraintDesc {
  // Blender game engine constraints (profile bge279), then Blender's rigid body constraints
  // (profile blender53-rb: placed by a world-space frame, between two bodies).
  enum Type { kPointToPoint, kHinge, kConeTwist, kGeneric6DofSpring,
              kRbFixed, kRbPoint, kRbHinge, kRbSlider, kRbPiston, kRbGeneric, kRbGenericSpring, kRbMotor };
  Type type = kPointToPoint;
  int a = 0, b = -1;              // body indices; b = -1 attaches to the static world
  Vec3 pivotA, pivotB;            // point-to-point
  Xform frameA, frameB;           // the others
  bool angularOnly = false;       // hinge
  // Blender rigid body constraint settings (axes: linear x y z, angular x y z)
  Xform frame;                    // world-space pivot and orientation
  bool enabled = true, disableCollisions = true;
  float breakingThreshold = -1;   // -1: unbreakable
  int iterations = -1;            // -1: the world's
  bool useLimit[6] = {false, false, false, false, false, false};
  float lower[6] = {0, 0, 0, 0, 0, 0}, upper[6] = {0, 0, 0, 0, 0, 0};
  bool useSpring[6] = {false, false, false, false, false, false};
  float stiffness[6] = {10, 10, 10, 10, 10, 10}, damping[6] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f};
  int springType = 2;             // 1: btGeneric6DofSpringConstraint, 2: ...Spring2
  bool useMotorLin = false, useMotorAng = false;
  float motorLinVelocity = 1, motorAngVelocity = 1, motorLinMaxImpulse = 1, motorAngMaxImpulse = 1;
};

// KX_ConstraintWrapper.setParam(index, value0, value1)
struct ParamDesc {
  int constraint = 0, index = 0;
  float v0 = 0, v1 = 0;
};

struct WheelTuning {
  bool set = false;
  float suspensionStiffness = 0, dampingCompression = 0, dampingRelaxation = 0;
  float frictionSlip = 0, rollInfluence = 0;
};

struct WheelDesc {
  int vehicle = 0;
  Vec3 pos, down, axle;
  float restLength = 0, radius = 0;
  bool steering = false;
  WheelTuning tuning;
};

struct VehicleDesc {
  int chassis = 0;
};

// One entry of the scene, in creation order.
struct ObjectDesc {
  enum Type { kBody, kConstraint, kParam, kVehicle, kWheel };
  Type type = kBody;
  int index = 0;  // into the matching list below
};

// Scripted game logic, as data. Ranges are inclusive; to = -1 runs to the end.
struct ActionDesc {
  enum Op { kVehicle, kWalk, kJump, kSuspend, kKinematicMove, kLinVel, kImpulse };
  Op op = kVehicle;
  int from = 0, to = 0;
  int target = 0;                 // body or vehicle index
  std::vector<float> engine, brake, steer;  // vehicle, per wheel (empty: not set by this action)
  Vec3 v;                         // walk direction, move delta, velocity or impulse
};

struct SettingsDesc {
  float ticrate = 60.0f;
  int substeps = 1;
  float gravityZ = -9.81f;
  float contactBreaking = 0.02f;
  float deactivationTime = 2.0f;
  float linSleep = 0.8f, angSleep = 1.0f;
  bool setSolverParams = false;
  float sor = 1.0f, tau = 0.6f, damping = 1.0f;
  // Blender rigid body world: substeps per frame are `substeps`, frames per second `ticrate`.
  bool splitImpulse = false;
  float timeScale = 1.0f;
  // Solver and Bullet NEXT settings (BGE NEXT's physics panel); -1 = the engine's default.
  int iterations = -1;
  bool nncg = false;               // solver "nncg" (Bullet NEXT), else sequential impulse
  float erp = -1, erp2 = -1, cfm = -1;
  int features = -1;               // Bullet NEXT features: 2 linked collision, 4 hull clipping,
                                   // 8 active edges, 16 CCD, 1 world configuration (bits add up)
};

struct SceneDesc {
  std::string name, covers, profile = "bge279";
  int frames = 0;
  SettingsDesc settings;
  std::vector<ObjectDesc> objects;
  std::vector<BodyDesc> bodies;
  std::vector<ConstraintDesc> constraints;
  std::vector<ParamDesc> params;
  std::vector<VehicleDesc> vehicles;
  std::vector<WheelDesc> wheels;
  std::vector<ActionDesc> actions;

  int addBody(const BodyDesc& b);
  int addConstraint(const ConstraintDesc& c);
  void addParam(int constraint, int index, float v0, float v1);
  int addVehicle(int chassis);
  int addWheel(const WheelDesc& w);
};

// Mesh generators (identical in every backend; floats only).
void gridMesh(int nx, int ny, float size, std::vector<Vec3>& v, std::vector<int>& t);
void boxMesh(const Vec3& half, std::vector<Vec3>& v, std::vector<int>& t);
void meshData(const MeshDesc& m, std::vector<Vec3>& v, std::vector<int>& t);

bool loadScene(const std::string& path, SceneDesc& out, std::string& error);
bool parseScene(const std::string& json, SceneDesc& out, std::string& error);
std::string sceneToJson(const SceneDesc& s);
bool saveScene(const std::string& path, const SceneDesc& s);

// Scene files shipped with the Lab: <dir>/<name>.json, listed by index.json.
std::vector<std::string> listScenes(const std::string& dir);

}  // namespace bnl

#endif
