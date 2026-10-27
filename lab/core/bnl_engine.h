// Bullet NEXT Lab - one interface over every physics engine the Lab runs (FL, CL.2).
// An Engine builds a world from a scene description, steps it and reports its state. It has no
// physics-engine types, so several engines (legacy Bullet, Bullet NEXT, Jolt...) can live in one
// program, e.g. the HARFANG viewer showing two of them side by side.
#ifndef BNL_ENGINE_H
#define BNL_ENGINE_H

#include "bnl_scene.h"

#include <cstdint>
#include <string>
#include <vector>

namespace bnl {

typedef std::uint64_t u64;

struct BodyState {
  Vec3 p;
  Quat q;
  Vec3 lin, ang;
};

struct Contact {
  int a = -1, b = -1;   // body indices (-1: not a scene body)
  Vec3 pa, pb, n;       // world points on A and B, normal on B
  float dist = 0;       // negative when penetrating
};

struct RayHit {
  int body = -1;
  Vec3 p, n;
  float t = 1;          // fraction along the ray
};

// Solver work since build() (F3c, C3.9); filled by engines that can measure it.
struct SolverStats {
  double prepareMicros = 0, substepMicros = 0, restitutionMicros = 0, storeMicros = 0;
  int groups = 0, bodies = 0, contactPoints = 0, jointRows = 0;
};

// How far each joint is from what it holds together (F3c, C3.9): the distance between the two
// anchor points (where the joint holds them together) and the angle between the hinge axes.
struct JointError {
  int constraint = -1;    // scene constraint index
  float linear = -1;      // metres; -1: the joint leaves the anchors free
  float angular = -1;     // radians; -1: the joint leaves the axis free
};

struct EngineOptions {
  int threads = -1;           // Bullet NEXT threaded world: -1 = BN_THREADS variable, 0 = off
  float gravityOffset = 0;    // added to gravity Z (the harness --variant switch)
  std::string recording;      // engine "replay": the recording to play (bnl_recording.h)
};

class Engine {
 public:
  virtual ~Engine() {}
  virtual const char* backend() const = 0;

  // Builds the scene's world. Returns false (with a message) if the engine cannot.
  virtual bool build(const SceneDesc& s, const EngineOptions& o, std::string& error) = 0;
  // One frame: the scene's scripted actions, then one fixed step.
  virtual void advance() = 0;
  virtual int frame() const = 0;
  virtual double lastStepMicros() const = 0;

  // State
  virtual int bodyCount() const = 0;
  virtual BodyState body(int i) const = 0;
  virtual void softNodes(int body, std::vector<Vec3>& out) const = 0;   // empty if not soft
  virtual int vehicleCount() const = 0;
  virtual int wheelCount(int vehicle) const = 0;
  virtual Xform wheel(int vehicle, int wheel) const = 0;
  virtual void contacts(std::vector<Contact>& out) const = 0;
  // Same hashes as the harness recorder: the last frame's, and the chain over all frames.
  virtual u64 frameHash() const = 0;
  virtual u64 finalHash() const = 0;

  // Game control (what game code does between frames)
  virtual void setLinearVelocity(int b, const Vec3& v) = 0;
  virtual void setAngularVelocity(int b, const Vec3& v) = 0;
  virtual void applyForce(int b, const Vec3& f) = 0;
  virtual void applyForceAt(int b, const Vec3& f, const Vec3& worldPoint) = 0;
  virtual void applyImpulse(int b, const Vec3& j) = 0;
  virtual void applyImpulseAt(int b, const Vec3& j, const Vec3& worldPoint) = 0;
  virtual void applyTorque(int b, const Vec3& t) = 0;
  virtual void applyTorqueImpulse(int b, const Vec3& t) = 0;
  virtual void setTransform(int b, const Xform& x) = 0;   // teleport a dynamic body, move a kinematic one
  virtual void wake(int b) = 0;
  virtual void setSleeping(int b, bool allowed) = 0;
  virtual bool raycast(const Vec3& from, const Vec3& to, RayHit& hit) const = 0;
  virtual void raycastAll(const Vec3& from, const Vec3& to, std::vector<RayHit>& hits) const = 0;
  virtual void vehicleControl(int vehicle, int wheel, float engine, float brake, float steer) = 0;
  virtual void characterWalk(int b, const Vec3& step) = 0;
  virtual void characterJump(int b) = 0;

  // Measurements (F3c, C3.9). Engines that cannot measure them return false or nothing.
  virtual bool solverStats(SolverStats& out) const { (void)out; return false; }
  virtual void jointErrors(std::vector<JointError>& out) const { out.clear(); }
  // Kinetic plus potential energy of the moving rigid bodies (J), gravity along Z.
  virtual bool energy(double& out) const { out = 0; return false; }
};

// Random collision pairs (TL.7, harness/src/bn_fuzz.cpp): Bullet engines only.
struct FuzzResult {
  int pairs = 0, touching = 0, contacts = 0;
  int nonFinite = 0;     // contacts or bodies with NaN or infinity
  int badNormals = 0;    // contact normals that are not unit length
  int failed = 0;        // pairs with a NaN or infinity
  float deepest = 0;     // most negative contact distance seen
  std::vector<std::string> failures;  // the first few failing pairs ("seed S pair P: ...")
};
// False if the engine has no pair fuzzer in this program.
bool fuzzCollisionPairs(const std::string& engine, unsigned seed, int pairs, FuzzResult& r);

// Engines linked into this program ("legacy", "next", "nextf", "jolt"...).
std::vector<std::string> engineNames();
Engine* createEngine(const std::string& name);   // null if that engine is not linked in;
                                                 // "replay" plays a recording (any program)

}  // namespace bnl

#endif
