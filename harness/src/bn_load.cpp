// Bullet NEXT Lab - builds a harness World from a scene file (FL, CL.5).
// Compiled once per backend, like the rest of the harness.
#include "bn_load.h"

BN_NAMESPACE_BEGIN

Settings settingsOf(const bnl::SceneDesc& sc) {
  Settings s = settingsOf(sc.settings);
  s.profile = sc.profile == "blender53-rb" ? kBlenderRb : kBge279;
  return s;
}

Settings settingsOf(const bnl::SettingsDesc& d) {
  Settings s;
  s.ticrate = d.ticrate;
  s.substeps = d.substeps;
  s.gravityZ = d.gravityZ;
  s.contactBreaking = d.contactBreaking;
  s.deactivationTime = d.deactivationTime;
  s.linSleep = d.linSleep;
  s.angSleep = d.angSleep;
  s.setSolverParams = d.setSolverParams;
  s.sor = d.sor; s.tau = d.tau; s.damping = d.damping;
  s.iterations = d.iterations;
  s.nncg = d.nncg ? 1 : 0;
  s.erp = d.erp; s.erp2 = d.erp2; s.cfm = d.cfm;
  s.features = d.features;
  s.splitImpulse = d.splitImpulse;
  s.timeScale = d.timeScale;
  return s;
}

namespace {

void meshOf(const bnl::MeshDesc& m, std::vector<btVector3>& v, std::vector<int>& t) {
  std::vector<bnl::Vec3> mv;
  bnl::meshData(m, mv, t);
  v.clear();
  for (size_t i = 0; i < mv.size(); ++i) v.push_back(toBt(mv[i]));
}

BodyOpts optsOf(const bnl::BodyDesc& d) {
  BodyOpts o;
  o.kind = (BodyKind)d.kind;
  o.xform = toBt(d.xform);
  o.mass = d.mass;
  o.friction = d.friction;
  o.restitution = d.restitution;
  o.linDamp = d.linDamp; o.angDamp = d.angDamp;
  o.inertia = d.inertia;
  o.linFactor = toBt(d.linFactor); o.angFactor = toBt(d.angFactor);
  o.ghost = d.ghost;
  o.userGroup = d.userGroup; o.userMask = d.userMask;
  o.clampVelMin = d.clampVelMin; o.clampVelMax = d.clampVelMax;
  o.clampAngMin = d.clampAngMin; o.clampAngMax = d.clampAngMax;
  o.reportCollisions = d.reportCollisions;
  o.name = d.name;
  o.animated = d.animated;
  o.useDeactivation = d.useDeactivation;
  o.startDeactivated = d.startDeactivated;
  o.rbLinSleep = d.linSleep;
  o.rbAngSleep = d.angSleep;
  o.colGroups = d.colGroups;
  o.rbLoc = toBt(d.xform.p);
  o.rbRot = btQuaternion(d.xform.q.x, d.xform.q.y, d.xform.q.z, d.xform.q.w);
  return o;
}

}  // namespace

namespace {

// Blender 5.3's shapes where they differ from the game engine's: RB_shape_new_convex_hull
// (btConvexHullComputer, margin embedded when possible), RB_shape_new_trimesh (BVH mesh in a
// scaled wrapper, vertices stored as btScalar) and RB_shape_new_gimpact_mesh. The margin is set
// afterwards for every shape (RB_shape_set_margin), as rigidbody_validate_sim_shape does.
btCollisionShape* makeRbShape(const bnl::ShapeDesc& s, bool dynamicBody) {
  btCollisionShape* shape = 0;
  switch (s.type) {
    case bnl::ShapeDesc::kHull: {
      std::vector<float> pts;
      for (size_t i = 0; i < s.points.size(); ++i) { pts.push_back(s.points[i].x); pts.push_back(s.points[i].y); pts.push_back(s.points[i].z); }
      btConvexHullComputer hull;
      if (hull.compute(&pts[0], 3 * sizeof(float), (int)s.points.size(), s.margin, 0.0f) < 0.0f)
        hull.compute(&pts[0], 3 * sizeof(float), (int)s.points.size(), 0.0f, 0.0f);
      shape = new btConvexHullShape(&(hull.vertices[0].getX()), hull.vertices.size());
      break;
    }
    case bnl::ShapeDesc::kMesh: {
      std::vector<bnl::Vec3> mv; std::vector<int> mt;
      bnl::meshData(s.mesh, mv, mt);
      std::vector<btScalar> fv;
      for (size_t i = 0; i < mv.size(); ++i) { fv.push_back(mv[i].x); fv.push_back(mv[i].y); fv.push_back(mv[i].z); }
      btTriangleIndexVertexArray* arr = new OwnedMesh(fv, mt);
      if (dynamicBody) {
        btGImpactMeshShape* g = new btGImpactMeshShape(arr);
        g->updateBound();
        shape = g;
      } else {
        shape = new btScaledBvhTriangleMeshShape(new btBvhTriangleMeshShape(arr, true, true), btVector3(1.0f, 1.0f, 1.0f));
      }
      break;
    }
    case bnl::ShapeDesc::kCompound: {
      btCompoundShape* c = new btCompoundShape();
      for (size_t i = 0; i < s.children.size(); ++i) {
        btTransform t;
        t.setIdentity();
        t.setOrigin(toBt(s.children[i].xform.p));
        t.setRotation(btQuaternion(s.children[i].xform.q.x, s.children[i].xform.q.y, s.children[i].xform.q.z, s.children[i].xform.q.w));
        c->addChildShape(t, makeRbShape(s.children[i].shape, dynamicBody));
      }
      shape = c;
      break;
    }
    default:
      return makeShape(s);  // primitives: the same shapes, the same margin
  }
  shape->setMargin(s.margin);  // RB_shape_set_margin
  if (shape->getShapeType() == GIMPACT_SHAPE_PROXYTYPE) static_cast<btGImpactMeshShape*>(shape)->updateBound();
  return shape;
}

}  // namespace

btCollisionShape* makeShape(const bnl::ShapeDesc& s) {
  switch (s.type) {
    case bnl::ShapeDesc::kBox: return makeBox(toBt(s.half), s.margin);
    case bnl::ShapeDesc::kSphere: return makeSphere(s.radius, s.margin);
    case bnl::ShapeDesc::kCylinderZ: return makeCylinderZ(toBt(s.half), s.margin);
    case bnl::ShapeDesc::kConeZ: return makeConeZ(s.radius, s.height, s.margin);
    case bnl::ShapeDesc::kCapsuleZ: return makeCapsuleZ(s.radius, s.height, s.margin);
    case bnl::ShapeDesc::kHull: {
      std::vector<btVector3> p;
      for (size_t i = 0; i < s.points.size(); ++i) p.push_back(toBt(s.points[i]));
      return makeHull(p, s.margin);
    }
    case bnl::ShapeDesc::kMesh: {
      std::vector<btVector3> v; std::vector<int> t;
      meshOf(s.mesh, v, t);
      return makeMesh(v, t, s.dynamicMesh, toBt(s.scale), s.margin);
    }
    case bnl::ShapeDesc::kCompound: {
      btCompoundShape* c = new btCompoundShape();
      for (size_t i = 0; i < s.children.size(); ++i)
        c->addChildShape(toBt(s.children[i].xform), makeShape(s.children[i].shape));
      return c;
    }
  }
  return 0;
}

void build(World& w, const bnl::SceneDesc& s) {
  std::vector<int> bodyIndex(s.bodies.size(), -1);
  for (size_t n = 0; n < s.objects.size(); ++n) {
    const bnl::ObjectDesc& ob = s.objects[n];
    switch (ob.type) {
      case bnl::ObjectDesc::kBody: {
        const bnl::BodyDesc& d = s.bodies[ob.index];
        BodyOpts o = optsOf(d);
        if (d.kind == bnl::kSoft) {
          std::vector<btVector3> v; std::vector<int> t;
          meshOf(d.softMesh, v, t);
          bodyIndex[ob.index] = w.addSoftFromTriMesh(v, t, o, d.bending, d.shapeMatching);
        } else if (d.kind == bnl::kCharacter) {
          o.shape = makeShape(d.shape);
          bodyIndex[ob.index] = w.addCharacter(o, d.stepHeight, d.jumpSpeed, d.fallSpeed);
        } else if (s.profile == "blender53-rb") {
          o.shape = makeRbShape(d.shape, (d.kind == bnl::kRigid || d.kind == bnl::kDynamic) && !d.animated);
          bodyIndex[ob.index] = w.add(o);
        } else {
          o.shape = makeShape(d.shape);
          bodyIndex[ob.index] = w.add(o);
        }
        break;
      }
      case bnl::ObjectDesc::kConstraint: {
        const bnl::ConstraintDesc& c = s.constraints[ob.index];
        const int a = bodyIndex[c.a], b = c.b < 0 ? -1 : bodyIndex[c.b];
        if (c.type >= bnl::ConstraintDesc::kRbFixed) {
          RbJoint j;
          j.type = c.type; j.a = a; j.b = b;
          j.frame = toBtRb(c.frame);
          j.enabled = c.enabled; j.disableCollisions = c.disableCollisions;
          j.breakingThreshold = c.breakingThreshold; j.iterations = c.iterations;
          for (int k = 0; k < 6; ++k) {
            j.useLimit[k] = c.useLimit[k]; j.lower[k] = c.lower[k]; j.upper[k] = c.upper[k];
            j.useSpring[k] = c.useSpring[k]; j.stiffness[k] = c.stiffness[k]; j.damping[k] = c.damping[k];
          }
          j.springType = c.springType;
          j.useMotorLin = c.useMotorLin; j.useMotorAng = c.useMotorAng;
          j.motorLinVelocity = c.motorLinVelocity; j.motorAngVelocity = c.motorAngVelocity;
          j.motorLinMaxImpulse = c.motorLinMaxImpulse; j.motorAngMaxImpulse = c.motorAngMaxImpulse;
          w.addRbJoint(j);
          break;
        }
        switch (c.type) {
          case bnl::ConstraintDesc::kPointToPoint: w.addPointToPoint(a, b, toBt(c.pivotA), toBt(c.pivotB)); break;
          case bnl::ConstraintDesc::kHinge: w.addHinge(a, b, toBt(c.frameA), toBt(c.frameB), c.angularOnly); break;
          case bnl::ConstraintDesc::kConeTwist: w.addConeTwist(a, b, toBt(c.frameA), toBt(c.frameB)); break;
          case bnl::ConstraintDesc::kGeneric6DofSpring: w.addGeneric6DofSpring(a, b, toBt(c.frameA), toBt(c.frameB)); break;
          default: break;
        }
        break;
      }
      case bnl::ObjectDesc::kParam: {
        const bnl::ParamDesc& p = s.params[ob.index];
        w.setParam(p.constraint, p.index, p.v0, p.v1);
        break;
      }
      case bnl::ObjectDesc::kVehicle:
        w.addVehicle(bodyIndex[s.vehicles[ob.index].chassis]);
        break;
      case bnl::ObjectDesc::kWheel: {
        const bnl::WheelDesc& d = s.wheels[ob.index];
        w.addWheel(d.vehicle, toBt(d.pos), toBt(d.down), toBt(d.axle), d.restLength, d.radius, d.steering);
        if (d.tuning.set) {
          btRaycastVehicle* rv = w.vehicle(d.vehicle);
          btWheelInfo& wi = rv->getWheelInfo(rv->getNumWheels() - 1);
          wi.m_suspensionStiffness = d.tuning.suspensionStiffness;
          wi.m_wheelsDampingCompression = d.tuning.dampingCompression;
          wi.m_wheelsDampingRelaxation = d.tuning.dampingRelaxation;
          wi.m_frictionSlip = d.tuning.frictionSlip;
          wi.m_rollInfluence = d.tuning.rollInfluence;
        }
        break;
      }
    }
  }
  for (size_t i = 0; i < s.bodies.size(); ++i) {
    const bnl::BodyDesc& d = s.bodies[i];
    BodyRec& r = w.body(bodyIndex[i]);
    if (!r.rb) continue;
    if (d.hasLinVel) r.rb->setLinearVelocity(toBt(d.linVel));
    if (d.hasAngVel) r.rb->setAngularVelocity(toBt(d.angVel));
    if (d.noSleep) r.rb->setActivationState(DISABLE_DEACTIVATION);
  }
}

void act(World& w, const bnl::SceneDesc& s, int f) {
  // Vehicle controls: every vehicle named by an action gets all three channels each frame,
  // zero unless an action covering this frame sets them.
  std::vector<int> driven;
  for (size_t i = 0; i < s.actions.size(); ++i)
    if (s.actions[i].op == bnl::ActionDesc::kVehicle) {
      bool seen = false;
      for (size_t k = 0; k < driven.size(); ++k) seen = seen || driven[k] == s.actions[i].target;
      if (!seen) driven.push_back(s.actions[i].target);
    }
  for (size_t k = 0; k < driven.size(); ++k) {
    btRaycastVehicle* rv = w.vehicle(driven[k]);
    const int n = rv->getNumWheels();
    std::vector<float> engine(n, 0.0f), brake(n, 0.0f), steer(n, 0.0f);
    for (size_t i = 0; i < s.actions.size(); ++i) {
      const bnl::ActionDesc& a = s.actions[i];
      if (a.op != bnl::ActionDesc::kVehicle || a.target != driven[k]) continue;
      if (f < a.from || (a.to >= 0 && f > a.to)) continue;
      for (int j = 0; j < n; ++j) {
        if (j < (int)a.engine.size()) engine[j] = a.engine[j];
        if (j < (int)a.brake.size()) brake[j] = a.brake[j];
        if (j < (int)a.steer.size()) steer[j] = a.steer[j];
      }
    }
    for (int j = 0; j < n; ++j) {
      rv->applyEngineForce(engine[j], j);
      rv->setBrake(brake[j], j);
      if (rv->getWheelInfo(j).m_bIsFrontWheel) rv->setSteeringValue(steer[j], j);
    }
  }
  for (size_t i = 0; i < s.actions.size(); ++i) {
    const bnl::ActionDesc& a = s.actions[i];
    if (a.op == bnl::ActionDesc::kVehicle) continue;
    if (f < a.from || (a.to >= 0 && f > a.to)) continue;
    BodyRec& r = w.body(a.target);
    switch (a.op) {
      case bnl::ActionDesc::kWalk: if (r.character) r.character->setWalkDirection(toBt(a.v)); break;
      case bnl::ActionDesc::kJump: if (r.character) r.character->jump(); break;
      case bnl::ActionDesc::kSuspend: w.suspendDynamics(a.target); break;
      case bnl::ActionDesc::kKinematicMove: w.moveKinematic(a.target, toBt(a.v)); break;
      case bnl::ActionDesc::kLinVel: if (r.rb) { r.rb->setLinearVelocity(toBt(a.v)); r.rb->activate(true); } break;
      case bnl::ActionDesc::kImpulse: if (r.rb) { r.rb->applyCentralImpulse(toBt(a.v)); r.rb->activate(true); } break;
      default: break;
    }
  }
}

BN_NAMESPACE_END  // namespace bn
