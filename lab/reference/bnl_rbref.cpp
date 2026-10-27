// Bullet NEXT Lab - reference for the host profile blender53-rb (FL, TL.5).
// Runs the Lab's Blender rigid body scenes through Blender 5.3's own rb_bullet_api.cpp (built
// with Blender's Bullet copy, double precision), driven as BKE_rigidbody does: bodies
// (rigidbody_validate_sim_object), constraints (rigidbody_validate_sim_constraint), then per frame
// the world settings and the substeps with keyframed bodies interpolated
// (BKE_rigidbody_do_simulation). Prints the same hashes as the harness runners, so the Lab's own
// profile (harness/src/bn_world.cpp, World::addRb / addRbJoint / stepRb) can be checked against it.
//   bnl_rbref [--scene NAME|all] [--scenes DIR]
#include "RBI_api.h"
#include "bn_rbmath.h"
#include "bnl_scene.h"

#include <cfloat>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#ifndef BN_SCENES_DIR
#define BN_SCENES_DIR "lab/scenes"
#endif

namespace {

typedef std::uint64_t u64;

struct Hash {
  u64 h = 1469598103934665603ULL;
  void bytes(const void* p, size_t n) {
    const unsigned char* c = static_cast<const unsigned char*>(p);
    for (size_t i = 0; i < n; ++i) { h ^= c[i]; h *= 1099511628211ULL; }
  }
};

void quatWxyz(const bnl::Quat& q, float r[4]) { r[0] = q.w; r[1] = q.x; r[2] = q.y; r[3] = q.z; }

rbCollisionShape* makeShape(const bnl::ShapeDesc& s, bool dynamicBody) {
  rbCollisionShape* shape = nullptr;
  switch (s.type) {
    case bnl::ShapeDesc::kBox: shape = RB_shape_new_box(s.half.x, s.half.y, s.half.z); break;
    case bnl::ShapeDesc::kSphere: shape = RB_shape_new_sphere(s.radius); break;
    case bnl::ShapeDesc::kCylinderZ: shape = RB_shape_new_cylinder(s.half.x, s.half.z); break;
    case bnl::ShapeDesc::kConeZ: shape = RB_shape_new_cone(s.radius, s.height); break;
    case bnl::ShapeDesc::kCapsuleZ: shape = RB_shape_new_capsule(s.radius, s.height); break;
    case bnl::ShapeDesc::kHull: {
      std::vector<float> p;
      for (const auto& v : s.points) { p.push_back(v.x); p.push_back(v.y); p.push_back(v.z); }
      bool embed = true;
      shape = RB_shape_new_convex_hull(p.data(), 3 * sizeof(float), (int)s.points.size(), s.margin, &embed);
      break;
    }
    case bnl::ShapeDesc::kMesh: {
      std::vector<bnl::Vec3> v;
      std::vector<int> t;
      bnl::meshData(s.mesh, v, t);
      rbMeshData* m = RB_trimesh_data_new((int)t.size() / 3, (int)v.size());
      std::vector<float> f;
      for (const auto& p : v) { f.push_back(p.x); f.push_back(p.y); f.push_back(p.z); }
      RB_trimesh_add_vertices(m, f.data(), (int)v.size(), 3 * sizeof(float));
      for (size_t i = 0; i + 2 < t.size(); i += 3) RB_trimesh_add_triangle_indices(m, (int)i / 3, t[i], t[i + 1], t[i + 2]);
      RB_trimesh_finish(m);
      shape = dynamicBody ? RB_shape_new_gimpact_mesh(m) : RB_shape_new_trimesh(m);
      break;
    }
    case bnl::ShapeDesc::kCompound: {
      shape = RB_shape_new_compound();
      for (const auto& c : s.children) {
        rbCollisionShape* child = makeShape(c.shape, dynamicBody);
        const float loc[3] = {c.xform.p.x, c.xform.p.y, c.xform.p.z};
        float rot[4];
        quatWxyz(c.xform.q, rot);
        RB_compound_add_child_shape(shape, child, loc, rot);
      }
      break;
    }
  }
  RB_shape_set_margin(shape, s.margin);
  return shape;
}

struct Target {
  int body;
  float from[3], to[3], rot[4];
};

u64 run(const bnl::SceneDesc& sc, int& framesOut) {
  const float g[3] = {0, 0, sc.settings.gravityZ};
  rbDynamicsWorld* world = RB_dworld_new(g);
  std::vector<rbRigidBody*> bodies(sc.bodies.size(), nullptr);
  for (const auto& ob : sc.objects) {
    if (ob.type == bnl::ObjectDesc::kBody) {
      const bnl::BodyDesc& d = sc.bodies[ob.index];
      const bool active = (d.kind == bnl::kRigid || d.kind == bnl::kDynamic) && !d.animated;
      rbCollisionShape* shape = makeShape(d.shape, active);
      const float loc[3] = {d.xform.p.x, d.xform.p.y, d.xform.p.z};
      float rot[4];
      quatWxyz(d.xform.q, rot);
      rbRigidBody* b = RB_body_new(shape, loc, rot);
      RB_body_set_friction(b, d.friction);
      RB_body_set_restitution(b, d.restitution);
      RB_body_set_damping(b, d.linDamp, d.angDamp);
      RB_body_set_sleep_thresh(b, d.linSleep, d.angSleep);
      RB_body_set_activation_state(b, d.useDeactivation);
      if (!active || d.startDeactivated) RB_body_deactivate(b);
      RB_body_set_linear_factor(b, d.linFactor.x, d.linFactor.y, d.linFactor.z);
      RB_body_set_angular_factor(b, d.angFactor.x, d.angFactor.y, d.angFactor.z);
      RB_body_set_mass(b, active ? d.mass : 0.0f);
      RB_body_set_kinematic_state(b, d.animated);
      RB_dworld_add_body(world, b, d.colGroups);
      bodies[ob.index] = b;
    } else if (ob.type == bnl::ObjectDesc::kConstraint) {
      const bnl::ConstraintDesc& c = sc.constraints[ob.index];
      rbRigidBody* rb1 = bodies[c.a];
      rbRigidBody* rb2 = bodies[c.b];
      float loc[3] = {c.frame.p.x, c.frame.p.y, c.frame.p.z};
      float rot[4];
      quatWxyz(c.frame.q, rot);
      rbConstraint* con = nullptr;
      auto limits = [&](void (*set)(rbConstraint*, int, float, float)) {
        for (int k = 0; k < 6; ++k) set(con, k, c.useLimit[k] ? c.lower[k] : 0.0f, c.useLimit[k] ? c.upper[k] : -1.0f);
      };
      switch (c.type) {
        case bnl::ConstraintDesc::kRbPoint: con = RB_constraint_new_point(loc, rb1, rb2); break;
        case bnl::ConstraintDesc::kRbFixed: con = RB_constraint_new_fixed(loc, rot, rb1, rb2); break;
        case bnl::ConstraintDesc::kRbHinge:
          con = RB_constraint_new_hinge(loc, rot, rb1, rb2);
          RB_constraint_set_limits_hinge(con, c.useLimit[5] ? c.lower[5] : 0.0f, c.useLimit[5] ? c.upper[5] : -1.0f);
          break;
        case bnl::ConstraintDesc::kRbSlider:
          con = RB_constraint_new_slider(loc, rot, rb1, rb2);
          RB_constraint_set_limits_slider(con, c.useLimit[0] ? c.lower[0] : 0.0f, c.useLimit[0] ? c.upper[0] : -1.0f);
          break;
        case bnl::ConstraintDesc::kRbPiston:
          con = RB_constraint_new_piston(loc, rot, rb1, rb2);
          RB_constraint_set_limits_piston(con, c.useLimit[0] ? c.lower[0] : 0.0f, c.useLimit[0] ? c.upper[0] : -1.0f,
                                          c.useLimit[3] ? c.lower[3] : 0.0f, c.useLimit[3] ? c.upper[3] : -1.0f);
          break;
        case bnl::ConstraintDesc::kRbGenericSpring:
          if (c.springType == 2) {
            con = RB_constraint_new_6dof_spring2(loc, rot, rb1, rb2);
            for (int k = 0; k < 6; ++k) {
              RB_constraint_set_spring_6dof_spring2(con, k, c.useSpring[k]);
              RB_constraint_set_stiffness_6dof_spring2(con, k, c.stiffness[k]);
              RB_constraint_set_damping_6dof_spring2(con, k, c.damping[k]);
            }
            RB_constraint_set_equilibrium_6dof_spring2(con);
            limits(RB_constraint_set_limits_6dof_spring2);
          } else {
            con = RB_constraint_new_6dof_spring(loc, rot, rb1, rb2);
            for (int k = 0; k < 6; ++k) {
              RB_constraint_set_spring_6dof_spring(con, k, c.useSpring[k]);
              RB_constraint_set_stiffness_6dof_spring(con, k, c.stiffness[k]);
              RB_constraint_set_damping_6dof_spring(con, k, c.damping[k]);
            }
            RB_constraint_set_equilibrium_6dof_spring(con);
            limits(RB_constraint_set_limits_6dof);
          }
          break;
        case bnl::ConstraintDesc::kRbGeneric:
          con = RB_constraint_new_6dof(loc, rot, rb1, rb2);
          limits(RB_constraint_set_limits_6dof);
          break;
        case bnl::ConstraintDesc::kRbMotor:
          con = RB_constraint_new_motor(loc, rot, rb1, rb2);
          RB_constraint_set_enable_motor(con, c.useMotorLin, c.useMotorAng);
          RB_constraint_set_max_impulse_motor(con, c.motorLinMaxImpulse, c.motorAngMaxImpulse);
          RB_constraint_set_target_velocity_motor(con, c.motorLinVelocity, c.motorAngVelocity);
          break;
        default: break;
      }
      if (!con) continue;
      RB_constraint_set_enabled(con, c.enabled);
      RB_constraint_set_breaking_threshold(con, c.breakingThreshold >= 0 ? c.breakingThreshold : FLT_MAX);
      RB_constraint_set_solver_iterations(con, c.iterations);
      RB_dworld_add_constraint(world, con, c.disableCollisions);
    }
  }
  u64 final = 0;
  std::vector<Target> targets;
  for (int f = 0; f < sc.frames; ++f) {
    // The scene's keyframed motion for this frame (the Lab's kinematic_move actions).
    targets.clear();
    for (const auto& a : sc.actions) {
      if (a.op != bnl::ActionDesc::kKinematicMove || f < a.from || (a.to >= 0 && f > a.to)) continue;
      const float d[3] = {a.v.x, a.v.y, a.v.z};
      bool found = false;
      for (auto& t : targets)
        if (t.body == a.target) { for (int k = 0; k < 3; ++k) t.to[k] += d[k]; found = true; }
      if (found) continue;
      Target t;
      t.body = a.target;
      RB_body_get_position(bodies[a.target], t.from);
      RB_body_get_orientation(bodies[a.target], t.rot);
      for (int k = 0; k < 3; ++k) t.to[k] = t.from[k] + d[k];
      targets.push_back(t);
    }
    // BKE_rigidbody_do_simulation, one frame
    const float timestep = 1.0f / sc.settings.ticrate * 1.0f * sc.settings.timeScale;
    const float substep = timestep / float(sc.settings.substeps);
    const float interpStep = 1.0f / float(sc.settings.substeps);
    float interp = interpStep;
    RB_dworld_set_gravity(world, g);
    if (sc.settings.iterations > 0) RB_dworld_set_solver_iterations(world, sc.settings.iterations);
    RB_dworld_set_split_impulse(world, sc.settings.splitImpulse);
    for (int i = 0; i < sc.settings.substeps; ++i) {
      for (const auto& t : targets) {
        float loc[3], rot[4];
        bnrb::interpV3(loc, t.from, t.to, interp);
        bnrb::interpQt(rot, t.rot, t.rot, interp);
        RB_body_activate(bodies[t.body]);
        RB_body_set_loc_rot(bodies[t.body], loc, rot);
      }
      RB_dworld_step_simulation(world, substep, 0, substep);
      interp += interpStep;
    }
    // The harness recorder's frame hash: 13 floats per body, then the event count (none here).
    Hash h;
    for (rbRigidBody* b : bodies) {
      float p[3], q[4], lv[3], av[3];
      RB_body_get_position(b, p);
      RB_body_get_orientation(b, q);
      RB_body_get_linear_velocity(b, lv);
      RB_body_get_angular_velocity(b, av);
      const float s[13] = {p[0], p[1], p[2], q[1], q[2], q[3], q[0], lv[0], lv[1], lv[2], av[0], av[1], av[2]};
      h.bytes(s, sizeof(s));
    }
    const int events = 0;
    h.bytes(&events, sizeof(events));
    Hash fin;
    fin.h = final;
    fin.bytes(&h.h, sizeof(h.h));
    final = fin.h;
  }
  framesOut = sc.frames;
  return final;
}

}  // namespace

int main(int argc, char** argv) {
  std::string which = "all", dir = BN_SCENES_DIR;
  for (int i = 1; i + 1 < argc; i += 2) {
    if (!std::strcmp(argv[i], "--scene")) which = argv[i + 1];
    else if (!std::strcmp(argv[i], "--scenes")) dir = argv[i + 1];
  }
  for (const std::string& name : bnl::listScenes(dir)) {
    if (which != "all" && which != name) continue;
    bnl::SceneDesc sc;
    std::string err;
    if (!bnl::loadScene(dir + "/" + name + ".json", sc, err)) { std::fprintf(stderr, "%s\n", err.c_str()); return 2; }
    if (sc.profile != "blender53-rb") continue;
    int frames = 0;
    const u64 h = run(sc, frames);
    std::printf("%-18s rbref frames %4d hash %016llx\n", name.c_str(), frames, (unsigned long long)h);
  }
  return 0;
}
