// Bullet NEXT Lab - scene description and scene files (FL, CL.5).
#include "bnl_scene.h"

#include "nlohmann/json.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace bnl {

using json = nlohmann::json;

// --- building ---------------------------------------------------------------------------------

int SceneDesc::addBody(const BodyDesc& b) {
  bodies.push_back(b);
  ObjectDesc o; o.type = ObjectDesc::kBody; o.index = (int)bodies.size() - 1;
  objects.push_back(o);
  return o.index;
}

int SceneDesc::addConstraint(const ConstraintDesc& c) {
  constraints.push_back(c);
  ObjectDesc o; o.type = ObjectDesc::kConstraint; o.index = (int)constraints.size() - 1;
  objects.push_back(o);
  return o.index;
}

void SceneDesc::addParam(int constraint, int index, float v0, float v1) {
  ParamDesc p; p.constraint = constraint; p.index = index; p.v0 = v0; p.v1 = v1;
  params.push_back(p);
  ObjectDesc o; o.type = ObjectDesc::kParam; o.index = (int)params.size() - 1;
  objects.push_back(o);
}

int SceneDesc::addVehicle(int chassis) {
  VehicleDesc v; v.chassis = chassis;
  vehicles.push_back(v);
  ObjectDesc o; o.type = ObjectDesc::kVehicle; o.index = (int)vehicles.size() - 1;
  objects.push_back(o);
  return o.index;
}

int SceneDesc::addWheel(const WheelDesc& w) {
  wheels.push_back(w);
  ObjectDesc o; o.type = ObjectDesc::kWheel; o.index = (int)wheels.size() - 1;
  objects.push_back(o);
  return o.index;
}

// --- mesh generators (same float expressions as the F0 harness) ----------------------------------

void gridMesh(int nx, int ny, float size, std::vector<Vec3>& v, std::vector<int>& t) {
  v.clear(); t.clear();
  for (int j = 0; j <= ny; ++j)
    for (int i = 0; i <= nx; ++i)
      v.push_back(Vec3(-size + 2 * size * i / nx, -size + 2 * size * j / ny, 0));
  for (int j = 0; j < ny; ++j)
    for (int i = 0; i < nx; ++i) {
      int a = j * (nx + 1) + i, b = a + 1, c = a + nx + 1, d = c + 1;
      t.push_back(a); t.push_back(b); t.push_back(d);
      t.push_back(a); t.push_back(d); t.push_back(c);
    }
}

void boxMesh(const Vec3& h, std::vector<Vec3>& v, std::vector<int>& t) {
  v.clear(); t.clear();
  for (int i = 0; i < 8; ++i)
    v.push_back(Vec3(i & 1 ? h.x : -h.x, i & 2 ? h.y : -h.y, i & 4 ? h.z : -h.z));
  const int f[36] = {0, 2, 1, 1, 2, 3, 4, 5, 6, 5, 7, 6, 0, 1, 4, 1, 5, 4,
                     2, 6, 3, 3, 6, 7, 0, 4, 2, 2, 4, 6, 1, 3, 5, 3, 7, 5};
  t.assign(f, f + 36);
}

void meshData(const MeshDesc& m, std::vector<Vec3>& v, std::vector<int>& t) {
  switch (m.gen) {
    case MeshDesc::kGrid: gridMesh(m.nx, m.ny, m.size, v, t); break;
    case MeshDesc::kBox: boxMesh(m.half, v, t); break;
    default: v = m.verts; t = m.tris; break;
  }
}

// --- JSON ---------------------------------------------------------------------------------------
// Floats are written as the shortest decimal that reads back to the same float, so a scene file
// rebuilds bit for bit what it was written from.

namespace {

// The shortest decimal that reads back (through a double, as the reader does) to the same float;
// if no short form survives that path, the float's exact double value.
json jf(float f) {
  char buf[32];
  for (int prec = 6; prec <= 9; ++prec) {
    std::snprintf(buf, sizeof(buf), "%.*g", prec, (double)f);
    const double d = std::strtod(buf, 0);
    if ((float)d == f && std::signbit(d) == std::signbit(f)) return json(d);
  }
  return json((double)f);
}
json jv(const Vec3& v) { return json::array({jf(v.x), jf(v.y), jf(v.z)}); }
json jq(const Quat& q) { return json::array({jf(q.x), jf(q.y), jf(q.z), jf(q.w)}); }

bool isIdentity(const Quat& q) { return q.x == 0 && q.y == 0 && q.z == 0 && q.w == 1; }

json jx(const Xform& x) {
  json o = json::object();
  o["p"] = jv(x.p);
  if (!isIdentity(x.q)) o["q"] = jq(x.q);
  return o;
}

json jmesh(const MeshDesc& m) {
  json o = json::object();
  if (m.gen == MeshDesc::kGrid) { o["grid"] = json::array({m.nx, m.ny}); o["size"] = jf(m.size); }
  else if (m.gen == MeshDesc::kBox) { o["box"] = jv(m.half); }
  else {
    json v = json::array();
    for (size_t i = 0; i < m.verts.size(); ++i) { v.push_back(jf(m.verts[i].x)); v.push_back(jf(m.verts[i].y)); v.push_back(jf(m.verts[i].z)); }
    o["verts"] = v;
    o["tris"] = m.tris;
  }
  return o;
}

const char* kShapeNames[] = {"box", "sphere", "cylinder_z", "cone_z", "capsule_z", "hull", "mesh", "compound"};
const char* kKindNames[] = {"static", "dynamic", "rigid", "sensor", "character", "soft"};
const char* kConstraintNames[] = {"point_to_point", "hinge", "cone_twist", "generic_6dof_spring",
                                  "rb_fixed", "rb_point", "rb_hinge", "rb_slider", "rb_piston", "rb_generic",
                                  "rb_generic_spring", "rb_motor"};
const int kConstraintCount = 12;
const char* kAxisNames[] = {"lin_x", "lin_y", "lin_z", "ang_x", "ang_y", "ang_z"};
const char* kOpNames[] = {"vehicle", "walk", "jump", "suspend", "kinematic_move", "lin_vel", "impulse"};

json jshape(const ShapeDesc& s) {
  json o = json::object();
  o["shape"] = kShapeNames[s.type];
  switch (s.type) {
    case ShapeDesc::kBox: case ShapeDesc::kCylinderZ: o["half"] = jv(s.half); break;
    case ShapeDesc::kSphere: o["radius"] = jf(s.radius); break;
    case ShapeDesc::kConeZ: case ShapeDesc::kCapsuleZ: o["radius"] = jf(s.radius); o["height"] = jf(s.height); break;
    case ShapeDesc::kHull: {
      json p = json::array();
      for (size_t i = 0; i < s.points.size(); ++i) p.push_back(jv(s.points[i]));
      o["points"] = p;
      break;
    }
    case ShapeDesc::kMesh:
      o["mesh"] = jmesh(s.mesh);
      if (s.dynamicMesh) o["dynamic"] = true;
      if (s.scale.x != 1 || s.scale.y != 1 || s.scale.z != 1) o["scale"] = jv(s.scale);
      break;
    case ShapeDesc::kCompound: {
      json c = json::array();
      for (size_t i = 0; i < s.children.size(); ++i) {
        json ch = jshape(s.children[i].shape);
        ch["at"] = jx(s.children[i].xform);
        c.push_back(ch);
      }
      o["children"] = c;
      break;
    }
  }
  if (s.margin != 0.04f && s.type != ShapeDesc::kCompound) o["margin"] = jf(s.margin);
  return o;
}

json jbody(const BodyDesc& b) {
  const BodyDesc d;
  json o = json::object();
  o["type"] = kKindNames[b.kind] ;
  if (!b.name.empty()) o["name"] = b.name;
  if (b.kind != kSoft) {
    json s = jshape(b.shape);
    for (auto it = s.begin(); it != s.end(); ++it) o[it.key()] = it.value();
  }
  o["at"] = jx(b.xform);
  if (b.mass != d.mass) o["mass"] = jf(b.mass);
  if (b.friction != d.friction) o["friction"] = jf(b.friction);
  if (b.restitution != d.restitution) o["restitution"] = jf(b.restitution);
  if (b.linDamp != d.linDamp) o["lin_damp"] = jf(b.linDamp);
  if (b.angDamp != d.angDamp) o["ang_damp"] = jf(b.angDamp);
  if (b.inertia != d.inertia) o["inertia"] = jf(b.inertia);
  if (b.linFactor.x != 1 || b.linFactor.y != 1 || b.linFactor.z != 1) o["lin_factor"] = jv(b.linFactor);
  if (b.angFactor.x != 1 || b.angFactor.y != 1 || b.angFactor.z != 1) o["ang_factor"] = jv(b.angFactor);
  if (b.ghost) o["ghost"] = true;
  if (b.userGroup != 0xffff) o["user_group"] = b.userGroup;
  if (b.userMask != 0xffff) o["user_mask"] = b.userMask;
  if (b.clampVelMin) o["clamp_vel_min"] = jf(b.clampVelMin);
  if (b.clampVelMax) o["clamp_vel_max"] = jf(b.clampVelMax);
  if (b.clampAngMin) o["clamp_ang_min"] = jf(b.clampAngMin);
  if (b.clampAngMax) o["clamp_ang_max"] = jf(b.clampAngMax);
  if (b.reportCollisions) o["report"] = true;
  if (b.hasLinVel) o["lin_vel"] = jv(b.linVel);
  if (b.hasAngVel) o["ang_vel"] = jv(b.angVel);
  if (b.noSleep) o["no_sleep"] = true;
  if (b.animated) o["animated"] = true;
  if (b.useDeactivation) o["use_deactivation"] = true;
  if (b.startDeactivated) o["start_deactivated"] = true;
  if (b.linSleep != d.linSleep) o["sleep_lin"] = jf(b.linSleep);
  if (b.angSleep != d.angSleep) o["sleep_ang"] = jf(b.angSleep);
  if (b.colGroups != d.colGroups) o["col_groups"] = b.colGroups;
  if (b.kind == kCharacter) {
    o["step_height"] = jf(b.stepHeight); o["jump_speed"] = jf(b.jumpSpeed); o["fall_speed"] = jf(b.fallSpeed);
  }
  if (b.kind == kSoft) {
    o["mesh"] = jmesh(b.softMesh);
    if (b.bending) o["bending"] = true;
    if (b.shapeMatching) o["shape_matching"] = true;
  }
  return o;
}

json jconstraint(const ConstraintDesc& c) {
  json o = json::object();
  o["type"] = kConstraintNames[c.type];
  o["a"] = c.a;
  o["b"] = c.b;
  if (c.type >= ConstraintDesc::kRbFixed) {
    const ConstraintDesc d;
    o["frame"] = jx(c.frame);
    if (!c.enabled) o["enabled"] = false;
    if (!c.disableCollisions) o["disable_collisions"] = false;
    if (c.breakingThreshold >= 0) o["breaking_threshold"] = jf(c.breakingThreshold);
    if (c.iterations >= 0) o["iterations"] = c.iterations;
    json lim = json::object(), spr = json::object();
    for (int k = 0; k < 6; ++k) {
      if (c.useLimit[k]) lim[kAxisNames[k]] = json::array({jf(c.lower[k]), jf(c.upper[k])});
      if (c.useSpring[k]) spr[kAxisNames[k]] = json::array({jf(c.stiffness[k]), jf(c.damping[k])});
    }
    if (!lim.empty()) o["limits"] = lim;
    if (!spr.empty()) { o["springs"] = spr; o["spring_type"] = c.springType; }
    if (c.type == ConstraintDesc::kRbMotor) {
      if (c.useMotorLin) o["motor_lin"] = json::array({jf(c.motorLinVelocity), jf(c.motorLinMaxImpulse)});
      if (c.useMotorAng) o["motor_ang"] = json::array({jf(c.motorAngVelocity), jf(c.motorAngMaxImpulse)});
    }
    return o;
  }
  if (c.type == ConstraintDesc::kPointToPoint) { o["pivot_a"] = jv(c.pivotA); o["pivot_b"] = jv(c.pivotB); }
  else { o["frame_a"] = jx(c.frameA); o["frame_b"] = jx(c.frameB); }
  if (c.angularOnly) o["angular_only"] = true;
  return o;
}

json jfloats(const std::vector<float>& v) {
  json a = json::array();
  for (size_t i = 0; i < v.size(); ++i) a.push_back(jf(v[i]));
  return a;
}

json jaction(const ActionDesc& a) {
  json o = json::object();
  o["op"] = kOpNames[a.op];
  if (a.from == a.to) o["at"] = a.from;
  else { o["from"] = a.from; o["to"] = a.to; }
  o[a.op == ActionDesc::kVehicle ? "vehicle" : "body"] = a.target;
  if (a.op == ActionDesc::kVehicle) {
    if (!a.engine.empty()) o["engine"] = jfloats(a.engine);
    if (!a.brake.empty()) o["brake"] = jfloats(a.brake);
    if (!a.steer.empty()) o["steer"] = jfloats(a.steer);
  } else if (a.op != ActionDesc::kJump && a.op != ActionDesc::kSuspend) {
    o["v"] = jv(a.v);
  }
  return o;
}

// --- reading ----------------------------------------------------------------------------------

struct Reader {
  std::string err;
  bool ok() const { return err.empty(); }
  void fail(const std::string& m) { if (err.empty()) err = m; }

  float f(const json& o, const char* k, float def) {
    auto it = o.find(k);
    if (it == o.end()) return def;
    if (!it->is_number()) { fail(std::string("'") + k + "' must be a number"); return def; }
    return (float)it->get<double>();
  }
  int i(const json& o, const char* k, int def) {
    auto it = o.find(k);
    if (it == o.end()) return def;
    if (!it->is_number_integer()) { fail(std::string("'") + k + "' must be an integer"); return def; }
    return it->get<int>();
  }
  bool b(const json& o, const char* k, bool def) {
    auto it = o.find(k);
    if (it == o.end()) return def;
    if (!it->is_boolean()) { fail(std::string("'") + k + "' must be true or false"); return def; }
    return it->get<bool>();
  }
  Vec3 v3(const json& a) {
    if (!a.is_array() || a.size() != 3) { fail("expected [x, y, z]"); return Vec3(); }
    return Vec3((float)a[0].get<double>(), (float)a[1].get<double>(), (float)a[2].get<double>());
  }
  Vec3 v(const json& o, const char* k, const Vec3& def) {
    auto it = o.find(k);
    return it == o.end() ? def : v3(*it);
  }
  Quat q4(const json& a) {
    if (!a.is_array() || a.size() != 4) { fail("expected [x, y, z, w]"); return Quat(); }
    return Quat((float)a[0].get<double>(), (float)a[1].get<double>(), (float)a[2].get<double>(), (float)a[3].get<double>());
  }
  Xform x(const json& o, const char* k) {
    Xform r;
    auto it = o.find(k);
    if (it == o.end()) return r;
    r.p = v(*it, "p", Vec3());
    auto q = it->find("q");
    if (q != it->end()) r.q = q4(*q);
    return r;
  }
  std::vector<float> fl(const json& o, const char* k) {
    std::vector<float> r;
    auto it = o.find(k);
    if (it == o.end()) return r;
    for (size_t n = 0; n < it->size(); ++n) r.push_back((float)(*it)[n].get<double>());
    return r;
  }
  int enumOf(const std::string& s, const char* const* names, int count, const char* what) {
    for (int n = 0; n < count; ++n) if (s == names[n]) return n;
    fail(std::string("unknown ") + what + " '" + s + "'");
    return 0;
  }

  MeshDesc mesh(const json& o) {
    MeshDesc m;
    if (o.contains("grid")) {
      m.gen = MeshDesc::kGrid;
      m.nx = o["grid"][0].get<int>(); m.ny = o["grid"][1].get<int>();
      m.size = f(o, "size", 0);
    } else if (o.contains("box")) {
      m.gen = MeshDesc::kBox;
      m.half = v3(o["box"]);
    } else {
      std::vector<float> fv = fl(o, "verts");
      for (size_t n = 0; n + 2 < fv.size(); n += 3) m.verts.push_back(Vec3(fv[n], fv[n + 1], fv[n + 2]));
      if (o.contains("tris")) m.tris = o["tris"].get<std::vector<int> >();
    }
    return m;
  }

  ShapeDesc shape(const json& o) {
    ShapeDesc s;
    s.type = (ShapeDesc::Type)enumOf(o.value("shape", std::string("box")), kShapeNames, 8, "shape");
    s.margin = f(o, "margin", 0.04f);
    switch (s.type) {
      case ShapeDesc::kBox: case ShapeDesc::kCylinderZ: s.half = v(o, "half", Vec3()); break;
      case ShapeDesc::kSphere: s.radius = f(o, "radius", 0); break;
      case ShapeDesc::kConeZ: case ShapeDesc::kCapsuleZ: s.radius = f(o, "radius", 0); s.height = f(o, "height", 0); break;
      case ShapeDesc::kHull:
        if (o.contains("points")) for (const auto& p : o["points"]) s.points.push_back(v3(p));
        break;
      case ShapeDesc::kMesh:
        if (o.contains("mesh")) s.mesh = mesh(o["mesh"]);
        s.dynamicMesh = b(o, "dynamic", false);
        s.scale = v(o, "scale", Vec3(1, 1, 1));
        break;
      case ShapeDesc::kCompound:
        if (o.contains("children"))
          for (const auto& c : o["children"]) {
            ChildDesc ch;
            ch.shape = shape(c);
            ch.xform = x(c, "at");
            s.children.push_back(ch);
          }
        break;
    }
    return s;
  }

  BodyDesc body(const json& o) {
    BodyDesc d;
    d.kind = (BodyKind)enumOf(o.value("type", std::string("rigid")), kKindNames, 6, "body type");
    d.name = o.value("name", std::string());
    if (d.kind != kSoft) d.shape = shape(o);
    d.xform = x(o, "at");
    d.mass = f(o, "mass", d.mass);
    d.friction = f(o, "friction", d.friction);
    d.restitution = f(o, "restitution", d.restitution);
    d.linDamp = f(o, "lin_damp", d.linDamp);
    d.angDamp = f(o, "ang_damp", d.angDamp);
    d.inertia = f(o, "inertia", d.inertia);
    d.linFactor = v(o, "lin_factor", d.linFactor);
    d.angFactor = v(o, "ang_factor", d.angFactor);
    d.ghost = b(o, "ghost", false);
    d.userGroup = (uint16_t)i(o, "user_group", 0xffff);
    d.userMask = (uint16_t)i(o, "user_mask", 0xffff);
    d.clampVelMin = f(o, "clamp_vel_min", 0); d.clampVelMax = f(o, "clamp_vel_max", 0);
    d.clampAngMin = f(o, "clamp_ang_min", 0); d.clampAngMax = f(o, "clamp_ang_max", 0);
    d.reportCollisions = b(o, "report", false);
    d.hasLinVel = o.contains("lin_vel"); d.linVel = v(o, "lin_vel", Vec3());
    d.hasAngVel = o.contains("ang_vel"); d.angVel = v(o, "ang_vel", Vec3());
    d.noSleep = b(o, "no_sleep", false);
    d.animated = b(o, "animated", false);
    d.useDeactivation = b(o, "use_deactivation", false);
    d.startDeactivated = b(o, "start_deactivated", false);
    d.linSleep = f(o, "sleep_lin", d.linSleep);
    d.angSleep = f(o, "sleep_ang", d.angSleep);
    d.colGroups = i(o, "col_groups", 1);
    d.stepHeight = f(o, "step_height", 0); d.jumpSpeed = f(o, "jump_speed", 0); d.fallSpeed = f(o, "fall_speed", 0);
    if (d.kind == kSoft) {
      if (o.contains("mesh")) d.softMesh = mesh(o["mesh"]);
      d.bending = b(o, "bending", false);
      d.shapeMatching = b(o, "shape_matching", false);
    }
    return d;
  }

  ConstraintDesc constraint(const json& o) {
    ConstraintDesc c;
    c.type = (ConstraintDesc::Type)enumOf(o.value("type", std::string()), kConstraintNames, kConstraintCount, "constraint");
    c.a = i(o, "a", 0); c.b = i(o, "b", -1);
    c.pivotA = v(o, "pivot_a", Vec3()); c.pivotB = v(o, "pivot_b", Vec3());
    c.frameA = x(o, "frame_a"); c.frameB = x(o, "frame_b");
    c.angularOnly = b(o, "angular_only", false);
    c.frame = x(o, "frame");
    c.enabled = b(o, "enabled", true);
    c.disableCollisions = b(o, "disable_collisions", true);
    c.breakingThreshold = f(o, "breaking_threshold", -1);
    c.iterations = i(o, "iterations", -1);
    for (int k = 0; k < 6; ++k) {
      if (o.contains("limits") && o["limits"].contains(kAxisNames[k])) {
        const json& l = o["limits"][kAxisNames[k]];
        c.useLimit[k] = true; c.lower[k] = (float)l[0].get<double>(); c.upper[k] = (float)l[1].get<double>();
      }
      if (o.contains("springs") && o["springs"].contains(kAxisNames[k])) {
        const json& l = o["springs"][kAxisNames[k]];
        c.useSpring[k] = true; c.stiffness[k] = (float)l[0].get<double>(); c.damping[k] = (float)l[1].get<double>();
      }
    }
    c.springType = i(o, "spring_type", 2);
    if (o.contains("motor_lin")) { c.useMotorLin = true; c.motorLinVelocity = (float)o["motor_lin"][0].get<double>(); c.motorLinMaxImpulse = (float)o["motor_lin"][1].get<double>(); }
    if (o.contains("motor_ang")) { c.useMotorAng = true; c.motorAngVelocity = (float)o["motor_ang"][0].get<double>(); c.motorAngMaxImpulse = (float)o["motor_ang"][1].get<double>(); }
    return c;
  }

  ActionDesc action(const json& o) {
    ActionDesc a;
    a.op = (ActionDesc::Op)enumOf(o.value("op", std::string()), kOpNames, 7, "action");
    if (o.contains("at")) a.from = a.to = i(o, "at", 0);
    else { a.from = i(o, "from", 0); a.to = i(o, "to", -1); }
    a.target = a.op == ActionDesc::kVehicle ? i(o, "vehicle", 0) : i(o, "body", 0);
    a.engine = fl(o, "engine"); a.brake = fl(o, "brake"); a.steer = fl(o, "steer");
    a.v = v(o, "v", Vec3());
    return a;
  }
};

}  // namespace

std::string sceneToJson(const SceneDesc& s) {
  const SettingsDesc d;
  json head = json::object();
  head["name"] = s.name;
  head["covers"] = s.covers;
  head["profile"] = s.profile;
  head["frames"] = s.frames;
  json st = json::object();
  st["ticrate"] = jf(s.settings.ticrate);
  st["substeps"] = s.settings.substeps;
  st["gravity_z"] = jf(s.settings.gravityZ);
  st["contact_breaking"] = jf(s.settings.contactBreaking);
  st["deactivation_time"] = jf(s.settings.deactivationTime);
  st["lin_sleep"] = jf(s.settings.linSleep);
  st["ang_sleep"] = jf(s.settings.angSleep);
  if (s.settings.setSolverParams) {
    st["sor"] = jf(s.settings.sor); st["tau"] = jf(s.settings.tau); st["damping"] = jf(s.settings.damping);
  }
  if (s.settings.iterations > 0) st["iterations"] = s.settings.iterations;
  if (s.settings.splitImpulse) st["split_impulse"] = true;
  if (s.settings.timeScale != 1.0f) st["time_scale"] = jf(s.settings.timeScale);
  if (s.settings.nncg) st["solver"] = "nncg";
  if (s.settings.softStep) st["solver"] = "soft_step";
  if (s.settings.solverSubsteps > 0) st["solver_substeps"] = s.settings.solverSubsteps;
  if (s.settings.substepRate > 0) st["solver_substep_rate"] = jf(s.settings.substepRate);
  if (s.settings.solverIterations > 0) st["solver_iterations"] = s.settings.solverIterations;
  if (s.settings.relaxIterations >= 0) st["relax_iterations"] = s.settings.relaxIterations;
  if (s.settings.manifoldIterations > 0) st["manifold_iterations"] = s.settings.manifoldIterations;
  if (s.settings.jointIterations > 0) st["joint_iterations"] = s.settings.jointIterations;
  if (s.settings.jointWarmStart >= 0) st["joint_warm_start"] = jf(s.settings.jointWarmStart);
  if (s.settings.jointHertz >= 0) st["joint_hertz"] = jf(s.settings.jointHertz);
  if (s.settings.jointDampingRatio >= 0) st["joint_damping_ratio"] = jf(s.settings.jointDampingRatio);
  if (s.settings.contactHertz >= 0) st["contact_hertz"] = jf(s.settings.contactHertz);
  if (s.settings.contactDampingRatio >= 0) st["contact_damping_ratio"] = jf(s.settings.contactDampingRatio);
  if (s.settings.contactPushSpeed >= 0) st["contact_push_speed"] = jf(s.settings.contactPushSpeed);
  if (s.settings.erp >= 0) st["erp"] = jf(s.settings.erp);
  if (s.settings.erp2 >= 0) st["erp2"] = jf(s.settings.erp2);
  if (s.settings.cfm >= 0) st["cfm"] = jf(s.settings.cfm);
  if (s.settings.features >= 0) st["features"] = s.settings.features;
  head["settings"] = st;

  // Written by hand so that every object and action sits on one line.
  std::ostringstream out;
  out << "{\n";
  for (auto it = head.begin(); it != head.end(); ++it) out << "  " << json(it.key()).dump() << ": " << it.value().dump() << ",\n";
  out << "  \"objects\": [\n";
  for (size_t n = 0; n < s.objects.size(); ++n) {
    const ObjectDesc& ob = s.objects[n];
    json o;
    switch (ob.type) {
      case ObjectDesc::kBody: o = json::object({{"body", jbody(s.bodies[ob.index])}}); break;
      case ObjectDesc::kConstraint: o = json::object({{"constraint", jconstraint(s.constraints[ob.index])}}); break;
      case ObjectDesc::kParam: {
        const ParamDesc& p = s.params[ob.index];
        o = json::object({{"param", json::object({{"constraint", p.constraint}, {"index", p.index}, {"v0", jf(p.v0)}, {"v1", jf(p.v1)}})}});
        break;
      }
      case ObjectDesc::kVehicle: o = json::object({{"vehicle", json::object({{"chassis", s.vehicles[ob.index].chassis}})}}); break;
      case ObjectDesc::kWheel: {
        const WheelDesc& w = s.wheels[ob.index];
        json wj = json::object({{"vehicle", w.vehicle}, {"pos", jv(w.pos)}, {"down", jv(w.down)}, {"axle", jv(w.axle)},
                                {"rest_length", jf(w.restLength)}, {"radius", jf(w.radius)}, {"steering", w.steering}});
        if (w.tuning.set)
          wj["tuning"] = json::object({{"suspension_stiffness", jf(w.tuning.suspensionStiffness)},
                                       {"damping_compression", jf(w.tuning.dampingCompression)},
                                       {"damping_relaxation", jf(w.tuning.dampingRelaxation)},
                                       {"friction_slip", jf(w.tuning.frictionSlip)},
                                       {"roll_influence", jf(w.tuning.rollInfluence)}});
        o = json::object({{"wheel", wj}});
        break;
      }
    }
    out << "    " << o.dump() << (n + 1 < s.objects.size() ? ",\n" : "\n");
  }
  out << "  ],\n  \"actions\": [\n";
  for (size_t n = 0; n < s.actions.size(); ++n)
    out << "    " << jaction(s.actions[n]).dump() << (n + 1 < s.actions.size() ? ",\n" : "\n");
  out << "  ]\n}\n";
  return out.str();
}

bool parseScene(const std::string& text, SceneDesc& s, std::string& error) {
  json root;
  try {
    root = json::parse(text);
  } catch (const std::exception& e) {
    error = e.what();
    return false;
  }
  Reader r;
  try {
    s = SceneDesc();
    s.name = root.value("name", std::string());
    s.covers = root.value("covers", std::string());
    s.profile = root.value("profile", std::string("bge279"));
    s.frames = r.i(root, "frames", 0);
    if (root.contains("settings")) {
      const json& st = root["settings"];
      SettingsDesc& d = s.settings;
      d.ticrate = r.f(st, "ticrate", d.ticrate);
      d.substeps = r.i(st, "substeps", d.substeps);
      d.gravityZ = r.f(st, "gravity_z", d.gravityZ);
      d.contactBreaking = r.f(st, "contact_breaking", d.contactBreaking);
      d.deactivationTime = r.f(st, "deactivation_time", d.deactivationTime);
      d.linSleep = r.f(st, "lin_sleep", d.linSleep);
      d.angSleep = r.f(st, "ang_sleep", d.angSleep);
      d.setSolverParams = st.contains("sor") || st.contains("tau") || st.contains("damping");
      d.sor = r.f(st, "sor", d.sor); d.tau = r.f(st, "tau", d.tau); d.damping = r.f(st, "damping", d.damping);
      d.iterations = r.i(st, "iterations", -1);
      d.splitImpulse = r.b(st, "split_impulse", false);
      d.timeScale = r.f(st, "time_scale", 1.0f);
      d.nncg = st.value("solver", std::string("sequential")) == "nncg";
      d.softStep = st.value("solver", std::string("sequential")) == "soft_step";
      d.solverSubsteps = r.i(st, "solver_substeps", -1);
      d.substepRate = r.f(st, "solver_substep_rate", -1);
      d.solverIterations = r.i(st, "solver_iterations", -1);
      d.relaxIterations = r.i(st, "relax_iterations", -1);
      d.manifoldIterations = r.i(st, "manifold_iterations", -1);
      d.jointIterations = r.i(st, "joint_iterations", -1);
      d.jointWarmStart = r.f(st, "joint_warm_start", -1);
      d.jointHertz = r.f(st, "joint_hertz", -1);
      d.jointDampingRatio = r.f(st, "joint_damping_ratio", -1);
      d.contactHertz = r.f(st, "contact_hertz", -1);
      d.contactDampingRatio = r.f(st, "contact_damping_ratio", -1);
      d.contactPushSpeed = r.f(st, "contact_push_speed", -1);
      d.erp = r.f(st, "erp", -1); d.erp2 = r.f(st, "erp2", -1); d.cfm = r.f(st, "cfm", -1);
      d.features = r.i(st, "features", -1);
    }
    if (root.contains("objects"))
      for (const auto& o : root["objects"]) {
        if (o.contains("body")) s.addBody(r.body(o["body"]));
        else if (o.contains("constraint")) s.addConstraint(r.constraint(o["constraint"]));
        else if (o.contains("param")) {
          const json& p = o["param"];
          s.addParam(r.i(p, "constraint", 0), r.i(p, "index", 0), r.f(p, "v0", 0), r.f(p, "v1", 0));
        } else if (o.contains("vehicle")) s.addVehicle(r.i(o["vehicle"], "chassis", 0));
        else if (o.contains("wheel")) {
          const json& wj = o["wheel"];
          WheelDesc w;
          w.vehicle = r.i(wj, "vehicle", 0);
          w.pos = r.v(wj, "pos", Vec3()); w.down = r.v(wj, "down", Vec3(0, 0, -1)); w.axle = r.v(wj, "axle", Vec3(-1, 0, 0));
          w.restLength = r.f(wj, "rest_length", 0); w.radius = r.f(wj, "radius", 0);
          w.steering = r.b(wj, "steering", false);
          if (wj.contains("tuning")) {
            const json& t = wj["tuning"];
            w.tuning.set = true;
            w.tuning.suspensionStiffness = r.f(t, "suspension_stiffness", 0);
            w.tuning.dampingCompression = r.f(t, "damping_compression", 0);
            w.tuning.dampingRelaxation = r.f(t, "damping_relaxation", 0);
            w.tuning.frictionSlip = r.f(t, "friction_slip", 0);
            w.tuning.rollInfluence = r.f(t, "roll_influence", 0);
          }
          s.addWheel(w);
        } else r.fail("unknown object entry");
      }
    if (root.contains("actions"))
      for (const auto& a : root["actions"]) s.actions.push_back(r.action(a));
  } catch (const std::exception& e) {
    r.fail(e.what());
  }
  if (!r.ok()) { error = r.err; return false; }
  return true;
}

bool loadScene(const std::string& path, SceneDesc& s, std::string& error) {
  std::ifstream f(path, std::ios::binary);
  if (!f) { error = "cannot open " + path; return false; }
  std::stringstream ss;
  ss << f.rdbuf();
  if (!parseScene(ss.str(), s, error)) { error = path + ": " + error; return false; }
  return true;
}

bool saveScene(const std::string& path, const SceneDesc& s) {
  std::ofstream f(path, std::ios::binary);
  if (!f) return false;
  f << sceneToJson(s);
  return (bool)f;
}

std::vector<std::string> listScenes(const std::string& dir) {
  std::vector<std::string> names;
  std::ifstream f(dir + "/index.json", std::ios::binary);
  if (!f) return names;
  try {
    json j = json::parse(f);
    for (const auto& n : j["scenes"]) names.push_back(n.get<std::string>());
  } catch (const std::exception&) {
  }
  return names;
}

}  // namespace bnl
