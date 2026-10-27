// Bullet NEXT Lab - recordings of any Lab engine, and their playback as an engine (FL, CL.8).
#include "bnl_recording.h"

#include <cstdio>
#include <cstring>

namespace bnl {

// File layout (little endian), "BNL1":
//   "BNL1" | u32 version | str engine | str scene | u32 bodies | u32 vehicles | u32 frames
//   per frame: per body f32[13] (p, q xyzw, lin, ang)
//              per body u32 n + f32[3n] soft nodes (n = 0 unless soft)
//              per vehicle u32 n + f32[7n] wheels (p, q xyzw)
//              u64 frame hash
//   u64 final hash

void Recording::capture(const Engine& e) {
  RecordedFrame f;
  const int n = e.bodyCount();
  f.bodies.resize(n);
  f.soft.resize(n);
  for (int i = 0; i < n; ++i) {
    f.bodies[i] = e.body(i);
    e.softNodes(i, f.soft[i]);
  }
  f.wheels.resize(e.vehicleCount());
  for (int v = 0; v < e.vehicleCount(); ++v)
    for (int w = 0; w < e.wheelCount(v); ++w) f.wheels[v].push_back(e.wheel(v, w));
  f.hash = e.frameHash();
  frames.push_back(f);
  finalHash = e.finalHash();
  if (engine.empty()) engine = e.backend();
}

namespace {

void putU32(FILE* f, std::uint32_t v) { fwrite(&v, 4, 1, f); }
void putStr(FILE* f, const std::string& s) { putU32(f, (std::uint32_t)s.size()); fwrite(s.data(), 1, s.size(), f); }
void putF(FILE* f, float v) { fwrite(&v, 4, 1, f); }

struct In {
  const std::vector<char>& d;
  size_t o = 0;
  bool bad = false;
  explicit In(const std::vector<char>& data) : d(data) {}
  void get(void* p, size_t n) {
    if (o + n > d.size()) { bad = true; std::memset(p, 0, n); return; }
    std::memcpy(p, &d[o], n);
    o += n;
  }
  std::uint32_t u32() { std::uint32_t v; get(&v, 4); return v; }
  u64 u64v() { u64 v; get(&v, 8); return v; }
  float f() { float v; get(&v, 4); return v; }
  std::string str() {
    const std::uint32_t n = u32();
    if (bad || o + n > d.size()) { bad = true; return std::string(); }
    std::string s(&d[o], n);
    o += n;
    return s;
  }
};

}  // namespace

bool Recording::write(const std::string& path) const {
  FILE* f = std::fopen(path.c_str(), "wb");
  if (!f) return false;
  fwrite("BNL1", 1, 4, f);
  putU32(f, 1);
  putStr(f, engine);
  putStr(f, scene);
  const std::uint32_t bodies = frames.empty() ? 0 : (std::uint32_t)frames[0].bodies.size();
  const std::uint32_t vehicles = frames.empty() ? 0 : (std::uint32_t)frames[0].wheels.size();
  putU32(f, bodies);
  putU32(f, vehicles);
  putU32(f, (std::uint32_t)frames.size());
  for (const RecordedFrame& fr : frames) {
    for (const BodyState& s : fr.bodies) {
      const float v[13] = {s.p.x, s.p.y, s.p.z, s.q.x, s.q.y, s.q.z, s.q.w, s.lin.x, s.lin.y, s.lin.z, s.ang.x, s.ang.y, s.ang.z};
      fwrite(v, 4, 13, f);
    }
    for (const auto& nodes : fr.soft) {
      putU32(f, (std::uint32_t)nodes.size());
      for (const Vec3& p : nodes) { putF(f, p.x); putF(f, p.y); putF(f, p.z); }
    }
    for (const auto& wheels : fr.wheels) {
      putU32(f, (std::uint32_t)wheels.size());
      for (const Xform& x : wheels) {
        const float v[7] = {x.p.x, x.p.y, x.p.z, x.q.x, x.q.y, x.q.z, x.q.w};
        fwrite(v, 4, 7, f);
      }
    }
    fwrite(&fr.hash, 8, 1, f);
  }
  fwrite(&finalHash, 8, 1, f);
  return std::fclose(f) == 0;
}

bool Recording::read(const std::string& path, std::string& error) {
  FILE* f = std::fopen(path.c_str(), "rb");
  if (!f) { error = "cannot open " + path; return false; }
  std::vector<char> data;
  char buf[65536];
  size_t n;
  while ((n = std::fread(buf, 1, sizeof(buf), f)) > 0) data.insert(data.end(), buf, buf + n);
  std::fclose(f);
  In in(data);
  char magic[4];
  in.get(magic, 4);
  if (in.bad || std::memcmp(magic, "BNL1", 4) != 0) { error = path + " is not a Lab recording"; return false; }
  in.u32();
  engine = in.str();
  scene = in.str();
  const std::uint32_t bodies = in.u32(), vehicles = in.u32(), count = in.u32();
  frames.clear();
  for (std::uint32_t k = 0; k < count && !in.bad; ++k) {
    RecordedFrame fr;
    fr.bodies.resize(bodies);
    for (BodyState& s : fr.bodies) {
      float v[13];
      in.get(v, sizeof(v));
      s.p = Vec3(v[0], v[1], v[2]); s.q = Quat(v[3], v[4], v[5], v[6]);
      s.lin = Vec3(v[7], v[8], v[9]); s.ang = Vec3(v[10], v[11], v[12]);
    }
    fr.soft.resize(bodies);
    for (auto& nodes : fr.soft) {
      const std::uint32_t m = in.u32();
      for (std::uint32_t j = 0; j < m && !in.bad; ++j) { const float x = in.f(), y = in.f(), z = in.f(); nodes.push_back(Vec3(x, y, z)); }
    }
    fr.wheels.resize(vehicles);
    for (auto& wheels : fr.wheels) {
      const std::uint32_t m = in.u32();
      for (std::uint32_t j = 0; j < m && !in.bad; ++j) {
        float v[7];
        in.get(v, sizeof(v));
        Xform x;
        x.p = Vec3(v[0], v[1], v[2]);
        x.q = Quat(v[3], v[4], v[5], v[6]);
        wheels.push_back(x);
      }
    }
    fr.hash = in.u64v();
    frames.push_back(fr);
  }
  finalHash = in.u64v();
  if (in.bad) { error = path + " is cut short"; return false; }
  return true;
}

namespace {

class ReplayEngine : public Engine {
 public:
  const char* backend() const override { return m_name.c_str(); }

  bool build(const SceneDesc& s, const EngineOptions& o, std::string& error) override {
    if (!m_rec.read(o.recording, error)) return false;
    if (!m_rec.scene.empty() && m_rec.scene != s.name) { error = o.recording + " records " + m_rec.scene + ", not " + s.name; return false; }
    if (!m_rec.frames.empty() && (int)m_rec.frames[0].bodies.size() != (int)s.bodies.size()) {
      error = o.recording + " has a different number of bodies than the scene";
      return false;
    }
    m_name = "replay:" + m_rec.engine;
    m_bodies = (int)s.bodies.size();
    return true;
  }

  void advance() override {
    if (m_frame >= (int)m_rec.frames.size()) return;
    const u64 h = m_rec.frames[m_frame].hash;
    u64 fin = m_final;
    const unsigned char* c = reinterpret_cast<const unsigned char*>(&h);
    for (int i = 0; i < 8; ++i) { fin ^= c[i]; fin *= 1099511628211ULL; }
    m_final = fin;
    ++m_frame;
  }
  int frame() const override { return m_frame; }
  double lastStepMicros() const override { return 0; }
  int bodyCount() const override { return m_bodies; }

  const RecordedFrame* current() const { return m_frame > 0 ? &m_rec.frames[m_frame - 1] : nullptr; }

  BodyState body(int i) const override {
    const RecordedFrame* f = current();
    return f ? f->bodies[i] : (m_rec.frames.empty() ? BodyState() : m_rec.frames[0].bodies[i]);
  }
  void softNodes(int i, std::vector<Vec3>& out) const override {
    const RecordedFrame* f = current() ? current() : (m_rec.frames.empty() ? nullptr : &m_rec.frames[0]);
    out = f ? f->soft[i] : std::vector<Vec3>();
  }
  int vehicleCount() const override { return m_rec.frames.empty() ? 0 : (int)m_rec.frames[0].wheels.size(); }
  int wheelCount(int v) const override { return m_rec.frames.empty() ? 0 : (int)m_rec.frames[0].wheels[v].size(); }
  Xform wheel(int v, int w) const override {
    const RecordedFrame* f = current() ? current() : &m_rec.frames[0];
    return f->wheels[v][w];
  }
  void contacts(std::vector<Contact>& out) const override { out.clear(); }
  u64 frameHash() const override { return current() ? current()->hash : 0; }
  u64 finalHash() const override { return m_final; }

  // A recording cannot be steered.
  void setLinearVelocity(int, const Vec3&) override {}
  void setAngularVelocity(int, const Vec3&) override {}
  void applyForce(int, const Vec3&) override {}
  void applyForceAt(int, const Vec3&, const Vec3&) override {}
  void applyImpulse(int, const Vec3&) override {}
  void applyImpulseAt(int, const Vec3&, const Vec3&) override {}
  void applyTorque(int, const Vec3&) override {}
  void applyTorqueImpulse(int, const Vec3&) override {}
  void setTransform(int, const Xform&) override {}
  void wake(int) override {}
  void setSleeping(int, bool) override {}
  bool raycast(const Vec3&, const Vec3&, RayHit&) const override { return false; }
  void raycastAll(const Vec3&, const Vec3&, std::vector<RayHit>& h) const override { h.clear(); }
  void vehicleControl(int, int, float, float, float) override {}
  void characterWalk(int, const Vec3&) override {}
  void characterJump(int) override {}

 private:
  Recording m_rec;
  std::string m_name = "replay";
  int m_bodies = 0, m_frame = 0;
  u64 m_final = 0;
};

}  // namespace

Engine* createReplayEngine() { return new ReplayEngine(); }

}  // namespace bnl
