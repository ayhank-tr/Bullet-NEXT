// Bullet NEXT Lab - runs scenes on several Lab engines inside one program (FL, CL.2).
//   bnl_run [--engines legacy,nextf] [--scene NAME|all] [--scenes DIR] [--threads N] [--record DIR]
//           [--replay-check 1] [--metrics FILE]
// --record DIR writes <scene>.<engine>.bnl recordings. --replay-check 1 (TL.6) plays every
// engine's recording back and checks that each frame's states and hashes equal the live run.
// The engines are stepped in turn, frame by frame, as the side-by-side viewer does. Prints each
// engine's final hash per scene; they must equal the standalone harness runners' hashes.
// Also checks every body state (TL.4): NaN or infinity, speeds above 1,000 m/s, bodies that fell
// more than 50 m below the scene (ghosts excepted: they fall through by design), and reports the
// total physics time.
// --metrics FILE (F3c, C3.9) appends one JSON line per scene and engine: deepest penetration
// (all frames, and after the first 2 s), fastest body after 2 s, joint drift, energy, solver
// timings, each body's first bounce apex, and how far the bodies moved and tilted.
#include "bnl_engine.h"
#include "bnl_recording.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <sstream>

namespace {

// Measurements of one engine over one scene (C3.9).
struct Metrics {
  float penMax = 0, penRest = 0, jitter = 0;
  int penFrame = 0, penA = -1, penB = -1;
  float driftLin = 0, driftAng = 0;
  double driftLinSum = 0;
  std::vector<float> jointLin, jointAng;  // per joint, the worst over the run
  long driftN = 0;
  bool hasEnergy = false;
  double e0 = 0, eMax = -1e300, eMin = 1e300, eEnd = 0;
  std::vector<int> phase;       // bounce state per body: 0 start, 1 falling, 2 rising, 3 done
  std::vector<float> apex;
  std::vector<float> lastVz, vIn, vOut, zLow;   // first bounce: speeds in and out, lowest point
  std::vector<bnl::BodyState> start;
};

bool solid(const bnl::BodyDesc& b) { return (b.kind == bnl::kRigid || b.kind == bnl::kDynamic) && !b.ghost; }
bool touchable(const bnl::BodyDesc& b) { return (b.kind == bnl::kRigid || b.kind == bnl::kDynamic || b.kind == bnl::kStatic) && !b.ghost; }

void beginMetrics(const bnl::SceneDesc& scene, const bnl::Engine& e, Metrics& m) {
  const int n = e.bodyCount();
  m.phase.assign(n, 0);
  m.apex.assign(n, -1.0f);
  m.lastVz.assign(n, 0.0f);
  m.vIn.assign(n, 0.0f);
  m.vOut.assign(n, 0.0f);
  m.zLow.assign(n, 0.0f);
  m.start.resize(n);
  for (int b = 0; b < n; ++b) m.start[b] = e.body(b);
  double en = 0;
  m.hasEnergy = e.energy(en);
  m.e0 = en;
  (void)scene;
}

void frameMetrics(const bnl::SceneDesc& scene, const bnl::Engine& e, int frame, Metrics& m) {
  const bool rest = frame > int(2.0f * scene.settings.ticrate + 0.5f);
  std::vector<bnl::Contact> cs;
  e.contacts(cs);
  for (const bnl::Contact& c : cs) {
    if (c.a < 0 || c.b < 0 || c.a >= (int)scene.bodies.size() || c.b >= (int)scene.bodies.size()) continue;
    if (!touchable(scene.bodies[c.a]) || !touchable(scene.bodies[c.b])) continue;
    if (!solid(scene.bodies[c.a]) && !solid(scene.bodies[c.b])) continue;
    const float pen = -c.dist;
    if (pen > m.penMax) { m.penMax = pen; m.penFrame = frame; m.penA = c.a; m.penB = c.b; }
    if (rest) m.penRest = std::max(m.penRest, pen);
  }
  for (int b = 0; b < e.bodyCount() && b < (int)scene.bodies.size(); ++b) {
    if (!solid(scene.bodies[b])) continue;
    const bnl::BodyState s = e.body(b);
    const float speed = std::sqrt(s.lin.x * s.lin.x + s.lin.y * s.lin.y + s.lin.z * s.lin.z);
    if (rest) m.jitter = std::max(m.jitter, speed);
    if (m.phase[b] == 1) m.zLow[b] = s.p.z;
    int& ph = m.phase[b];
    if (ph == 0 && s.lin.z < -0.1f) ph = 1;
    else if (ph == 1 && s.lin.z > 0.05f) { ph = 2; m.apex[b] = s.p.z; m.vIn[b] = m.lastVz[b]; m.vOut[b] = s.lin.z; m.zLow[b] = s.p.z; }
    else if (ph == 2) { m.apex[b] = std::max(m.apex[b], s.p.z); if (s.lin.z < -0.05f) ph = 3; }
    m.lastVz[b] = s.lin.z;
  }
  std::vector<bnl::JointError> je;
  e.jointErrors(je);
  for (const bnl::JointError& j : je) {
    if (j.constraint >= (int)m.jointLin.size()) { m.jointLin.resize(j.constraint + 1, -1.0f); m.jointAng.resize(j.constraint + 1, -1.0f); }
    if (j.linear >= 0) { m.driftLin = std::max(m.driftLin, j.linear); m.driftLinSum += j.linear; ++m.driftN; m.jointLin[j.constraint] = std::max(m.jointLin[j.constraint], j.linear); }
    if (j.angular >= 0) { m.driftAng = std::max(m.driftAng, j.angular); m.jointAng[j.constraint] = std::max(m.jointAng[j.constraint], j.angular); }
  }
  double en = 0;
  if (e.energy(en)) { m.eMax = std::max(m.eMax, en); m.eMin = std::min(m.eMin, en); m.eEnd = en; }
}

std::string num(double v) {
  char b[64];
  std::snprintf(b, sizeof b, "%.6g", v);
  return b;
}

std::string metricsJson(const bnl::SceneDesc& scene, const bnl::Engine& e, double ms, const Metrics& m) {
  std::ostringstream o;
  o << "{\"scene\": \"" << scene.name << "\", \"engine\": \"" << e.backend() << "\", \"frames\": " << e.frame()
    << ", \"ms\": " << num(ms) << ", \"pen_max\": " << num(m.penMax) << ", \"pen_max_at\": [" << m.penFrame << ", " << m.penA << ", " << m.penB << "]"
    << ", \"pen_rest\": " << num(m.penRest)
    << ", \"jitter\": " << num(m.jitter) << ", \"drift_lin_max\": " << num(m.driftLin)
    << ", \"drift_lin_mean\": " << num(m.driftN ? m.driftLinSum / m.driftN : 0.0) << ", \"drift_ang_max\": " << num(m.driftAng);
  if (m.hasEnergy)
    o << ", \"energy\": {\"start\": " << num(m.e0) << ", \"max\": " << num(m.eMax) << ", \"min\": " << num(m.eMin) << ", \"end\": " << num(m.eEnd) << "}";
  float moved = 0, dzMin = 0, tilt = 0;
  o << ", \"apex\": [";
  bool first = true;
  for (int b = 0; b < e.bodyCount() && b < (int)scene.bodies.size(); ++b) {
    if (!solid(scene.bodies[b])) continue;
    const bnl::BodyState s = e.body(b), s0 = m.start[b];
    moved = std::max(moved, std::sqrt((s.p.x - s0.p.x) * (s.p.x - s0.p.x) + (s.p.y - s0.p.y) * (s.p.y - s0.p.y)));
    dzMin = std::min(dzMin, s.p.z - s0.p.z);
    // rotation from the start: angle of q * conj(q0)
    const float w = std::fabs(s.q.w * s0.q.w + s.q.x * s0.q.x + s.q.y * s0.q.y + s.q.z * s0.q.z);
    tilt = std::max(tilt, 2.0f * std::acos(std::min(1.0f, w)));
    o << (first ? "" : ", ") << num(m.apex[b]);
    first = false;
  }
  o << "], \"bounce\": [";
  first = true;
  for (int b = 0; b < e.bodyCount() && b < (int)scene.bodies.size(); ++b) {
    if (!solid(scene.bodies[b])) continue;
    o << (first ? "" : ", ") << "[" << num(m.vIn[b]) << ", " << num(m.vOut[b]) << ", " << num(m.zLow[b]) << ", " << num(m.start[b].p.z) << "]";
    first = false;
  }
  o << "], \"joints\": [";
  for (size_t j = 0; j < m.jointLin.size(); ++j) o << (j ? ", " : "") << "[" << num(m.jointLin[j]) << ", " << num(m.jointAng[j]) << "]";
  o << "], \"moved_xy_max\": " << num(moved) << ", \"dz_min\": " << num(dzMin) << ", \"tilt_max\": " << num(tilt);
  bnl::SolverStats st;
  if (e.solverStats(st))
    o << ", \"solver\": {\"prepare_ms\": " << num(st.prepareMicros / 1000) << ", \"substeps_ms\": " << num(st.substepMicros / 1000)
      << ", \"restitution_ms\": " << num(st.restitutionMicros / 1000) << ", \"store_ms\": " << num(st.storeMicros / 1000)
      << ", \"groups\": " << st.groups << ", \"bodies\": " << st.bodies << ", \"contact_points\": " << st.contactPoints
      << ", \"joint_rows\": " << st.jointRows << "}";
  o << "}";
  return o.str();
}

}  // namespace

#ifndef BN_SCENES_DIR
#define BN_SCENES_DIR "lab/scenes"
#endif

int main(int argc, char** argv) {
  std::string engines = "", which = "all", dir = BN_SCENES_DIR;
  int threads = -1;
  std::string recordDir, metricsPath;
  int dumpFrame = -1;
  bool replayCheck = false;
  for (int i = 1; i + 1 < argc; i += 2) {
    if (!std::strcmp(argv[i], "--record")) recordDir = argv[i + 1];
    if (!std::strcmp(argv[i], "--replay-check")) replayCheck = std::atoi(argv[i + 1]) != 0;
    if (!std::strcmp(argv[i], "--metrics")) metricsPath = argv[i + 1];
    if (!std::strcmp(argv[i], "--dump-contacts")) dumpFrame = std::atoi(argv[i + 1]);
    if (!std::strcmp(argv[i], "--engines")) engines = argv[i + 1];
    else if (!std::strcmp(argv[i], "--scene")) which = argv[i + 1];
    else if (!std::strcmp(argv[i], "--scenes")) dir = argv[i + 1];
    else if (!std::strcmp(argv[i], "--threads")) threads = std::atoi(argv[i + 1]);
  }
  std::vector<std::string> names;
  if (engines.empty()) names = bnl::engineNames();
  else {
    std::stringstream ss(engines);
    std::string n;
    while (std::getline(ss, n, ',')) names.push_back(n);
  }
  int failures = 0;
  for (const std::string& sceneName : bnl::listScenes(dir)) {
    if (which != "all" && which != sceneName) continue;
    bnl::SceneDesc scene;
    std::string err;
    if (!bnl::loadScene(dir + "/" + sceneName + ".json", scene, err)) { std::fprintf(stderr, "%s\n", err.c_str()); return 2; }
    std::vector<std::unique_ptr<bnl::Engine> > list;
    for (const std::string& n : names) {
      bnl::Engine* e = bnl::createEngine(n);
      if (!e) { std::fprintf(stderr, "engine '%s' is not linked into this program\n", n.c_str()); return 2; }
      bnl::EngineOptions o;
      o.threads = threads;
      if (!e->build(scene, o, err)) { std::fprintf(stderr, "%s: %s\n", n.c_str(), err.c_str()); ++failures; delete e; continue; }
      list.emplace_back(e);
    }
    std::vector<bnl::Recording> recs(list.size());
    for (size_t k = 0; k < list.size(); ++k) recs[k].scene = scene.name;
    std::vector<double> ms(list.size(), 0.0);
    std::vector<std::string> bad(list.size());
    std::vector<float> maxSpeed(list.size(), 0.0f);
    std::vector<Metrics> metrics(list.size());
    if (!metricsPath.empty())
      for (size_t k = 0; k < list.size(); ++k) beginMetrics(scene, *list[k], metrics[k]);
    for (int f = 0; f < scene.frames; ++f)
      for (size_t k = 0; k < list.size(); ++k) {
        bnl::Engine& e = *list[k];
        e.advance();
        ms[k] += e.lastStepMicros() / 1000.0;
        if (!metricsPath.empty()) frameMetrics(scene, e, f + 1, metrics[k]);
        if (f + 1 == dumpFrame) {
          std::vector<bnl::Contact> cs;
          e.contacts(cs);
          for (const bnl::Contact& c : cs)
            std::printf("contact %s f%d %d-%d dist %.5f pa %.3f %.3f %.3f n %.3f %.3f %.3f\n", e.backend(), f + 1, c.a, c.b, c.dist,
                        c.pa.x, c.pa.y, c.pa.z, c.n.x, c.n.y, c.n.z);
        }
        if (!recordDir.empty() || replayCheck) recs[k].capture(e);
        for (int b = 0; b < e.bodyCount() && bad[k].empty(); ++b) {
          const bnl::BodyState s = e.body(b);
          const float v[13] = {s.p.x, s.p.y, s.p.z, s.q.x, s.q.y, s.q.z, s.q.w, s.lin.x, s.lin.y, s.lin.z, s.ang.x, s.ang.y, s.ang.z};
          for (float x : v)
            if (!std::isfinite(x)) bad[k] = "NaN at frame " + std::to_string(f + 1);
          const float speed = std::sqrt(s.lin.x * s.lin.x + s.lin.y * s.lin.y + s.lin.z * s.lin.z);
          maxSpeed[k] = std::max(maxSpeed[k], speed);
          if (bad[k].empty() && speed > 1000.0f) bad[k] = "speed " + std::to_string(speed) + " at frame " + std::to_string(f + 1);
          // Falls through by design: a ghost (no contact response), or a Blender body in no
          // collision collection of any passive body.
          int passive = 0;
          for (const auto& sb : scene.bodies) if (sb.kind == bnl::kStatic) passive |= sb.colGroups;
          const bool fallsThrough = scene.bodies[b].ghost || (scene.profile == "blender53-rb" && !(scene.bodies[b].colGroups & passive));
          if (bad[k].empty() && !fallsThrough && s.p.z < -50.0f) bad[k] = "body " + std::to_string(b) + " fell below -50 m at frame " + std::to_string(f + 1);
        }
      }
    for (size_t k = 0; k < list.size(); ++k) {
      bnl::Engine& e = *list[k];
      std::printf("%-18s %s frames %4d hash %016llx %s ms %9.2f vmax %8.2f %s\n", sceneName.c_str(), e.backend(), e.frame(),
                  (unsigned long long)e.finalHash(), bad[k].empty() ? "ok " : "BAD", ms[k], maxSpeed[k], bad[k].c_str());
      if (!bad[k].empty()) ++failures;
      if (!metricsPath.empty()) {
        if (FILE* mf = std::fopen(metricsPath.c_str(), "a")) {
          std::fprintf(mf, "%s\n", metricsJson(scene, e, ms[k], metrics[k]).c_str());
          std::fclose(mf);
        }
      }
      if (!recordDir.empty() || replayCheck) {
        const std::string path = (recordDir.empty() ? std::string(".") : recordDir) + "/" + sceneName + "." + e.backend() + ".bnl";
        if (!recs[k].write(path)) { std::fprintf(stderr, "cannot write %s\n", path.c_str()); ++failures; continue; }
        if (replayCheck) {
          std::unique_ptr<bnl::Engine> r(bnl::createEngine("replay"));
          bnl::EngineOptions ro;
          ro.recording = path;
          std::string rerr;
          bool same = r->build(scene, ro, rerr);
          for (int f = 0; same && f < scene.frames; ++f) {
            r->advance();
            const bnl::RecordedFrame& live = recs[k].frames[f];
            same = r->frameHash() == live.hash;
            for (int b = 0; same && b < r->bodyCount(); ++b) { const bnl::BodyState rs = r->body(b); same = std::memcmp(&live.bodies[b], &rs, sizeof(bnl::BodyState)) == 0; }
          }
          same = same && r->finalHash() == e.finalHash();
          std::printf("%-18s %s replay %s %s\n", sceneName.c_str(), e.backend(), same ? "same" : "DIFFERENT", rerr.c_str());
          if (!same) ++failures;
          if (recordDir.empty()) std::remove(path.c_str());
        }
      }
    }
  }
  return failures ? 1 : 0;
}
