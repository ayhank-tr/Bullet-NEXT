// Bullet NEXT Lab - runs scenes on several Lab engines inside one program (FL, CL.2).
//   bnl_run [--engines legacy,nextf] [--scene NAME|all] [--scenes DIR] [--threads N] [--record DIR]
//           [--replay-check 1]
// --record DIR writes <scene>.<engine>.bnl recordings. --replay-check 1 (TL.6) plays every
// engine's recording back and checks that each frame's states and hashes equal the live run.
// The engines are stepped in turn, frame by frame, as the side-by-side viewer does. Prints each
// engine's final hash per scene; they must equal the standalone harness runners' hashes.
// Also checks every body state (TL.4): NaN or infinity, speeds above 1,000 m/s, bodies that fell
// more than 50 m below the scene (ghosts excepted: they fall through by design), and reports the
// total physics time.
#include "bnl_engine.h"
#include "bnl_recording.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <sstream>

#ifndef BN_SCENES_DIR
#define BN_SCENES_DIR "lab/scenes"
#endif

int main(int argc, char** argv) {
  std::string engines = "", which = "all", dir = BN_SCENES_DIR;
  int threads = -1;
  std::string recordDir;
  bool replayCheck = false;
  for (int i = 1; i + 1 < argc; i += 2) {
    if (!std::strcmp(argv[i], "--record")) recordDir = argv[i + 1];
    if (!std::strcmp(argv[i], "--replay-check")) replayCheck = std::atoi(argv[i + 1]) != 0;
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
    for (int f = 0; f < scene.frames; ++f)
      for (size_t k = 0; k < list.size(); ++k) {
        bnl::Engine& e = *list[k];
        e.advance();
        ms[k] += e.lastStepMicros() / 1000.0;
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
