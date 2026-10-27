// Bullet NEXT golden harness - headless runner (commit C0.10; scene files since FL, CL.5).
//   bn_harness_<backend> list [--scenes DIR]
//   bn_harness_<backend> run <scene|all> [--out DIR] [--repeat N] [--variant K] [--scenes DIR]
//   bn_harness_<backend> record <scene|all> [--out DIR] [--scenes DIR]
// Scenes are the Lab scene files (lab/scenes/<name>.json, listed by index.json). "record" writes
// Lab recordings (<scene>.<backend>.bnl) through the Lab engine, for the viewer's playback (CL.8).
// --variant K adds 0.0005*K m/s^2 to gravity: a tiny change that shows how chaotic a scene is (T1.5).
// Writes per scene: <scene>.<backend>.bnr (recording), <scene>.<backend>.metrics.csv and one
// summary line (scene, frames, final hash, hash stable over repeats, best ms) to summary.csv.
#ifdef _WIN32
#include <direct.h>
#else
#include <sys/stat.h>
#endif
#include "bn_record.h"
#include "bn_load.h"
#include "bnl_recording.h"

#include <memory>

#include <chrono>
#include <cstdlib>
#include <cstring>

using namespace bn;

#define BN_CAT2(a, b) a##b
#define BN_CAT(a, b) BN_CAT2(a, b)
bnl::Engine* BN_CAT(bnl_create_engine_, BN_TAG)();

namespace {

// Creates the output folder (one level) so recordings are never dropped silently.
void makeDir(const std::string& d) {
#ifdef _WIN32
  _mkdir(d.c_str());
#else
  mkdir(d.c_str(), 0755);
#endif
}


struct RunResult {
  u64 finalHash = 0;
  double totalMs = 0;
  size_t frames = 0;
};

static int gVariant = 0;

RunResult runOnce(const bnl::SceneDesc& sc, const std::string& outDir, bool write) {
  Settings settings = settingsOf(sc);
  settings.gravityZ += 0.0005f * float(gVariant);
  World w(settings);
  build(w, sc);
  Recorder rec(sc.name, w);
  RunResult r;
  for (int f = 0; f < sc.frames; ++f) {
    act(w, sc, f);
    const auto t0 = std::chrono::steady_clock::now();
    w.step();
    const auto t1 = std::chrono::steady_clock::now();
    FrameMetrics m = measure(w);
    m.stepMicros = std::chrono::duration<double, std::micro>(t1 - t0).count();
    r.totalMs += m.stepMicros / 1000.0;
    rec.capture(w, m);
  }
  r.finalHash = rec.finalHash();
  r.frames = rec.frames();
  if (write) {
    const std::string base = outDir + "/" + sc.name + "." + BN_BACKEND_NAME;
    rec.writeBinary(base + ".bnr");
    rec.writeMetricsCsv(base + ".metrics.csv");
  }
  return r;
}

}  // namespace

#ifndef BN_SCENES_DIR
#define BN_SCENES_DIR "lab/scenes"
#endif

bool loadAll(const std::string& dir, std::vector<bnl::SceneDesc>& out) {
  const std::vector<std::string> names = bnl::listScenes(dir);
  if (names.empty()) { std::fprintf(stderr, "no scenes in %s/index.json\n", dir.c_str()); return false; }
  for (size_t i = 0; i < names.size(); ++i) {
    bnl::SceneDesc s;
    std::string err;
    if (!bnl::loadScene(dir + "/" + names[i] + ".json", s, err)) { std::fprintf(stderr, "%s\n", err.c_str()); return false; }
    out.push_back(s);
  }
  return true;
}

int main(int argc, char** argv) {
  std::string sceneDir = BN_SCENES_DIR;
  for (int i = 1; i + 1 < argc; ++i)
    if (!std::strcmp(argv[i], "--scenes")) sceneDir = argv[i + 1];
  std::vector<bnl::SceneDesc> scenes;
  if (!loadAll(sceneDir, scenes)) return 2;
  if (argc < 2 || !std::strcmp(argv[1], "list")) {
    std::printf("backend %s (BT_BULLET_VERSION %d)\n", BN_BACKEND_NAME, (int)BT_BULLET_VERSION);
    for (size_t i = 0; i < scenes.size(); ++i)
      std::printf("  %-18s frames %4d  covers %s\n", scenes[i].name.c_str(), scenes[i].frames, scenes[i].covers.c_str());
    return 0;
  }
  if (!std::strcmp(argv[1], "record") && argc >= 3) {
    std::string which = argv[2], outDir = ".";
    for (int i = 3; i + 1 < argc; i += 2)
      if (!std::strcmp(argv[i], "--out")) outDir = argv[i + 1];
    makeDir(outDir);
    for (size_t i = 0; i < scenes.size(); ++i) {
      const bnl::SceneDesc& sc = scenes[i];
      if (which != "all" && which != sc.name) continue;
      std::unique_ptr<bnl::Engine> e(BN_CAT(bnl_create_engine_, BN_TAG)());
      std::string err;
      if (!e->build(sc, bnl::EngineOptions(), err)) { std::fprintf(stderr, "%s: %s\n", sc.name.c_str(), err.c_str()); return 1; }
      bnl::Recording rec;
      rec.scene = sc.name;
      for (int f = 0; f < sc.frames; ++f) { e->advance(); rec.capture(*e); }
      const std::string path = outDir + "/" + sc.name + "." + BN_BACKEND_NAME + ".bnl";
      if (!rec.write(path)) { std::fprintf(stderr, "cannot write %s\n", path.c_str()); return 1; }
      std::printf("%-18s %s frames %4d hash %016llx -> %s\n", sc.name.c_str(), BN_BACKEND_NAME, e->frame(),
                  (unsigned long long)e->finalHash(), path.c_str());
    }
    return 0;
  }
  if (std::strcmp(argv[1], "run") || argc < 3) {
    std::fprintf(stderr, "usage: %s run <scene|all> [--out DIR] [--repeat N] [--variant K]\n", argv[0]);
    return 2;
  }
  std::string which = argv[2], outDir = ".";
  int repeat = 3;
  for (int i = 3; i + 1 < argc; i += 2) {
    if (!std::strcmp(argv[i], "--out")) outDir = argv[i + 1];
    else if (!std::strcmp(argv[i], "--repeat")) repeat = std::atoi(argv[i + 1]);
    else if (!std::strcmp(argv[i], "--variant")) gVariant = std::atoi(argv[i + 1]);
    else if (!std::strcmp(argv[i], "--scenes")) continue;
  }
  if (repeat < 1) repeat = 1;
  makeDir(outDir);
  std::string summary = outDir + "/summary." + BN_BACKEND_NAME + ".csv";
  FILE* s = std::fopen(summary.c_str(), "w");
  if (s) std::fprintf(s, "scene,backend,frames,final_hash,stable_over_%d_runs,best_ms\n", repeat);
  int unstable = 0;
  for (size_t i = 0; i < scenes.size(); ++i) {
    const bnl::SceneDesc& sc = scenes[i];
    if (which != "all" && which != sc.name) continue;
    RunResult first = runOnce(sc, outDir, true);
    bool stable = true;
    double best = first.totalMs;
    for (int r = 1; r < repeat; ++r) {
      RunResult again = runOnce(sc, outDir, false);
      stable = stable && again.finalHash == first.finalHash;
      if (again.totalMs < best) best = again.totalMs;
    }
    if (!stable) ++unstable;
    std::printf("%-18s %s frames %4zu hash %016llx %s best %.2f ms\n", sc.name.c_str(), BN_BACKEND_NAME,
                first.frames, (unsigned long long)first.finalHash, stable ? "stable  " : "UNSTABLE", best);
    if (s) std::fprintf(s, "%s,%s,%zu,%016llx,%s,%.3f\n", sc.name.c_str(), BN_BACKEND_NAME, first.frames,
                        (unsigned long long)first.finalHash, stable ? "yes" : "no", best);
  }
  if (s) std::fclose(s);
  return unstable ? 1 : 0;
}
