// Bullet NEXT Lab - random collision pairs on the Bullet engines (FL, CL.11, test TL.7).
//   bnl_fuzz [--pairs N] [--seed S] [--engines legacy,next,nextf]
// Each engine gets the same N pairs (harness/src/bn_fuzz.cpp). Exit code 1 if any pair produced
// a NaN or an infinity; contacts whose normal is not unit length are counted and reported.
#include "bnl_engine.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

int main(int argc, char** argv) {
  int pairs = 100000;
  unsigned seed = 1;
  std::string engines = "legacy,next,nextf";
  for (int i = 1; i + 1 < argc; i += 2) {
    if (!std::strcmp(argv[i], "--pairs")) pairs = std::atoi(argv[i + 1]);
    else if (!std::strcmp(argv[i], "--seed")) seed = (unsigned)std::strtoul(argv[i + 1], nullptr, 10);
    else if (!std::strcmp(argv[i], "--engines")) engines = argv[i + 1];
  }
  std::stringstream ss(engines);
  std::string name;
  int bad = 0;
  while (std::getline(ss, name, ',')) {
    bnl::FuzzResult r;
    const auto t0 = std::chrono::steady_clock::now();
    if (!bnl::fuzzCollisionPairs(name, seed, pairs, r)) {
      std::fprintf(stderr, "engine '%s' has no pair fuzzer in this program\n", name.c_str());
      return 2;
    }
    const double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::printf("%-7s pairs %d touching %d contacts %d nonfinite %d failed %d bad_normals %d deepest %.4g  %.1f s\n",
                name.c_str(), r.pairs, r.touching, r.contacts, r.nonFinite, r.failed, r.badNormals, r.deepest, s);
    for (const std::string& f : r.failures) std::printf("  %s\n", f.c_str());
    if (r.failed) ++bad;
  }
  return bad ? 1 : 0;
}
