// Bullet NEXT - compare tool (commit C0.7). Independent of any Bullet build.
//   bn_compare <golden.bnr> <candidate.bnr>                 exact mode (bit-identical)
//   bn_compare <golden.bnr> <candidate.bnr> --tol P R V     tolerance mode (metres, radians, m/s)
//   bn_compare --selftest <file.bnr>                        T0.5: exact catches 1 ulp, tolerance passes it
// Exit code 0 = match, 1 = divergence, 2 = usage or file error.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace {

const int kF = 13;

struct Frame {
  std::vector<float> state;
  std::vector<int> evInts;
  std::vector<float> evFloats;
  std::uint64_t hash = 0;
};

struct Rec {
  std::string backend, scene;
  std::uint32_t bodies = 0;
  std::vector<Frame> frames;
  std::uint64_t finalHash = 0;
};

bool readU32(FILE* f, std::uint32_t& v) { return fread(&v, 4, 1, f) == 1; }
bool readStr(FILE* f, std::string& s) {
  std::uint32_t n;
  if (!readU32(f, n) || n > (1u << 20)) return false;
  s.resize(n);
  return n == 0 || fread(&s[0], 1, n, f) == n;
}

bool load(const char* path, Rec& r) {
  FILE* f = fopen(path, "rb");
  if (!f) return false;
  char magic[4];
  std::uint32_t ver, nframes;
  bool ok = fread(magic, 1, 4, f) == 4 && !std::memcmp(magic, "BNR1", 4) && readU32(f, ver) &&
            readStr(f, r.backend) && readStr(f, r.scene) && readU32(f, r.bodies) && readU32(f, nframes);
  for (std::uint32_t i = 0; ok && i < nframes; ++i) {
    Frame fr;
    fr.state.resize((size_t)r.bodies * kF);
    std::uint32_t ne;
    ok = fread(fr.state.data(), sizeof(float), fr.state.size(), f) == fr.state.size() && readU32(f, ne);
    fr.evInts.resize(ne * 2);
    fr.evFloats.resize(ne * 9);
    for (std::uint32_t e = 0; ok && e < ne; ++e)
      ok = fread(&fr.evInts[e * 2], sizeof(int), 2, f) == 2 && fread(&fr.evFloats[e * 9], sizeof(float), 9, f) == 9;
    ok = ok && fread(&fr.hash, 8, 1, f) == 1;
    r.frames.push_back(fr);
  }
  ok = ok && fread(&r.finalHash, 8, 1, f) == 1;
  fclose(f);
  return ok;
}

const char* kField[kF] = {"pos.x", "pos.y", "pos.z", "rot.x", "rot.y", "rot.z", "rot.w",
                          "lin.x", "lin.y", "lin.z", "ang.x", "ang.y", "ang.z"};

// Returns frame index of first divergence, or -1 when within tolerance.
int compare(const Rec& g, const Rec& c, bool exact, float tp, float tr, float tv, bool verbose) {
  if (g.bodies != c.bodies || g.frames.size() != c.frames.size()) {
    if (verbose) std::printf("structure differs: bodies %u vs %u, frames %zu vs %zu\n", g.bodies, c.bodies,
                             g.frames.size(), c.frames.size());
    return 0;
  }
  for (size_t f = 0; f < g.frames.size(); ++f) {
    const Frame& a = g.frames[f];
    const Frame& b = c.frames[f];
    for (std::uint32_t body = 0; body < g.bodies; ++body) {
      const float* x = &a.state[body * kF];
      const float* y = &b.state[body * kF];
      if (exact) {
        for (int k = 0; k < kF; ++k)
          if (std::memcmp(&x[k], &y[k], sizeof(float))) {
            if (verbose) std::printf("first divergence: frame %zu body %u %s %.9g vs %.9g\n", f + 1, body, kField[k], x[k], y[k]);
            return (int)f;
          }
      } else {
        const float dp = std::sqrt((x[0] - y[0]) * (x[0] - y[0]) + (x[1] - y[1]) * (x[1] - y[1]) + (x[2] - y[2]) * (x[2] - y[2]));
        float dot = std::fabs(x[3] * y[3] + x[4] * y[4] + x[5] * y[5] + x[6] * y[6]);
        if (dot > 1.0f) dot = 1.0f;
        const float da = 2.0f * std::acos(dot);
        float dv = 0;
        for (int k = 7; k < 13; ++k) dv = std::fmax(dv, std::fabs(x[k] - y[k]));
        if (dp > tp || da > tr || dv > tv) {
          if (verbose) std::printf("first divergence: frame %zu body %u pos %.3g m rot %.3g rad vel %.3g m/s\n", f + 1, body, dp, da, dv);
          return (int)f;
        }
      }
    }
    if (exact && (a.evInts != b.evInts || a.evFloats.size() != b.evFloats.size() ||
                  (a.evFloats.size() && std::memcmp(a.evFloats.data(), b.evFloats.data(), a.evFloats.size() * 4)))) {
      if (verbose) std::printf("first divergence: frame %zu contact events differ\n", f + 1);
      return (int)f;
    }
    if (!exact && a.evInts.size() != b.evInts.size()) {
      if (verbose) std::printf("first divergence: frame %zu contact event count %zu vs %zu\n", f + 1, a.evInts.size() / 2, b.evInts.size() / 2);
      return (int)f;
    }
  }
  return -1;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc == 3 && !std::strcmp(argv[1], "--selftest")) {
    Rec g;
    if (!load(argv[2], g) || g.frames.empty() || !g.bodies) { std::printf("cannot load %s\n", argv[2]); return 2; }
    Rec c = g;
    Frame& mid = c.frames[c.frames.size() / 2];
    std::uint32_t bits;
    std::memcpy(&bits, &mid.state[0], 4);
    bits += 1;  // one unit in the last place
    std::memcpy(&mid.state[0], &bits, 4);
    const bool exactCaught = compare(g, c, true, 0, 0, 0, false) >= 0;
    const bool tolPassed = compare(g, c, false, 1e-4f, 1e-3f, 1e-3f, false) < 0;
    const bool identical = compare(g, g, true, 0, 0, 0, false) < 0;
    std::printf("selftest %s: identical=%s exact_catches_1ulp=%s tolerance_passes_1ulp=%s\n", argv[2],
                identical ? "yes" : "NO", exactCaught ? "yes" : "NO", tolPassed ? "yes" : "NO");
    return identical && exactCaught && tolPassed ? 0 : 1;
  }
  if (argc != 3 && argc != 7) {
    std::fprintf(stderr, "usage: %s golden.bnr candidate.bnr [--tol POS ROT VEL]\n       %s --selftest file.bnr\n", argv[0], argv[0]);
    return 2;
  }
  Rec g, c;
  if (!load(argv[1], g) || !load(argv[2], c)) { std::printf("cannot load input\n"); return 2; }
  const bool exact = argc == 3;
  const float tp = exact ? 0 : (float)std::atof(argv[4]), tr = exact ? 0 : (float)std::atof(argv[5]),
              tv = exact ? 0 : (float)std::atof(argv[6]);
  const int d = compare(g, c, exact, tp, tr, tv, true);
  std::printf("%s %s vs %s: %s\n", g.scene.c_str(), g.backend.c_str(), c.backend.c_str(),
              d < 0 ? (exact ? "IDENTICAL" : "WITHIN TOLERANCE") : "DIVERGES");
  return d < 0 ? 0 : 1;
}
