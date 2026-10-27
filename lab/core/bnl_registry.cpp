// Bullet NEXT Lab - the engines linked into this program (FL, CL.2).
#include "bnl_engine.h"
#include "bnl_recording.h"

#if BNL_HAVE_LEGACY
bnl::Engine* bnl_create_engine_legacy();
void bnl_fuzz_legacy(unsigned seed, int pairs, bnl::FuzzResult& r);
#endif
#if BNL_HAVE_NEXT
bnl::Engine* bnl_create_engine_next();
void bnl_fuzz_next(unsigned seed, int pairs, bnl::FuzzResult& r);
#endif
#if BNL_HAVE_NEXTF
bnl::Engine* bnl_create_engine_nextf();
bnl::Engine* bnl_create_engine_nextf_softstep();
void bnl_fuzz_nextf(unsigned seed, int pairs, bnl::FuzzResult& r);
#endif
#if BNL_HAVE_JOLT
bnl::Engine* bnl_create_engine_jolt();
#endif

namespace bnl {

std::vector<std::string> engineNames() {
  std::vector<std::string> n;
#if BNL_HAVE_LEGACY
  n.push_back("legacy");
#endif
#if BNL_HAVE_NEXT
  n.push_back("next");
#endif
#if BNL_HAVE_NEXTF
  n.push_back("nextf");
  n.push_back("nexts");
#endif
#if BNL_HAVE_JOLT
  n.push_back("jolt");
#endif
  return n;
}

Engine* createEngine(const std::string& name) {
#if BNL_HAVE_LEGACY
  if (name == "legacy") return bnl_create_engine_legacy();
#endif
#if BNL_HAVE_NEXT
  if (name == "next") return bnl_create_engine_next();
#endif
#if BNL_HAVE_NEXTF
  if (name == "nextf") return bnl_create_engine_nextf();
  if (name == "nexts") return bnl_create_engine_nextf_softstep();
#endif
#if BNL_HAVE_JOLT
  if (name == "jolt") return bnl_create_engine_jolt();
#endif
  if (name == "replay") return createReplayEngine();
  return nullptr;
}

bool fuzzCollisionPairs(const std::string& name, unsigned seed, int pairs, FuzzResult& r) {
#if BNL_HAVE_LEGACY
  if (name == "legacy") { bnl_fuzz_legacy(seed, pairs, r); return true; }
#endif
#if BNL_HAVE_NEXT
  if (name == "next") { bnl_fuzz_next(seed, pairs, r); return true; }
#endif
#if BNL_HAVE_NEXTF
  if (name == "nextf") { bnl_fuzz_nextf(seed, pairs, r); return true; }
#endif
  (void)seed; (void)pairs; (void)r;
  return false;
}

}  // namespace bnl
