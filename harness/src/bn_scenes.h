// Bullet NEXT golden harness - scene library (commit C0.8).
#ifndef BN_SCENES_H
#define BN_SCENES_H

#include "bn_world.h"

namespace bn {

struct Scene {
  const char* name;
  const char* covers;   // compatibility-surface points (1-12) this scene exercises
  int frames;
  Settings settings;
  void (*build)(World&);
  void (*drive)(World&, int frame);  // per-frame game logic (may be null)
};

const std::vector<Scene>& scenes();
const Scene* findScene(const std::string& name);

}  // namespace bn

#endif
