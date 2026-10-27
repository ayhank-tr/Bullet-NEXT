// Bullet NEXT Lab - recordings of any Lab engine, and their playback as an engine (FL, CL.8).
// A recording keeps, per frame, every body's state, the soft-body nodes, the wheels and the frame
// hash, so a run made in another program (orig, upstream, Blender's double-precision Bullet,
// Chrono) can be watched in the viewer like a live engine: createEngine("replay") with
// EngineOptions::recording set to the file.
#ifndef BNL_RECORDING_H
#define BNL_RECORDING_H

#include "bnl_engine.h"

#include <string>
#include <vector>

namespace bnl {

struct RecordedFrame {
  std::vector<BodyState> bodies;
  std::vector<std::vector<Vec3> > soft;     // per body (empty unless soft)
  std::vector<std::vector<Xform> > wheels;  // per vehicle
  u64 hash = 0;
};

struct Recording {
  std::string engine, scene;
  std::vector<RecordedFrame> frames;
  u64 finalHash = 0;

  void capture(const Engine& e);  // after each Engine::advance()
  bool write(const std::string& path) const;
  bool read(const std::string& path, std::string& error);
};

// Plays a recording back through the Engine interface (state only: game controls do nothing).
Engine* createReplayEngine();

}  // namespace bnl

#endif
