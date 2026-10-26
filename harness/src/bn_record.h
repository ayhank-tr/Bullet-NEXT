// Bullet NEXT golden harness - recorder and metrics (commits C0.6, C0.9).
#ifndef BN_RECORD_H
#define BN_RECORD_H

#include "bn_world.h"

namespace bn {

// Per-body state per frame: position(3), rotation quaternion xyzw(4), linear(3), angular(3).
const int kFloatsPerBody = 13;

struct FrameMetrics {
  float maxPenetration = 0;   // deepest contact (positive metres)
  float sumPenetration = 0;
  float energy = 0;           // kinetic + potential of dynamic rigid bodies (J)
  float maxJointDrift = 0;    // largest pivot separation over all constraints (m)
  int contacts = 0;           // contact points in all manifolds
  double stepMicros = 0;
};

class Recorder {
 public:
  Recorder(const std::string& scene, World& w);
  void capture(World& w, const FrameMetrics& m);   // after each World::step()
  bool writeBinary(const std::string& path) const;
  bool writeMetricsCsv(const std::string& path) const;
  u64 finalHash() const { return m_final; }
  const std::vector<u64>& frameHashes() const { return m_hashes; }
  size_t frames() const { return m_hashes.size(); }

 private:
  std::string m_scene;
  int m_bodies;
  std::vector<float> m_state;          // frames * bodies * kFloatsPerBody
  std::vector<int> m_eventCounts;
  std::vector<int> m_eventInts;        // a, b per event
  std::vector<float> m_eventFloats;    // p1, p2, normal per event
  std::vector<u64> m_hashes;
  std::vector<FrameMetrics> m_metrics;
  u64 m_final = 0;
};

FrameMetrics measure(World& w);

}  // namespace bn

#endif
