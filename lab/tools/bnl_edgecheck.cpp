// Bullet NEXT Lab - heightfield edge data check and benchmark (Stage 1: T1S.1, T1S.3, B1S.1).
//
//   bnl_edgecheck check <scene.json>   every heightfield in the scene: the edge data worked out from
//                                      the heights (what btNextEnableActiveEdges now uses) against
//                                      Bullet's bake, triangle by triangle
//   bnl_edgecheck bench <samples>      a rolling field of samples x samples at 1 m: time and memory
//                                      to switch active edges on, baked and worked out
//
// Prints one JSON object per line.
#include "bnl_scene.h"

#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"
#include "BulletCollision/CollisionShapes/btTriangleInfoMap.h"
#include "BulletCollision/CollisionDispatch/btInternalEdgeUtility.h"
#include "BulletDynamics/Dynamics/btNextFeatures.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

namespace {

long rssKb() {
  std::ifstream f("/proc/self/status");
  std::string l;
  while (std::getline(f, l))
    if (l.rfind("VmRSS:", 0) == 0) return std::atol(l.c_str() + 6);
  return 0;  // not Linux: memory not measured
}

double now() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }

// The Lab's own heightfield (harness/src/bn_world.cpp, makeHeightfield): x fastest, z up, the
// spacing as local scaling.
btHeightfieldTerrainShape* makeField(int nx, int ny, float spacing, const std::vector<float>& h) {
  float lo = h[0], hi = h[0];
  for (float v : h) { lo = v < lo ? v : lo; hi = v > hi ? v : hi; }
  btHeightfieldTerrainShape* s = new btHeightfieldTerrainShape(nx, ny, h.data(), 1.0f, lo, hi, 2, PHY_FLOAT, false);
  s->setLocalScaling(btVector3(spacing, spacing, 1.0f));
  return s;
}

struct Compare : public btTriangleCallback {
  const btTriangleInfoMap* baked = 0;
  const btTriangleInfoMap* computed = 0;
  long triangles = 0, withData = 0, mismatches = 0;
  double maxDiff = 0;
  void processTriangle(btVector3* t, int partId, int index) override {
    ++triangles;
    btTriangleInfo scratch;
    const btTriangleInfo* a = baked->find(baked->bnKey(partId, index));
    const btTriangleInfo* b = btNextFindTriangleInfo(computed, t, partId, index, scratch);
    if (!a || !b) {
      if (a || b) ++mismatches;
      return;
    }
    ++withData;
    const double d = std::max(std::fabs((double)a->m_edgeV0V1Angle - b->m_edgeV0V1Angle),
                              std::max(std::fabs((double)a->m_edgeV1V2Angle - b->m_edgeV1V2Angle),
                                       std::fabs((double)a->m_edgeV2V0Angle - b->m_edgeV2V0Angle)));
    maxDiff = d > maxDiff ? d : maxDiff;
    if (a->m_flags != b->m_flags || d > 1e-5) ++mismatches;
  }
};

int check(const char* path) {
  bnl::SceneDesc scene;
  std::string err;
  if (!bnl::loadScene(path, scene, err)) { std::fprintf(stderr, "%s\n", err.c_str()); return 2; }
  int bad = 0, fields = 0;
  for (bnl::BodyDesc& b : scene.bodies) {
    if (b.shape.type != bnl::ShapeDesc::kHeightfield) continue;
    if (b.shape.heights.empty()) bnl::generateHeights(b.shape);
    btHeightfieldTerrainShape* bakedShape = makeField(b.shape.samplesX, b.shape.samplesY, b.shape.spacing, b.shape.heights);
    btHeightfieldTerrainShape* computedShape = makeField(b.shape.samplesX, b.shape.samplesY, b.shape.spacing, b.shape.heights);
    btTriangleInfoMap* bakedMap = new btTriangleInfoMap();
    btGenerateInternalEdgeInfo(bakedShape, bakedMap);
    btCollisionObject obj;
    obj.setCollisionShape(computedShape);
    btNextEnableActiveEdges(&obj);
    Compare c;
    c.baked = bakedMap;
    c.computed = computedShape->getTriangleInfoMap();
    const btVector3 big(BT_LARGE_FLOAT, BT_LARGE_FLOAT, BT_LARGE_FLOAT);
    computedShape->processAllTriangles(&c, -big, big);
    const bool ok = c.mismatches == 0 && c.triangles > 0 && c.computed && c.computed->size() == 0 && c.computed->m_bnHeightfield;
    std::printf("{\"scene\": \"%s\", \"body\": \"%s\", \"samples\": [%d, %d], \"triangles\": %ld, \"with_edge_data\": %ld, "
                "\"baked_entries\": %d, \"computed_map_entries\": %d, \"max_angle_diff_rad\": %.3g, \"mismatches\": %ld, \"ok\": %s}\n",
                scene.name.c_str(), b.name.c_str(), b.shape.samplesX, b.shape.samplesY, c.triangles, c.withData,
                bakedMap->size(), c.computed ? c.computed->size() : -1, c.maxDiff, c.mismatches, ok ? "true" : "false");
    bad += ok ? 0 : 1;
    ++fields;
    delete bakedMap;
    delete bakedShape;
    delete computedShape;  // deletes its own (computed) map
  }
  return fields == 0 ? 2 : (bad ? 1 : 0);
}

int bench(int n) {
  bnl::ShapeDesc s;
  s.type = bnl::ShapeDesc::kHeightfield;
  s.samplesX = s.samplesY = n;
  s.spacing = 1.0f;
  s.terrain.kind = "rolling";
  s.terrain.amplitude = 3.0f;
  s.terrain.wavelength = 120.0f;
  s.terrain.octaves = 3;
  s.terrain.seed = 7;
  bnl::generateHeights(s);
  for (int computed = 0; computed < 2; ++computed) {
    btHeightfieldTerrainShape* shape = makeField(n, n, 1.0f, s.heights);
    btCollisionObject obj;
    obj.setCollisionShape(shape);
    const long r0 = rssKb();
    const double t0 = now();
    if (computed) {
      btNextEnableActiveEdges(&obj);
    } else {
      btTriangleInfoMap* map = new btTriangleInfoMap();
      map->m_bnOwnedByShape = true;
      btGenerateInternalEdgeInfo(shape, map);
    }
    const double t = now() - t0;
    const long r1 = rssKb();
    std::printf("{\"samples\": %d, \"triangles\": %ld, \"edges\": \"%s\", \"seconds\": %.4f, \"extra_memory_mb\": %.1f, \"map_entries\": %d}\n",
                n, 2L * (n - 1) * (n - 1), computed ? "worked out" : "baked", t, (r1 - r0) / 1024.0,
                shape->getTriangleInfoMap()->size());
    std::fflush(stdout);
    delete shape;
  }
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc == 3 && !std::strcmp(argv[1], "check")) return check(argv[2]);
  if (argc == 3 && !std::strcmp(argv[1], "bench")) return bench(std::atoi(argv[2]));
  std::fprintf(stderr, "usage: bnl_edgecheck check <scene.json> | bench <samples>\n");
  return 2;
}
