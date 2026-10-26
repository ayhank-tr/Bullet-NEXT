// Bullet NEXT (F1, T1.7): a box and a sphere sliding at 8 m/s over a flat static triangle mesh
// in Chrono (Bullet collision). Prints the largest upward speed after settling, which internal
// mesh edges cause ("ghost bumps"). Run with and without CH_BN_NEXT_FEATURES=1.
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/geometry/ChTriangleMeshSoup.h"
#include "chrono/collision/ChCollisionShapeTriangleMesh.h"
#include "chrono/collision/bullet/ChCollisionSystemBullet.h"
#include <cstdio>
#include <cstdlib>
using namespace chrono;
int main(int argc, char** argv) {
  ChSystemNSC sys;
  sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
  sys.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));
  auto mat = chrono_types::make_shared<ChContactMaterialNSC>();
  mat->SetFriction(0.1f);
  auto soup = chrono_types::make_shared<ChTriangleMeshSoup>();
  const int n = 40; const double h = 20.0, d = 2 * h / n;
  for (int j = 0; j < n; ++j)
    for (int i = 0; i < n; ++i) {
      ChVector3d a(-h + i * d, -h + j * d, 0), b(a.x() + d, a.y(), 0), c(a.x() + d, a.y() + d, 0), e(a.x(), a.y() + d, 0);
      soup->AddTriangle(a, b, c);
      soup->AddTriangle(a, c, e);
    }
  auto ground = chrono_types::make_shared<ChBody>();
  ground->SetFixed(true);
  ground->AddCollisionShape(chrono_types::make_shared<ChCollisionShapeTriangleMesh>(mat, soup, true, false, 0.0));
  ground->EnableCollision(true);
  sys.Add(ground);
  auto box = chrono_types::make_shared<ChBodyEasyBox>(0.6, 0.6, 0.6, 1000, false, true, mat);
  box->SetPos(ChVector3d(-15, 0.37, 0.31));
  box->SetPosDt(ChVector3d(8, 0, 0));
  sys.Add(box);
  auto ball = chrono_types::make_shared<ChBodyEasySphere>(0.3, 1000, false, true, mat);
  ball->SetPos(ChVector3d(-15, -3.13, 0.31));
  ball->SetPosDt(ChVector3d(8, 0, 0));
  sys.Add(ball);
  double bumpBox = 0, bumpBall = 0;
  for (int f = 0; f < 240; ++f) {
    sys.DoStepDynamics(1.0 / 60);
    if (f >= 10) {
      bumpBox = std::max(bumpBox, box->GetPosDt().z());
      bumpBall = std::max(bumpBall, ball->GetPosDt().z());
    }
  }
  std::printf("%.4f %.4f %.3f\n", bumpBox, bumpBall, box->GetPos().x());
}
