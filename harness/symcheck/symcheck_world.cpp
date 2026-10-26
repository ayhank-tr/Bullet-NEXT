// T0.2 symbol isolation: compiled once against Bullet NEXT and once (renamed) against the
// legacy backend. Both copies are linked into one program by symcheck_main.cpp.
#include "btBulletDynamicsCommon.h"

float BN_SYM_FN() {
  btDefaultCollisionConfiguration config;
  btCollisionDispatcher dispatcher(&config);
  btDbvtBroadphase broadphase;
  btSequentialImpulseConstraintSolver solver;
  btDiscreteDynamicsWorld world(&dispatcher, &broadphase, &solver, &config);
  world.setGravity(btVector3(0, 0, -9.81f));
  btBoxShape groundShape(btVector3(10, 10, 0.5f));
  btRigidBody ground(0, 0, &groundShape);
  ground.setWorldTransform(btTransform(btQuaternion::getIdentity(), btVector3(0, 0, -0.5f)));
  world.addRigidBody(&ground);
  btSphereShape ballShape(0.5f);
  btVector3 inertia(0, 0, 0);
  ballShape.calculateLocalInertia(1.0f, inertia);
  btDefaultMotionState* ms = new btDefaultMotionState(btTransform(btQuaternion::getIdentity(), btVector3(0, 0, 3)));
  btRigidBody ball(1.0f, ms, &ballShape, inertia);
  world.addRigidBody(&ball);
  for (int i = 0; i < 120; ++i) world.stepSimulation(1.0f / 60.0f, 1, 1.0f / 60.0f);
  const float z = (float)ball.getWorldTransform().getOrigin().z();
  world.removeRigidBody(&ball);
  world.removeRigidBody(&ground);
  delete ms;
  return z;
}
