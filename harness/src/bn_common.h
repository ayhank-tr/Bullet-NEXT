// Bullet NEXT golden harness - common definitions.
// This file is compiled three times: against the original Bullet 2.84 copy, against the
// renamed legacy backend (after tools/rename_legacy.py rewrites it) and against Bullet NEXT.
// Rule for harness code: never name our own identifiers with a "bt" prefix.
#ifndef BN_COMMON_H
#define BN_COMMON_H

#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btGhostObject.h"
#include "BulletDynamics/Character/btKinematicCharacterController.h"
#include "BulletCollision/NarrowPhaseCollision/btRaycastCallback.h"
#include "LinearMath/btConvexHullComputer.h"
#if BN_NEXT_FEATURES || BN_NEXT_LIB
#include "BulletDynamics/Dynamics/btNextFeatures.h"
#include "BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.h"
#include "BulletDynamics/ConstraintSolver/btNNCGConstraintSolver.h"
#include "BulletDynamics/ConstraintSolver/btSoftStepConstraintSolver.h"
#endif
#if BN_NEXT_FEATURES
#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorldMt.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolverMt.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcherMt.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorldMt.h"
#include "LinearMath/btThreads.h"
#endif

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#ifndef BN_BACKEND_NAME
#define BN_BACKEND_NAME "unknown"
#endif

// Bullet 2.84 (Blender 2.79b) reports 284; upstream master reports 326.
#if BT_BULLET_VERSION >= 286
#define BN_NEW_API 1
#else
#define BN_NEW_API 0
#endif

// Each backend's copy of the harness lives in its own inline namespace (bn::legacy, bn::next...),
// so several backends can share one program (the Lab engines, CL.2) without clashing names.
#ifndef BN_TAG
#define BN_TAG unnamed_backend
#endif
#define BN_NAMESPACE_BEGIN namespace bn { inline namespace BN_TAG {
#define BN_NAMESPACE_END } }

BN_NAMESPACE_BEGIN

typedef std::uint64_t u64;

// FNV-1a 64-bit, used for per-frame state hashes (no external dependency).
struct Hash64 {
  u64 h = 1469598103934665603ULL;
  void bytes(const void* p, size_t n) {
    const unsigned char* c = static_cast<const unsigned char*>(p);
    for (size_t i = 0; i < n; ++i) { h ^= c[i]; h *= 1099511628211ULL; }
  }
  void f(float v) { bytes(&v, sizeof(v)); }
  void i(int v) { bytes(&v, sizeof(v)); }
};

BN_NAMESPACE_END  // namespace bn

#endif
