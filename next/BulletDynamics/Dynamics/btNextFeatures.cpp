/*
Bullet NEXT feature helpers (F1).
Copyright (c) 2026 BGE NEXT contributors. Same zlib terms as Bullet.
*/
#include "btNextFeatures.h"

#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletCollision/CollisionDispatch/btInternalEdgeUtility.h"
#include "BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h"
#include "BulletCollision/CollisionDispatch/btManifoldResult.h"
#include "BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btScaledBvhTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"
#include "BulletCollision/CollisionShapes/btTriangleInfoMap.h"
#include "BulletCollision/CollisionShapes/btPolyhedralConvexShape.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/BroadphaseCollision/btQuantizedBvh.h"
#include "BulletCollision/CollisionShapes/btTriangleShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.h"

void btNextSetLinkedCollisionWhileEnabled(bool enable)
{
	gBnLinkedCollisionWhileEnabled = enable;
}
void btNextConfigureWorld(btDiscreteDynamicsWorld* world)
{
	world->getDispatchInfo().m_deterministicOverlappingPairs = true;
	world->getDispatchInfo().m_enableSatConvex = true;
	world->getDispatchInfo().m_useContinuous = true;
	// Bullet 2.84 (Blender 2.79b) recovered deep penetration (split impulse) with erp2 = 0.8;
	// upstream lowered it to 0.2, which leaves resting bodies up to 4x deeper. Keep 2.79b's value.
	world->getSolverInfo().m_erp2 = btScalar(0.8);
}
