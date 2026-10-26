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

int btNextEnableHullClipping(btCollisionShape* shape)
{
	if (!shape)
		return 0;
	if (shape->isCompound())
	{
		btCompoundShape* c = static_cast<btCompoundShape*>(shape);
		int n = 0;
		for (int i = 0; i < c->getNumChildShapes(); ++i)
			n += btNextEnableHullClipping(c->getChildShape(i));
		return n;
	}
	if (shape->isPolyhedral())
	{
		btPolyhedralConvexShape* p = static_cast<btPolyhedralConvexShape*>(shape);
		if (!p->getConvexPolyhedron())
			p->initializePolyhedralFeatures();
		return 1;
	}
	return 0;
}

// --- Active edges -------------------------------------------------------------------------
// Heightfields keep internal-edge data from Bullet 2.85 on; copies built on an older Bullet
// (Chrono's) set this to 0.
#ifndef BT_NEXT_HEIGHTFIELD_EDGES
#define BT_NEXT_HEIGHTFIELD_EDGES 1
#endif

bool gBnFlatRegionRule = true;
static ContactAddedCallback s_previousContactAdded = 0;
static bool s_activeEdgesInstalled = false;

static bool isTriangle(const btCollisionObjectWrapper* w)
{
	return w && w->getCollisionShape()->getShapeType() == TRIANGLE_SHAPE_PROXYTYPE;
}

// The triangle's parent wrapper holds the mesh shape, also when the mesh is a compound child.
static btTriangleInfoMap* meshInfoMap(const btCollisionObjectWrapper* triWrap)
{
	const btCollisionShape* shape = triWrap->m_parent ? triWrap->m_parent->getCollisionShape() : triWrap->getCollisionObject()->getCollisionShape();
	switch (shape->getShapeType())
	{
		case SCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE:
			return const_cast<btTriangleInfoMap*>(static_cast<const btScaledBvhTriangleMeshShape*>(shape)->getChildShape()->getTriangleInfoMap());
		case TRIANGLE_MESH_SHAPE_PROXYTYPE:
			return const_cast<btTriangleInfoMap*>(static_cast<const btBvhTriangleMeshShape*>(shape)->getTriangleInfoMap());
#if BT_NEXT_HEIGHTFIELD_EDGES
		case TERRAIN_SHAPE_PROXYTYPE:
			return const_cast<btTriangleInfoMap*>(static_cast<const btHeightfieldTerrainShape*>(shape)->getTriangleInfoMap());
#endif
		default:
			return 0;
	}
}

// Jolt-style rule for flat regions: when every edge of the touched triangle joins a coplanar
// neighbour (within 5 degrees), no edge or vertex can be a real feature, so the contact normal
// must be the face normal. Mesh is body B here.
static void flatRegionNormal(btManifoldPoint& cp, const btCollisionObjectWrapper* triWrap, int partId, int index)
{
	btTriangleInfoMap* map = meshInfoMap(triWrap);
	if (!map)
		return;
	const int hash = (partId << (31 - MAX_NUM_PARTS_IN_BITS)) | index;
	const btTriangleInfo* info = map->find(hash);
	if (!info)
		return;
	const btScalar flat = btScalar(0.0873);  // 5 degrees
	if (btFabs(info->m_edgeV0V1Angle) > flat || btFabs(info->m_edgeV1V2Angle) > flat || btFabs(info->m_edgeV2V0Angle) > flat)
		return;
	const btTriangleShape* tri = static_cast<const btTriangleShape*>(triWrap->getCollisionShape());
	btVector3 n;
	tri->calcNormal(n);
	n = triWrap->getWorldTransform().getBasis() * n;
	if (n.dot(cp.m_normalWorldOnB) < 0)
		n = -n;
	cp.m_normalWorldOnB = n;
	cp.m_positionWorldOnB = cp.m_positionWorldOnA - n * cp.m_distance1;
	cp.m_localPointB = triWrap->getWorldTransform().invXform(cp.m_positionWorldOnB);
}

// Mesh-as-B convention. After the edge correction swaps the normal for the face normal, the
// point on A is still where the edge test put it. For a sphere that point is off-centre, so a
// rolling sphere gets a false approach speed (spin x lever arm) and pops off the mesh. Rebuild
// the sphere contact exactly from the face plane instead.
static void sphereOnFace(btManifoldPoint& cp, const btCollisionObjectWrapper* triWrap, const btCollisionObjectWrapper* otherWrap)
{
	if (otherWrap->getCollisionShape()->getShapeType() != SPHERE_SHAPE_PROXYTYPE)
		return;
	const btTriangleShape* tri = static_cast<const btTriangleShape*>(triWrap->getCollisionShape());
	btVector3 v0;
	tri->getVertex(0, v0);
	v0 = triWrap->getWorldTransform() * v0;
	const btVector3& n = cp.m_normalWorldOnB;
	const btVector3 c = otherWrap->getWorldTransform().getOrigin();
	const btScalar r = static_cast<const btSphereShape*>(otherWrap->getCollisionShape())->getRadius();
	const btScalar h = (c - v0).dot(n);
	cp.m_distance1 = h - r;
	cp.m_positionWorldOnB = c - n * h;
	cp.m_positionWorldOnA = c - n * r;
	cp.m_localPointB = triWrap->getWorldTransform().invXform(cp.m_positionWorldOnB);
	cp.m_localPointA = otherWrap->getWorldTransform().invXform(cp.m_positionWorldOnA);
}

static void correctMeshContact(btManifoldPoint& cp, const btCollisionObjectWrapper* triWrap,
							   const btCollisionObjectWrapper* otherWrap, int partId, int index)
{
	const btVector3 before = cp.m_normalWorldOnB;
	btAdjustInternalEdgeContacts(cp, triWrap, otherWrap, partId, index);
	if (gBnFlatRegionRule)
		flatRegionNormal(cp, triWrap, partId, index);
	if (before.dot(cp.m_normalWorldOnB) < btScalar(0.99999))
		sphereOnFace(cp, triWrap, otherWrap);
}

static bool bnContactAdded(btManifoldPoint& cp, const btCollisionObjectWrapper* w0, int partId0, int index0,
						   const btCollisionObjectWrapper* w1, int partId1, int index1)
{
	if (isTriangle(w1))
	{
		// Mesh is body B: the utility's own convention.
		correctMeshContact(cp, w1, w0, partId1, index1);
	}
	else if (isTriangle(w0))
	{
		// Mesh is body A: flip the point into the mesh-as-B convention, correct, flip back.
		btSwap(cp.m_positionWorldOnA, cp.m_positionWorldOnB);
		btSwap(cp.m_localPointA, cp.m_localPointB);
		cp.m_normalWorldOnB = -cp.m_normalWorldOnB;
		correctMeshContact(cp, w0, w1, partId0, index0);
		btSwap(cp.m_positionWorldOnA, cp.m_positionWorldOnB);
		btSwap(cp.m_localPointA, cp.m_localPointB);
		cp.m_normalWorldOnB = -cp.m_normalWorldOnB;
	}
	if (s_previousContactAdded)
		return s_previousContactAdded(cp, w0, partId0, index0, w1, partId1, index1);
	return false;
}

// Bake internal-edge data for every mesh and heightfield in the shape (compounds recursively).
static int bakeEdgeInfo(btCollisionShape* shape)
{
	if (shape->isCompound())
	{
		btCompoundShape* c = static_cast<btCompoundShape*>(shape);
		int n = 0;
		for (int i = 0; i < c->getNumChildShapes(); ++i)
			n += bakeEdgeInfo(c->getChildShape(i));
		return n;
	}
	btBvhTriangleMeshShape* bvh = 0;
	if (shape->getShapeType() == SCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE)
		bvh = static_cast<btScaledBvhTriangleMeshShape*>(shape)->getChildShape();
	else if (shape->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE)
		bvh = static_cast<btBvhTriangleMeshShape*>(shape);
	if (bvh)
	{
		if (!bvh->getTriangleInfoMap())
		{
			btTriangleInfoMap* map = new btTriangleInfoMap();  // owned by the shape's lifetime
			btGenerateInternalEdgeInfo(bvh, map);
		}
		return 1;
	}
#if BT_NEXT_HEIGHTFIELD_EDGES
	if (shape->getShapeType() == TERRAIN_SHAPE_PROXYTYPE)
	{
		btHeightfieldTerrainShape* hf = static_cast<btHeightfieldTerrainShape*>(shape);
		if (!hf->getTriangleInfoMap())
		{
			btTriangleInfoMap* map = new btTriangleInfoMap();
			btGenerateInternalEdgeInfo(hf, map);
		}
		return 1;
	}
#endif
	return 0;
}

bool btNextEnableActiveEdges(btCollisionObject* obj)
{
	if (!obj || !obj->getCollisionShape())
		return false;
	if (bakeEdgeInfo(obj->getCollisionShape()) == 0)
		return false;

	obj->setCollisionFlags(obj->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
	if (!s_activeEdgesInstalled)
	{
		s_previousContactAdded = gContactAddedCallback;
		gContactAddedCallback = bnContactAdded;
		s_activeEdgesInstalled = true;
	}
	return true;
}

// --- CCD ------------------------------------------------------------------------------------
void btNextConfigureCcd(btRigidBody* body, bool shapeCast)
{
	btTransform ident;
	ident.setIdentity();
	btVector3 mn, mx;
	body->getCollisionShape()->getAabb(ident, mn, mx);
	const btVector3 half = (mx - mn) * btScalar(0.5);
	const btScalar minHalf = btMin(half.x(), btMin(half.y(), half.z()));
	// Sweep when a step moves the body more than half its thinnest extent.
	body->setCcdMotionThreshold(minHalf * btScalar(0.5));
	body->setCcdSweptSphereRadius(minHalf * btScalar(0.9));
	body->setCcdShapeCast(shapeCast && body->getCollisionShape()->isConvex());
}

void btNextConfigureWorld(btDiscreteDynamicsWorld* world)
{
	world->getDispatchInfo().m_deterministicOverlappingPairs = true;
	world->getDispatchInfo().m_enableSatConvex = true;
	world->getDispatchInfo().m_useContinuous = true;
	gBnMeshContactReduction = true;  // C1.6b
	// Bullet 2.84 (Blender 2.79b) recovered deep penetration (split impulse) with erp2 = 0.8;
	// upstream lowered it to 0.2, which leaves resting bodies up to 4x deeper. Keep 2.79b's value.
	world->getSolverInfo().m_erp2 = btScalar(0.8);
}
