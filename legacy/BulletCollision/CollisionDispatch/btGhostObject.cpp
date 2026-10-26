/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2008 Erwin Coumans  http://bulletphysics.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "btGhostObject.h"
#include "btCollisionWorld.h"
#include "BulletCollision/CollisionShapes/btConvexShape.h"
#include "LinearMath/btAabbUtil2.h"

btlGhostObject::btlGhostObject()
{
	m_internalType = CO_GHOST_OBJECT;
}

btlGhostObject::~btlGhostObject()
{
	///btGhostObject should have been removed from the world, so no overlapping objects
	btlAssert(!m_overlappingObjects.size());
}


void btlGhostObject::addOverlappingObjectInternal(btlBroadphaseProxy* otherProxy,btlBroadphaseProxy* thisProxy)
{
	btlCollisionObject* otherObject = (btlCollisionObject*)otherProxy->m_clientObject;
	btlAssert(otherObject);
	///if this linearSearch becomes too slow (too many overlapping objects) we should add a more appropriate data structure
	int index = m_overlappingObjects.findLinearSearch(otherObject);
	if (index==m_overlappingObjects.size())
	{
		//not found
		m_overlappingObjects.push_back(otherObject);
	}
}

void btlGhostObject::removeOverlappingObjectInternal(btlBroadphaseProxy* otherProxy,btlDispatcher* dispatcher,btlBroadphaseProxy* thisProxy)
{
	btlCollisionObject* otherObject = (btlCollisionObject*)otherProxy->m_clientObject;
	btlAssert(otherObject);
	int index = m_overlappingObjects.findLinearSearch(otherObject);
	if (index<m_overlappingObjects.size())
	{
		m_overlappingObjects[index] = m_overlappingObjects[m_overlappingObjects.size()-1];
		m_overlappingObjects.pop_back();
	}
}


btlPairCachingGhostObject::btlPairCachingGhostObject()
{
	m_hashPairCache = new (btlAlignedAlloc(sizeof(btlHashedOverlappingPairCache),16)) btlHashedOverlappingPairCache();
}

btlPairCachingGhostObject::~btlPairCachingGhostObject()
{
	m_hashPairCache->~btlHashedOverlappingPairCache();
	btlAlignedFree( m_hashPairCache );
}

void btlPairCachingGhostObject::addOverlappingObjectInternal(btlBroadphaseProxy* otherProxy,btlBroadphaseProxy* thisProxy)
{
	btlBroadphaseProxy*actualThisProxy = thisProxy ? thisProxy : getBroadphaseHandle();
	btlAssert(actualThisProxy);

	btlCollisionObject* otherObject = (btlCollisionObject*)otherProxy->m_clientObject;
	btlAssert(otherObject);
	int index = m_overlappingObjects.findLinearSearch(otherObject);
	if (index==m_overlappingObjects.size())
	{
		m_overlappingObjects.push_back(otherObject);
		m_hashPairCache->addOverlappingPair(actualThisProxy,otherProxy);
	}
}

void btlPairCachingGhostObject::removeOverlappingObjectInternal(btlBroadphaseProxy* otherProxy,btlDispatcher* dispatcher,btlBroadphaseProxy* thisProxy1)
{
	btlCollisionObject* otherObject = (btlCollisionObject*)otherProxy->m_clientObject;
	btlBroadphaseProxy* actualThisProxy = thisProxy1 ? thisProxy1 : getBroadphaseHandle();
	btlAssert(actualThisProxy);

	btlAssert(otherObject);
	int index = m_overlappingObjects.findLinearSearch(otherObject);
	if (index<m_overlappingObjects.size())
	{
		m_overlappingObjects[index] = m_overlappingObjects[m_overlappingObjects.size()-1];
		m_overlappingObjects.pop_back();
		m_hashPairCache->removeOverlappingPair(actualThisProxy,otherProxy,dispatcher);
	}
}


void	btlGhostObject::convexSweepTest(const btlConvexShape* castShape, const btlTransform& convexFromWorld, const btlTransform& convexToWorld, btlCollisionWorld::ConvexResultCallback& resultCallback, btlScalar allowedCcdPenetration) const
{
	btlTransform	convexFromTrans,convexToTrans;
	convexFromTrans = convexFromWorld;
	convexToTrans = convexToWorld;
	btlVector3 castShapeAabbMin, castShapeAabbMax;
	/* Compute AABB that encompasses angular movement */
	{
		btlVector3 linVel, angVel;
		btlTransformUtil::calculateVelocity (convexFromTrans, convexToTrans, 1.0, linVel, angVel);
		btlTransform R;
		R.setIdentity ();
		R.setRotation (convexFromTrans.getRotation());
		castShape->calculateTemporalAabb (R, linVel, angVel, 1.0, castShapeAabbMin, castShapeAabbMax);
	}

	/// go over all objects, and if the ray intersects their aabb + cast shape aabb,
	// do a ray-shape query using convexCaster (CCD)
	int i;
	for (i=0;i<m_overlappingObjects.size();i++)
	{
		btlCollisionObject*	collisionObject= m_overlappingObjects[i];
		//only perform raycast if filterMask matches
		if(resultCallback.needsCollision(collisionObject->getBroadphaseHandle())) {
			//RigidcollisionObject* collisionObject = ctrl->GetRigidcollisionObject();
			btlVector3 collisionObjectAabbMin,collisionObjectAabbMax;
			collisionObject->getCollisionShape()->getAabb(collisionObject->getWorldTransform(),collisionObjectAabbMin,collisionObjectAabbMax);
			AabbExpand (collisionObjectAabbMin, collisionObjectAabbMax, castShapeAabbMin, castShapeAabbMax);
			btlScalar hitLambda = btlScalar(1.); //could use resultCallback.m_closestHitFraction, but needs testing
			btlVector3 hitNormal;
			if (btlRayAabb(convexFromWorld.getOrigin(),convexToWorld.getOrigin(),collisionObjectAabbMin,collisionObjectAabbMax,hitLambda,hitNormal))
			{
				btlCollisionWorld::objectQuerySingle(castShape, convexFromTrans,convexToTrans,
					collisionObject,
						collisionObject->getCollisionShape(),
						collisionObject->getWorldTransform(),
						resultCallback,
						allowedCcdPenetration);
			}
		}
	}

}

void	btlGhostObject::rayTest(const btlVector3& rayFromWorld, const btlVector3& rayToWorld, btlCollisionWorld::RayResultCallback& resultCallback) const
{
	btlTransform rayFromTrans;
	rayFromTrans.setIdentity();
	rayFromTrans.setOrigin(rayFromWorld);
	btlTransform  rayToTrans;
	rayToTrans.setIdentity();
	rayToTrans.setOrigin(rayToWorld);


	int i;
	for (i=0;i<m_overlappingObjects.size();i++)
	{
		btlCollisionObject*	collisionObject= m_overlappingObjects[i];
		//only perform raycast if filterMask matches
		if(resultCallback.needsCollision(collisionObject->getBroadphaseHandle())) 
		{
			btlCollisionWorld::rayTestSingle(rayFromTrans,rayToTrans,
							collisionObject,
								collisionObject->getCollisionShape(),
								collisionObject->getWorldTransform(),
								resultCallback);
		}
	}
}

