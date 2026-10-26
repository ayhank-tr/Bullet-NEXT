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

#ifndef BT_GHOST_OBJECT_H
#define BT_GHOST_OBJECT_H


#include "btCollisionObject.h"
#include "BulletCollision/BroadphaseCollision/btOverlappingPairCallback.h"
#include "LinearMath/btAlignedAllocator.h"
#include "BulletCollision/BroadphaseCollision/btOverlappingPairCache.h"
#include "btCollisionWorld.h"

class btlConvexShape;

class btlDispatcher;

///The btGhostObject can keep track of all objects that are overlapping
///By default, this overlap is based on the AABB
///This is useful for creating a character controller, collision sensors/triggers, explosions etc.
///We plan on adding rayTest and other queries for the btGhostObject
ATTRIBUTE_ALIGNED16(class) btlGhostObject : public btlCollisionObject
{
protected:

	btlAlignedObjectArray<btlCollisionObject*> m_overlappingObjects;

public:

	btlGhostObject();

	virtual ~btlGhostObject();

	void	convexSweepTest(const class btlConvexShape* castShape, const btlTransform& convexFromWorld, const btlTransform& convexToWorld, btlCollisionWorld::ConvexResultCallback& resultCallback, btlScalar allowedCcdPenetration = 0.f) const;

	void	rayTest(const btlVector3& rayFromWorld, const btlVector3& rayToWorld, btlCollisionWorld::RayResultCallback& resultCallback) const; 

	///this method is mainly for expert/internal use only.
	virtual void	addOverlappingObjectInternal(btlBroadphaseProxy* otherProxy, btlBroadphaseProxy* thisProxy=0);
	///this method is mainly for expert/internal use only.
	virtual void	removeOverlappingObjectInternal(btlBroadphaseProxy* otherProxy,btlDispatcher* dispatcher,btlBroadphaseProxy* thisProxy=0);

	int	getNumOverlappingObjects() const
	{
		return m_overlappingObjects.size();
	}

	btlCollisionObject*	getOverlappingObject(int index)
	{
		return m_overlappingObjects[index];
	}

	const btlCollisionObject*	getOverlappingObject(int index) const
	{
		return m_overlappingObjects[index];
	}

	btlAlignedObjectArray<btlCollisionObject*>&	getOverlappingPairs()
	{
		return m_overlappingObjects;
	}

	const btlAlignedObjectArray<btlCollisionObject*>	getOverlappingPairs() const
	{
		return m_overlappingObjects;
	}

	//
	// internal cast
	//

	static const btlGhostObject*	upcast(const btlCollisionObject* colObj)
	{
		if (colObj->getInternalType()==CO_GHOST_OBJECT)
			return (const btlGhostObject*)colObj;
		return 0;
	}
	static btlGhostObject*			upcast(btlCollisionObject* colObj)
	{
		if (colObj->getInternalType()==CO_GHOST_OBJECT)
			return (btlGhostObject*)colObj;
		return 0;
	}

};

class	btlPairCachingGhostObject : public btlGhostObject
{
	btlHashedOverlappingPairCache*	m_hashPairCache;

public:

	btlPairCachingGhostObject();

	virtual ~btlPairCachingGhostObject();

	///this method is mainly for expert/internal use only.
	virtual void	addOverlappingObjectInternal(btlBroadphaseProxy* otherProxy, btlBroadphaseProxy* thisProxy=0);

	virtual void	removeOverlappingObjectInternal(btlBroadphaseProxy* otherProxy,btlDispatcher* dispatcher,btlBroadphaseProxy* thisProxy=0);

	btlHashedOverlappingPairCache*	getOverlappingPairCache()
	{
		return m_hashPairCache;
	}

};



///The btGhostPairCallback interfaces and forwards adding and removal of overlapping pairs from the btBroadphaseInterface to btGhostObject.
class btlGhostPairCallback : public btlOverlappingPairCallback
{
	
public:
	btlGhostPairCallback()
	{
	}

	virtual ~btlGhostPairCallback()
	{
		
	}

	virtual btlBroadphasePair*	addOverlappingPair(btlBroadphaseProxy* proxy0,btlBroadphaseProxy* proxy1)
	{
		btlCollisionObject* colObj0 = (btlCollisionObject*) proxy0->m_clientObject;
		btlCollisionObject* colObj1 = (btlCollisionObject*) proxy1->m_clientObject;
		btlGhostObject* ghost0 = 		btlGhostObject::upcast(colObj0);
		btlGhostObject* ghost1 = 		btlGhostObject::upcast(colObj1);
		if (ghost0)
			ghost0->addOverlappingObjectInternal(proxy1, proxy0);
		if (ghost1)
			ghost1->addOverlappingObjectInternal(proxy0, proxy1);
		return 0;
	}

	virtual void*	removeOverlappingPair(btlBroadphaseProxy* proxy0,btlBroadphaseProxy* proxy1,btlDispatcher* dispatcher)
	{
		btlCollisionObject* colObj0 = (btlCollisionObject*) proxy0->m_clientObject;
		btlCollisionObject* colObj1 = (btlCollisionObject*) proxy1->m_clientObject;
		btlGhostObject* ghost0 = 		btlGhostObject::upcast(colObj0);
		btlGhostObject* ghost1 = 		btlGhostObject::upcast(colObj1);
		if (ghost0)
			ghost0->removeOverlappingObjectInternal(proxy1,dispatcher,proxy0);
		if (ghost1)
			ghost1->removeOverlappingObjectInternal(proxy0,dispatcher,proxy1);
		return 0;
	}

	virtual void	removeOverlappingPairsContainingProxy(btlBroadphaseProxy* /*proxy0*/,btlDispatcher* /*dispatcher*/)
	{
		btlAssert(0);
		//need to keep track of all ghost objects and call them here
		//m_hashPairCache->removeOverlappingPairsContainingProxy(proxy0,dispatcher);
	}

	

};

#endif

