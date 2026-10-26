/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_COLLISION__DISPATCHER_H
#define BT_COLLISION__DISPATCHER_H

#include "BulletCollision/BroadphaseCollision/btDispatcher.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"

#include "BulletCollision/CollisionDispatch/btManifoldResult.h"

#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "LinearMath/btAlignedObjectArray.h"

class btlIDebugDraw;
class btlOverlappingPairCache;
class btlPoolAllocator;
class btlCollisionConfiguration;

#include "btCollisionCreateFunc.h"

#define USE_DISPATCH_REGISTRY_ARRAY 1

class btlCollisionDispatcher;
///user can override this nearcallback for collision filtering and more finegrained control over collision detection
typedef void (*btlNearCallback)(btlBroadphasePair& collisionPair, btlCollisionDispatcher& dispatcher, const btlDispatcherInfo& dispatchInfo);


///btCollisionDispatcher supports algorithms that handle ConvexConvex and ConvexConcave collision pairs.
///Time of Impact, Closest Points and Penetration Depth.
class btlCollisionDispatcher : public btlDispatcher
{

protected:

	int		m_dispatcherFlags;

	btlAlignedObjectArray<btlPersistentManifold*>	m_manifoldsPtr;

	btlManifoldResult	m_defaultManifoldResult;

	btlNearCallback		m_nearCallback;
	
	btlPoolAllocator*	m_collisionAlgorithmPoolAllocator;

	btlPoolAllocator*	m_persistentManifoldPoolAllocator;

	btlCollisionAlgorithmCreateFunc* m_doubleDispatch[MAX_BROADPHASE_COLLISION_TYPES][MAX_BROADPHASE_COLLISION_TYPES];

	btlCollisionConfiguration*	m_collisionConfiguration;


public:

	enum DispatcherFlags
	{
		CD_STATIC_STATIC_REPORTED = 1,
		CD_USE_RELATIVE_CONTACT_BREAKING_THRESHOLD = 2,
		CD_DISABLE_CONTACTPOOL_DYNAMIC_ALLOCATION = 4
	};

	int	getDispatcherFlags() const
	{
		return m_dispatcherFlags;
	}

	void	setDispatcherFlags(int flags)
	{
		m_dispatcherFlags = flags;
	}

	///registerCollisionCreateFunc allows registration of custom/alternative collision create functions
	void	registerCollisionCreateFunc(int proxyType0,int proxyType1, btlCollisionAlgorithmCreateFunc* createFunc);

	int	getNumManifolds() const
	{ 
		return int( m_manifoldsPtr.size());
	}

	btlPersistentManifold**	getInternalManifoldPointer()
	{
		return m_manifoldsPtr.size()? &m_manifoldsPtr[0] : 0;
	}

	 btlPersistentManifold* getManifoldByIndexInternal(int index)
	{
		return m_manifoldsPtr[index];
	}

	 const btlPersistentManifold* getManifoldByIndexInternal(int index) const
	{
		return m_manifoldsPtr[index];
	}

	btlCollisionDispatcher (btlCollisionConfiguration* collisionConfiguration);

	virtual ~btlCollisionDispatcher();

	virtual btlPersistentManifold*	getNewManifold(const btlCollisionObject* b0,const btlCollisionObject* b1);
	
	virtual void releaseManifold(btlPersistentManifold* manifold);


	virtual void clearManifold(btlPersistentManifold* manifold);

	btlCollisionAlgorithm* findAlgorithm(const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap,btlPersistentManifold* sharedManifold = 0);
		
	virtual bool	needsCollision(const btlCollisionObject* body0,const btlCollisionObject* body1);
	
	virtual bool	needsResponse(const btlCollisionObject* body0,const btlCollisionObject* body1);
	
	virtual void	dispatchAllCollisionPairs(btlOverlappingPairCache* pairCache,const btlDispatcherInfo& dispatchInfo,btlDispatcher* dispatcher) ;

	void	setNearCallback(btlNearCallback	nearCallback)
	{
		m_nearCallback = nearCallback; 
	}

	btlNearCallback	getNearCallback() const
	{
		return m_nearCallback;
	}

	//by default, Bullet will use this near callback
	static void  defaultNearCallback(btlBroadphasePair& collisionPair, btlCollisionDispatcher& dispatcher, const btlDispatcherInfo& dispatchInfo);

	virtual	void* allocateCollisionAlgorithm(int size);

	virtual	void freeCollisionAlgorithm(void* ptr);

	btlCollisionConfiguration*	getCollisionConfiguration()
	{
		return m_collisionConfiguration;
	}

	const btlCollisionConfiguration*	getCollisionConfiguration() const
	{
		return m_collisionConfiguration;
	}

	void	setCollisionConfiguration(btlCollisionConfiguration* config)
	{
		m_collisionConfiguration = config;
	}

	virtual	btlPoolAllocator*	getInternalManifoldPool()
	{
		return m_persistentManifoldPoolAllocator;
	}

	virtual	const btlPoolAllocator*	getInternalManifoldPool() const
	{
		return m_persistentManifoldPoolAllocator;
	}

};

#endif //BT_COLLISION__DISPATCHER_H

