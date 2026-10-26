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

#ifndef BT_DISPATCHER_H
#define BT_DISPATCHER_H
#include "LinearMath/btScalar.h"

class btlCollisionAlgorithm;
struct btlBroadphaseProxy;
class btlRigidBody;
class	btlCollisionObject;
class btlOverlappingPairCache;
struct btlCollisionObjectWrapper;

class btlPersistentManifold;
class btlPoolAllocator;

struct btlDispatcherInfo
{
	enum DispatchFunc
	{
		DISPATCH_DISCRETE = 1,
		DISPATCH_CONTINUOUS
	};
	btlDispatcherInfo()
		:m_timeStep(btlScalar(0.)),
		m_stepCount(0),
		m_dispatchFunc(DISPATCH_DISCRETE),
		m_timeOfImpact(btlScalar(1.)),
		m_useContinuous(true),
		m_debugDraw(0),
		m_enableSatConvex(false),
		m_enableSPU(true),
		m_useEpa(true),
		m_allowedCcdPenetration(btlScalar(0.04)),
		m_useConvexConservativeDistanceUtil(false),
		m_convexConservativeDistanceThreshold(0.0f)
	{

	}
	btlScalar	m_timeStep;
	int			m_stepCount;
	int			m_dispatchFunc;
	mutable btlScalar	m_timeOfImpact;
	bool		m_useContinuous;
	class btlIDebugDraw*	m_debugDraw;
	bool		m_enableSatConvex;
	bool		m_enableSPU;
	bool		m_useEpa;
	btlScalar	m_allowedCcdPenetration;
	bool		m_useConvexConservativeDistanceUtil;
	btlScalar	m_convexConservativeDistanceThreshold;
};

///The btDispatcher interface class can be used in combination with broadphase to dispatch calculations for overlapping pairs.
///For example for pairwise collision detection, calculating contact points stored in btPersistentManifold or user callbacks (game logic).
class btlDispatcher
{


public:
	virtual ~btlDispatcher() ;

	virtual btlCollisionAlgorithm* findAlgorithm(const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap,btlPersistentManifold* sharedManifold=0) = 0;

	virtual btlPersistentManifold*	getNewManifold(const btlCollisionObject* b0,const btlCollisionObject* b1)=0;

	virtual void releaseManifold(btlPersistentManifold* manifold)=0;

	virtual void clearManifold(btlPersistentManifold* manifold)=0;

	virtual bool	needsCollision(const btlCollisionObject* body0,const btlCollisionObject* body1) = 0;

	virtual bool	needsResponse(const btlCollisionObject* body0,const btlCollisionObject* body1)=0;

	virtual void	dispatchAllCollisionPairs(btlOverlappingPairCache* pairCache,const btlDispatcherInfo& dispatchInfo,btlDispatcher* dispatcher)  =0;

	virtual int getNumManifolds() const = 0;

	virtual btlPersistentManifold* getManifoldByIndexInternal(int index) = 0;

	virtual	btlPersistentManifold**	getInternalManifoldPointer() = 0;

	virtual	btlPoolAllocator*	getInternalManifoldPool() = 0;

	virtual	const btlPoolAllocator*	getInternalManifoldPool() const = 0;

	virtual	void* allocateCollisionAlgorithm(int size)  = 0;

	virtual	void freeCollisionAlgorithm(void* ptr) = 0;

};


#endif //BT_DISPATCHER_H
