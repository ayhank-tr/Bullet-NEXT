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

#ifndef BT_COLLISION_ALGORITHM_H
#define BT_COLLISION_ALGORITHM_H

#include "LinearMath/btScalar.h"
#include "LinearMath/btAlignedObjectArray.h"

struct btlBroadphaseProxy;
class btlDispatcher;
class btlManifoldResult;
class btlCollisionObject;
struct btlCollisionObjectWrapper;
struct btlDispatcherInfo;
class	btlPersistentManifold;

typedef btlAlignedObjectArray<btlPersistentManifold*>	btlManifoldArray;

struct btlCollisionAlgorithmConstructionInfo
{
	btlCollisionAlgorithmConstructionInfo()
		:m_dispatcher1(0),
		m_manifold(0)
	{
	}
	btlCollisionAlgorithmConstructionInfo(btlDispatcher* dispatcher,int temp)
		:m_dispatcher1(dispatcher)
	{
		(void)temp;
	}

	btlDispatcher*	m_dispatcher1;
	btlPersistentManifold*	m_manifold;

//	int	getDispatcherId();

};


///btCollisionAlgorithm is an collision interface that is compatible with the Broadphase and btDispatcher.
///It is persistent over frames
class btlCollisionAlgorithm
{

protected:

	btlDispatcher*	m_dispatcher;

protected:
//	int	getDispatcherId();
	
public:

	btlCollisionAlgorithm() {};

	btlCollisionAlgorithm(const btlCollisionAlgorithmConstructionInfo& ci);

	virtual ~btlCollisionAlgorithm() {};

	virtual void processCollision (const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap,const btlDispatcherInfo& dispatchInfo,btlManifoldResult* resultOut) = 0;

	virtual btlScalar calculateTimeOfImpact(btlCollisionObject* body0,btlCollisionObject* body1,const btlDispatcherInfo& dispatchInfo,btlManifoldResult* resultOut) = 0;

	virtual	void	getAllContactManifolds(btlManifoldArray&	manifoldArray) = 0;
};


#endif //BT_COLLISION_ALGORITHM_H
