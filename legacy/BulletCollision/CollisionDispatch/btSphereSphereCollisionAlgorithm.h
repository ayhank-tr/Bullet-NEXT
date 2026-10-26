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

#ifndef BT_SPHERE_SPHERE_COLLISION_ALGORITHM_H
#define BT_SPHERE_SPHERE_COLLISION_ALGORITHM_H

#include "btActivatingCollisionAlgorithm.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/CollisionDispatch/btCollisionCreateFunc.h"
#include "btCollisionDispatcher.h"

class btlPersistentManifold;

/// btSphereSphereCollisionAlgorithm  provides sphere-sphere collision detection.
/// Other features are frame-coherency (persistent data) and collision response.
/// Also provides the most basic sample for custom/user btCollisionAlgorithm
class btlSphereSphereCollisionAlgorithm : public btlActivatingCollisionAlgorithm
{
	bool	m_ownManifold;
	btlPersistentManifold*	m_manifoldPtr;
	
public:
	btlSphereSphereCollisionAlgorithm(btlPersistentManifold* mf,const btlCollisionAlgorithmConstructionInfo& ci,const btlCollisionObjectWrapper* col0Wrap,const btlCollisionObjectWrapper* col1Wrap);

	btlSphereSphereCollisionAlgorithm(const btlCollisionAlgorithmConstructionInfo& ci)
		: btlActivatingCollisionAlgorithm(ci) {}

	virtual void processCollision (const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap,const btlDispatcherInfo& dispatchInfo,btlManifoldResult* resultOut);

	virtual btlScalar calculateTimeOfImpact(btlCollisionObject* body0,btlCollisionObject* body1,const btlDispatcherInfo& dispatchInfo,btlManifoldResult* resultOut);

	virtual	void	getAllContactManifolds(btlManifoldArray&	manifoldArray)
	{
		if (m_manifoldPtr && m_ownManifold)
		{
			manifoldArray.push_back(m_manifoldPtr);
		}
	}
	
	virtual ~btlSphereSphereCollisionAlgorithm();

	struct CreateFunc :public 	btlCollisionAlgorithmCreateFunc
	{
		virtual	btlCollisionAlgorithm* CreateCollisionAlgorithm(btlCollisionAlgorithmConstructionInfo& ci, const btlCollisionObjectWrapper* col0Wrap,const btlCollisionObjectWrapper* col1Wrap)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(btlSphereSphereCollisionAlgorithm));
			return new(mem) btlSphereSphereCollisionAlgorithm(0,ci,col0Wrap,col1Wrap);
		}
	};

};

#endif //BT_SPHERE_SPHERE_COLLISION_ALGORITHM_H

