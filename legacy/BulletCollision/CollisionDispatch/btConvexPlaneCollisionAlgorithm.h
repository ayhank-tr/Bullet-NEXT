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

#ifndef BT_CONVEX_PLANE_COLLISION_ALGORITHM_H
#define BT_CONVEX_PLANE_COLLISION_ALGORITHM_H

#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/CollisionDispatch/btCollisionCreateFunc.h"
class btlPersistentManifold;
#include "btCollisionDispatcher.h"

#include "LinearMath/btVector3.h"

/// btSphereBoxCollisionAlgorithm  provides sphere-box collision detection.
/// Other features are frame-coherency (persistent data) and collision response.
class btlConvexPlaneCollisionAlgorithm : public btlCollisionAlgorithm
{
	bool		m_ownManifold;
	btlPersistentManifold*	m_manifoldPtr;
	bool		m_isSwapped;
	int			m_numPerturbationIterations;
	int			m_minimumPointsPerturbationThreshold;

public:

	btlConvexPlaneCollisionAlgorithm(btlPersistentManifold* mf,const btlCollisionAlgorithmConstructionInfo& ci,const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap, bool isSwapped, int numPerturbationIterations,int minimumPointsPerturbationThreshold);

	virtual ~btlConvexPlaneCollisionAlgorithm();

	virtual void processCollision (const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap,const btlDispatcherInfo& dispatchInfo,btlManifoldResult* resultOut);

	void collideSingleContact (const btlQuaternion& perturbeRot, const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap,const btlDispatcherInfo& dispatchInfo,btlManifoldResult* resultOut);

	virtual btlScalar calculateTimeOfImpact(btlCollisionObject* body0,btlCollisionObject* body1,const btlDispatcherInfo& dispatchInfo,btlManifoldResult* resultOut);

	virtual	void	getAllContactManifolds(btlManifoldArray&	manifoldArray)
	{
		if (m_manifoldPtr && m_ownManifold)
		{
			manifoldArray.push_back(m_manifoldPtr);
		}
	}

	struct CreateFunc :public 	btlCollisionAlgorithmCreateFunc
	{
		int	m_numPerturbationIterations;
		int m_minimumPointsPerturbationThreshold;
			
		CreateFunc() 
			: m_numPerturbationIterations(1),
			m_minimumPointsPerturbationThreshold(0)
		{
		}
		
		virtual	btlCollisionAlgorithm* CreateCollisionAlgorithm(btlCollisionAlgorithmConstructionInfo& ci, const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(btlConvexPlaneCollisionAlgorithm));
			if (!m_swapped)
			{
				return new(mem) btlConvexPlaneCollisionAlgorithm(0,ci,body0Wrap,body1Wrap,false,m_numPerturbationIterations,m_minimumPointsPerturbationThreshold);
			} else
			{
				return new(mem) btlConvexPlaneCollisionAlgorithm(0,ci,body0Wrap,body1Wrap,true,m_numPerturbationIterations,m_minimumPointsPerturbationThreshold);
			}
		}
	};

};

#endif //BT_CONVEX_PLANE_COLLISION_ALGORITHM_H

