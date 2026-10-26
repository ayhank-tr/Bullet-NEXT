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

#ifndef BT_SPHERE_BOX_COLLISION_ALGORITHM_H
#define BT_SPHERE_BOX_COLLISION_ALGORITHM_H

#include "btActivatingCollisionAlgorithm.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/CollisionDispatch/btCollisionCreateFunc.h"
class btlPersistentManifold;
#include "btCollisionDispatcher.h"

#include "LinearMath/btVector3.h"

/// btSphereBoxCollisionAlgorithm  provides sphere-box collision detection.
/// Other features are frame-coherency (persistent data) and collision response.
class btlSphereBoxCollisionAlgorithm : public btlActivatingCollisionAlgorithm
{
	bool	m_ownManifold;
	btlPersistentManifold*	m_manifoldPtr;
	bool	m_isSwapped;
	
public:

	btlSphereBoxCollisionAlgorithm(btlPersistentManifold* mf,const btlCollisionAlgorithmConstructionInfo& ci,const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap, bool isSwapped);

	virtual ~btlSphereBoxCollisionAlgorithm();

	virtual void processCollision (const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap,const btlDispatcherInfo& dispatchInfo,btlManifoldResult* resultOut);

	virtual btlScalar calculateTimeOfImpact(btlCollisionObject* body0,btlCollisionObject* body1,const btlDispatcherInfo& dispatchInfo,btlManifoldResult* resultOut);

	virtual	void	getAllContactManifolds(btlManifoldArray&	manifoldArray)
	{
		if (m_manifoldPtr && m_ownManifold)
		{
			manifoldArray.push_back(m_manifoldPtr);
		}
	}

	bool getSphereDistance( const btlCollisionObjectWrapper* boxObjWrap, btlVector3& v3PointOnBox, btlVector3& normal, btlScalar& penetrationDepth, const btlVector3& v3SphereCenter, btlScalar fRadius, btlScalar maxContactDistance );

	btlScalar getSpherePenetration( btlVector3 const &boxHalfExtent, btlVector3 const &sphereRelPos, btlVector3 &closestPoint, btlVector3& normal );
	
	struct CreateFunc :public 	btlCollisionAlgorithmCreateFunc
	{
		virtual	btlCollisionAlgorithm* CreateCollisionAlgorithm(btlCollisionAlgorithmConstructionInfo& ci, const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(btlSphereBoxCollisionAlgorithm));
			if (!m_swapped)
			{
				return new(mem) btlSphereBoxCollisionAlgorithm(0,ci,body0Wrap,body1Wrap,false);
			} else
			{
				return new(mem) btlSphereBoxCollisionAlgorithm(0,ci,body0Wrap,body1Wrap,true);
			}
		}
	};

};

#endif //BT_SPHERE_BOX_COLLISION_ALGORITHM_H

