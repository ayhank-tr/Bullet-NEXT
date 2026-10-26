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

#ifndef BT_CONVEX_CONCAVE_COLLISION_ALGORITHM_H
#define BT_CONVEX_CONCAVE_COLLISION_ALGORITHM_H

#include "btActivatingCollisionAlgorithm.h"
#include "BulletCollision/BroadphaseCollision/btDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseInterface.h"
#include "BulletCollision/CollisionShapes/btTriangleCallback.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
class btlDispatcher;
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "btCollisionCreateFunc.h"

///For each triangle in the concave mesh that overlaps with the AABB of a convex (m_convexProxy), processTriangle is called.
class btlConvexTriangleCallback : public btlTriangleCallback
{
	const btlCollisionObjectWrapper* m_convexBodyWrap;
	const btlCollisionObjectWrapper* m_triBodyWrap;

	btlVector3	m_aabbMin;
	btlVector3	m_aabbMax ;


	btlManifoldResult* m_resultOut;
	btlDispatcher*	m_dispatcher;
	const btlDispatcherInfo* m_dispatchInfoPtr;
	btlScalar m_collisionMarginTriangle;
	
public:
int	m_triangleCount;
	
	btlPersistentManifold*	m_manifoldPtr;

	btlConvexTriangleCallback(btlDispatcher* dispatcher,const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap,bool isSwapped);

	void	setTimeStepAndCounters(btlScalar collisionMarginTriangle,const btlDispatcherInfo& dispatchInfo,const btlCollisionObjectWrapper* convexBodyWrap, const btlCollisionObjectWrapper* triBodyWrap, btlManifoldResult* resultOut);

	void	clearWrapperData()
	{
		m_convexBodyWrap = 0;
		m_triBodyWrap = 0;
	}
	virtual ~btlConvexTriangleCallback();

	virtual void processTriangle(btlVector3* triangle, int partId, int triangleIndex);
	
	void clearCache();

	SIMD_FORCE_INLINE const btlVector3& getAabbMin() const
	{
		return m_aabbMin;
	}
	SIMD_FORCE_INLINE const btlVector3& getAabbMax() const
	{
		return m_aabbMax;
	}

};




/// btConvexConcaveCollisionAlgorithm  supports collision between convex shapes and (concave) trianges meshes.
class btlConvexConcaveCollisionAlgorithm  : public btlActivatingCollisionAlgorithm
{

	bool	m_isSwapped;

	btlConvexTriangleCallback m_btConvexTriangleCallback;



public:

	btlConvexConcaveCollisionAlgorithm( const btlCollisionAlgorithmConstructionInfo& ci,const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap,bool isSwapped);

	virtual ~btlConvexConcaveCollisionAlgorithm();

	virtual void processCollision (const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap,const btlDispatcherInfo& dispatchInfo,btlManifoldResult* resultOut);

	btlScalar	calculateTimeOfImpact(btlCollisionObject* body0,btlCollisionObject* body1,const btlDispatcherInfo& dispatchInfo,btlManifoldResult* resultOut);

	virtual	void	getAllContactManifolds(btlManifoldArray&	manifoldArray);
	
	void	clearCache();

	struct CreateFunc :public 	btlCollisionAlgorithmCreateFunc
	{
		virtual	btlCollisionAlgorithm* CreateCollisionAlgorithm(btlCollisionAlgorithmConstructionInfo& ci, const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(btlConvexConcaveCollisionAlgorithm));
			return new(mem) btlConvexConcaveCollisionAlgorithm(ci,body0Wrap,body1Wrap,false);
		}
	};

	struct SwappedCreateFunc :public 	btlCollisionAlgorithmCreateFunc
	{
		virtual	btlCollisionAlgorithm* CreateCollisionAlgorithm(btlCollisionAlgorithmConstructionInfo& ci, const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(btlConvexConcaveCollisionAlgorithm));
			return new(mem) btlConvexConcaveCollisionAlgorithm(ci,body0Wrap,body1Wrap,true);
		}
	};

};

#endif //BT_CONVEX_CONCAVE_COLLISION_ALGORITHM_H
