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

#ifndef BT_SOFT_BODY_CONCAVE_COLLISION_ALGORITHM_H
#define BT_SOFT_BODY_CONCAVE_COLLISION_ALGORITHM_H

#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"
#include "BulletCollision/BroadphaseCollision/btDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseInterface.h"
#include "BulletCollision/CollisionShapes/btTriangleCallback.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
class btlDispatcher;
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/CollisionDispatch/btCollisionCreateFunc.h"
class btlSoftBody;
class btlCollisionShape;

#include "LinearMath/btHashMap.h"

#include "BulletCollision/BroadphaseCollision/btQuantizedBvh.h" //for definition of MAX_NUM_PARTS_IN_BITS

struct btlTriIndex
{
	int m_PartIdTriangleIndex;
	class btlCollisionShape*	m_childShape;

	btlTriIndex(int partId,int triangleIndex,btlCollisionShape* shape)
	{
		m_PartIdTriangleIndex = (partId<<(31-MAX_NUM_PARTS_IN_BITS)) | triangleIndex;
		m_childShape = shape;
	}

	int	getTriangleIndex() const
	{
		// Get only the lower bits where the triangle index is stored
		unsigned int x = 0;
		unsigned int y = (~(x&0))<<(31-MAX_NUM_PARTS_IN_BITS);
		return (m_PartIdTriangleIndex&~(y));
	}
	int	getPartId() const
	{
		// Get only the highest bits where the part index is stored
		return (m_PartIdTriangleIndex>>(31-MAX_NUM_PARTS_IN_BITS));
	}
	int	getUid() const
	{
		return m_PartIdTriangleIndex;
	}
};


///For each triangle in the concave mesh that overlaps with the AABB of a soft body (m_softBody), processTriangle is called.
class btlSoftBodyTriangleCallback : public btlTriangleCallback
{
	btlSoftBody* m_softBody;
	const btlCollisionObject* m_triBody;

	btlVector3	m_aabbMin;
	btlVector3	m_aabbMax ;

	btlManifoldResult* m_resultOut;

	btlDispatcher*	m_dispatcher;
	const btlDispatcherInfo* m_dispatchInfoPtr;
	btlScalar m_collisionMarginTriangle;

	btlHashMap<btlHashKey<btlTriIndex>,btlTriIndex> m_shapeCache;

public:
	int	m_triangleCount;

	//	btPersistentManifold*	m_manifoldPtr;

	btlSoftBodyTriangleCallback(btlDispatcher* dispatcher,const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap,bool isSwapped);

	void	setTimeStepAndCounters(btlScalar collisionMarginTriangle,const btlCollisionObjectWrapper* triObjWrap,const btlDispatcherInfo& dispatchInfo,btlManifoldResult* resultOut);

	virtual ~btlSoftBodyTriangleCallback();

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




/// btSoftBodyConcaveCollisionAlgorithm  supports collision between soft body shapes and (concave) trianges meshes.
class btlSoftBodyConcaveCollisionAlgorithm  : public btlCollisionAlgorithm
{

	bool	m_isSwapped;

	btlSoftBodyTriangleCallback m_btSoftBodyTriangleCallback;

public:

	btlSoftBodyConcaveCollisionAlgorithm( const btlCollisionAlgorithmConstructionInfo& ci,const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap,bool isSwapped);

	virtual ~btlSoftBodyConcaveCollisionAlgorithm();

	virtual void processCollision (const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap,const btlDispatcherInfo& dispatchInfo,btlManifoldResult* resultOut);

	btlScalar	calculateTimeOfImpact(btlCollisionObject* body0,btlCollisionObject* body1,const btlDispatcherInfo& dispatchInfo,btlManifoldResult* resultOut);

	virtual	void	getAllContactManifolds(btlManifoldArray&	manifoldArray)
	{
		//we don't add any manifolds
	}

	void	clearCache();

	struct CreateFunc :public 	btlCollisionAlgorithmCreateFunc
	{
		virtual	btlCollisionAlgorithm* CreateCollisionAlgorithm(btlCollisionAlgorithmConstructionInfo& ci, const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(btlSoftBodyConcaveCollisionAlgorithm));
			return new(mem) btlSoftBodyConcaveCollisionAlgorithm(ci,body0Wrap,body1Wrap,false);
		}
	};

	struct SwappedCreateFunc :public 	btlCollisionAlgorithmCreateFunc
	{
		virtual	btlCollisionAlgorithm* CreateCollisionAlgorithm(btlCollisionAlgorithmConstructionInfo& ci, const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(btlSoftBodyConcaveCollisionAlgorithm));
			return new(mem) btlSoftBodyConcaveCollisionAlgorithm(ci,body0Wrap,body1Wrap,true);
		}
	};

};

#endif //BT_SOFT_BODY_CONCAVE_COLLISION_ALGORITHM_H
