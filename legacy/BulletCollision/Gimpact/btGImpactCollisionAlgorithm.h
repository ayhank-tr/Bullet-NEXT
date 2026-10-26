/*! \file btGImpactShape.h
\author Francisco Leon Najera
*/
/*
This source file is part of GIMPACT Library.

For the latest info, see http://gimpact.sourceforge.net/

Copyright (c) 2007 Francisco Leon Najera. C.C. 80087371.
email: projectileman@yahoo.com


This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_GIMPACT_BVH_CONCAVE_COLLISION_ALGORITHM_H
#define BT_GIMPACT_BVH_CONCAVE_COLLISION_ALGORITHM_H

#include "BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.h"
#include "BulletCollision/BroadphaseCollision/btDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseInterface.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
class btlDispatcher;
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/CollisionDispatch/btCollisionCreateFunc.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"

#include "LinearMath/btAlignedObjectArray.h"

#include "btGImpactShape.h"
#include "BulletCollision/CollisionShapes/btStaticPlaneShape.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.h"
#include "LinearMath/btIDebugDraw.h"
#include "BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h"


//! Collision Algorithm for GImpact Shapes
/*!
For register this algorithm in Bullet, proceed as following:
 \code
btCollisionDispatcher * dispatcher = static_cast<btCollisionDispatcher *>(m_dynamicsWorld ->getDispatcher());
btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);
 \endcode
*/
class btlGImpactCollisionAlgorithm : public btlActivatingCollisionAlgorithm
{
protected:
	btlCollisionAlgorithm * m_convex_algorithm;
    btlPersistentManifold * m_manifoldPtr;
	btlManifoldResult* m_resultOut;
	const btlDispatcherInfo * m_dispatchInfo;
	int m_triface0;
	int m_part0;
	int m_triface1;
	int m_part1;


	//! Creates a new contact point
	SIMD_FORCE_INLINE btlPersistentManifold* newContactManifold(const btlCollisionObject* body0,const btlCollisionObject* body1)
	{
		m_manifoldPtr = m_dispatcher->getNewManifold(body0,body1);
		return m_manifoldPtr;
	}

	SIMD_FORCE_INLINE void destroyConvexAlgorithm()
	{
		if(m_convex_algorithm)
		{
			m_convex_algorithm->~btlCollisionAlgorithm();
			m_dispatcher->freeCollisionAlgorithm( m_convex_algorithm);
			m_convex_algorithm = NULL;
		}
	}

	SIMD_FORCE_INLINE void destroyContactManifolds()
	{
		if(m_manifoldPtr == NULL) return;
		m_dispatcher->releaseManifold(m_manifoldPtr);
		m_manifoldPtr = NULL;
	}

	SIMD_FORCE_INLINE void clearCache()
	{
		destroyContactManifolds();
		destroyConvexAlgorithm();

		m_triface0 = -1;
		m_part0 = -1;
		m_triface1 = -1;
		m_part1 = -1;
	}

	SIMD_FORCE_INLINE btlPersistentManifold* getLastManifold()
	{
		return m_manifoldPtr;
	}


	// Call before process collision
	SIMD_FORCE_INLINE void checkManifold(const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap)
	{
		if(getLastManifold() == 0)
		{
			newContactManifold(body0Wrap->getCollisionObject(),body1Wrap->getCollisionObject());
		}

		m_resultOut->setPersistentManifold(getLastManifold());
	}

	// Call before process collision
	SIMD_FORCE_INLINE btlCollisionAlgorithm * newAlgorithm(const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap)
	{
		checkManifold(body0Wrap,body1Wrap);

		btlCollisionAlgorithm * convex_algorithm = m_dispatcher->findAlgorithm(
				body0Wrap,body1Wrap,getLastManifold());
		return convex_algorithm ;
	}

	// Call before process collision
	SIMD_FORCE_INLINE void checkConvexAlgorithm(const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap)
	{
		if(m_convex_algorithm) return;
		m_convex_algorithm = newAlgorithm(body0Wrap,body1Wrap);
	}




	void addContactPoint(const btlCollisionObjectWrapper * body0Wrap,
					const btlCollisionObjectWrapper * body1Wrap,
					const btlVector3 & point,
					const btlVector3 & normal,
					btlScalar distance);

//! Collision routines
//!@{

	void collide_gjk_triangles(const btlCollisionObjectWrapper* body0Wrap,
				  const btlCollisionObjectWrapper* body1Wrap,
				  const btlGImpactMeshShapePart * shape0,
				  const btlGImpactMeshShapePart * shape1,
				  const int * pairs, int pair_count);

	void collide_sat_triangles(const btlCollisionObjectWrapper* body0Wrap,
					  const btlCollisionObjectWrapper* body1Wrap,
					  const btlGImpactMeshShapePart * shape0,
					  const btlGImpactMeshShapePart * shape1,
					  const int * pairs, int pair_count);




	void shape_vs_shape_collision(
					  const btlCollisionObjectWrapper* body0,
					  const btlCollisionObjectWrapper* body1,
					  const btlCollisionShape * shape0,
					  const btlCollisionShape * shape1);

	void convex_vs_convex_collision(const btlCollisionObjectWrapper* body0Wrap,
					  const btlCollisionObjectWrapper* body1Wrap,
					  const btlCollisionShape* shape0,
					  const btlCollisionShape* shape1);



	void gimpact_vs_gimpact_find_pairs(
					  const btlTransform & trans0,
					  const btlTransform & trans1,
					  const btlGImpactShapeInterface * shape0,
					  const btlGImpactShapeInterface * shape1,btlPairSet & pairset);

	void gimpact_vs_shape_find_pairs(
					  const btlTransform & trans0,
					  const btlTransform & trans1,
					  const btlGImpactShapeInterface * shape0,
					  const btlCollisionShape * shape1,
					  btlAlignedObjectArray<int> & collided_primitives);


	void gimpacttrimeshpart_vs_plane_collision(
					  const btlCollisionObjectWrapper * body0Wrap,
					  const btlCollisionObjectWrapper * body1Wrap,
					  const btlGImpactMeshShapePart * shape0,
					  const btlStaticPlaneShape * shape1,bool swapped);


public:

	btlGImpactCollisionAlgorithm( const btlCollisionAlgorithmConstructionInfo& ci,const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap);

	virtual ~btlGImpactCollisionAlgorithm();

	virtual void processCollision (const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap,const btlDispatcherInfo& dispatchInfo,btlManifoldResult* resultOut);

	btlScalar	calculateTimeOfImpact(btlCollisionObject* body0,btlCollisionObject* body1,const btlDispatcherInfo& dispatchInfo,btlManifoldResult* resultOut);

	virtual	void	getAllContactManifolds(btlManifoldArray&	manifoldArray)
	{
		if (m_manifoldPtr)
			manifoldArray.push_back(m_manifoldPtr);
	}

	btlManifoldResult*	internalGetResultOut()
	{
		return m_resultOut;
	}

	struct CreateFunc :public 	btlCollisionAlgorithmCreateFunc
	{
		virtual	btlCollisionAlgorithm* CreateCollisionAlgorithm(btlCollisionAlgorithmConstructionInfo& ci, const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(btlGImpactCollisionAlgorithm));
			return new(mem) btlGImpactCollisionAlgorithm(ci,body0Wrap,body1Wrap);
		}
	};

	//! Use this function for register the algorithm externally
	static void registerAlgorithm(btlCollisionDispatcher * dispatcher);
#ifdef TRI_COLLISION_PROFILING
	//! Gets the average time in miliseconds of tree collisions
	static float getAverageTreeCollisionTime();

	//! Gets the average time in miliseconds of triangle collisions
	static float getAverageTriangleCollisionTime();
#endif //TRI_COLLISION_PROFILING

	//! Collides two gimpact shapes
	/*!
	\pre shape0 and shape1 couldn't be btGImpactMeshShape objects
	*/


	void gimpact_vs_gimpact(const btlCollisionObjectWrapper* body0Wrap,
					  const btlCollisionObjectWrapper * body1Wrap,
					  const btlGImpactShapeInterface * shape0,
					  const btlGImpactShapeInterface * shape1);

	void gimpact_vs_shape(const btlCollisionObjectWrapper* body0Wrap,
					  const btlCollisionObjectWrapper* body1Wrap,
					  const btlGImpactShapeInterface * shape0,
					  const btlCollisionShape * shape1,bool swapped);

	void gimpact_vs_compoundshape(const btlCollisionObjectWrapper * body0Wrap,
					  const btlCollisionObjectWrapper * body1Wrap,
					  const btlGImpactShapeInterface * shape0,
					  const btlCompoundShape * shape1,bool swapped);

	void gimpact_vs_concave(
					  const btlCollisionObjectWrapper * body0Wrap,
					  const btlCollisionObjectWrapper * body1Wrap,
					  const btlGImpactShapeInterface * shape0,
					  const btlConcaveShape * shape1,bool swapped);




		/// Accessor/Mutator pairs for Part and triangleID
    void 	setFace0(int value) 
    { 
    	m_triface0 = value; 
    }
    int getFace0() 
    { 
    	return m_triface0; 
    }
    void setFace1(int value) 
    { 
    	m_triface1 = value; 
    }
    int getFace1() 
    { 
    	return m_triface1; 
    }
    void setPart0(int value) 
    { 
    	m_part0 = value; 
    }
    int getPart0() 
    { 
    	return m_part0; 
    }
    void setPart1(int value) 
    { 
    	m_part1 = value; 
		}
    int getPart1() 
    { 
    	return m_part1; 
    }

};


//algorithm details
//#define BULLET_TRIANGLE_COLLISION 1
#define GIMPACT_VS_PLANE_COLLISION 1



#endif //BT_GIMPACT_BVH_CONCAVE_COLLISION_ALGORITHM_H
