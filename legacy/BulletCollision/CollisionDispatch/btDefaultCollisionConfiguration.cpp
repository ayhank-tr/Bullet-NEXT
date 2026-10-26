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

#include "btDefaultCollisionConfiguration.h"

#include "BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btCompoundCompoundCollisionAlgorithm.h"

#include "BulletCollision/CollisionDispatch/btConvexPlaneCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btBoxBoxCollisionAlgorithm.h"
#include "BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.h"
#ifdef USE_BUGGY_SPHERE_BOX_ALGORITHM
#include "BulletCollision/CollisionDispatch/btSphereBoxCollisionAlgorithm.h"
#endif //USE_BUGGY_SPHERE_BOX_ALGORITHM
#include "BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"



#include "LinearMath/btPoolAllocator.h"





btlDefaultCollisionConfiguration::btlDefaultCollisionConfiguration(const btlDefaultCollisionConstructionInfo& constructionInfo)
//btDefaultCollisionConfiguration::btDefaultCollisionConfiguration(btStackAlloc*	stackAlloc,btPoolAllocator*	persistentManifoldPool,btPoolAllocator*	collisionAlgorithmPool)
{

	void* mem = btlAlignedAlloc(sizeof(btlVoronoiSimplexSolver),16);
	m_simplexSolver = new (mem)btlVoronoiSimplexSolver();

	if (constructionInfo.m_useEpaPenetrationAlgorithm)
	{
		mem = btlAlignedAlloc(sizeof(btlGjkEpaPenetrationDepthSolver),16);
		m_pdSolver = new (mem)btlGjkEpaPenetrationDepthSolver;
	}else
	{
		mem = btlAlignedAlloc(sizeof(btlMinkowskiPenetrationDepthSolver),16);
		m_pdSolver = new (mem)btlMinkowskiPenetrationDepthSolver;
	}
	
	//default CreationFunctions, filling the m_doubleDispatch table
	mem = btlAlignedAlloc(sizeof(btlConvexConvexAlgorithm::CreateFunc),16);
	m_convexConvexCreateFunc = new(mem) btlConvexConvexAlgorithm::CreateFunc(m_simplexSolver,m_pdSolver);
	mem = btlAlignedAlloc(sizeof(btlConvexConcaveCollisionAlgorithm::CreateFunc),16);
	m_convexConcaveCreateFunc = new (mem)btlConvexConcaveCollisionAlgorithm::CreateFunc;
	mem = btlAlignedAlloc(sizeof(btlConvexConcaveCollisionAlgorithm::CreateFunc),16);
	m_swappedConvexConcaveCreateFunc = new (mem)btlConvexConcaveCollisionAlgorithm::SwappedCreateFunc;
	mem = btlAlignedAlloc(sizeof(btlCompoundCollisionAlgorithm::CreateFunc),16);
	m_compoundCreateFunc = new (mem)btlCompoundCollisionAlgorithm::CreateFunc;

	mem = btlAlignedAlloc(sizeof(btlCompoundCompoundCollisionAlgorithm::CreateFunc),16);
	m_compoundCompoundCreateFunc = new (mem)btlCompoundCompoundCollisionAlgorithm::CreateFunc;

	mem = btlAlignedAlloc(sizeof(btlCompoundCollisionAlgorithm::SwappedCreateFunc),16);
	m_swappedCompoundCreateFunc = new (mem)btlCompoundCollisionAlgorithm::SwappedCreateFunc;
	mem = btlAlignedAlloc(sizeof(btlEmptyAlgorithm::CreateFunc),16);
	m_emptyCreateFunc = new(mem) btlEmptyAlgorithm::CreateFunc;
	
	mem = btlAlignedAlloc(sizeof(btlSphereSphereCollisionAlgorithm::CreateFunc),16);
	m_sphereSphereCF = new(mem) btlSphereSphereCollisionAlgorithm::CreateFunc;
#ifdef USE_BUGGY_SPHERE_BOX_ALGORITHM
	mem = btlAlignedAlloc(sizeof(btlSphereBoxCollisionAlgorithm::CreateFunc),16);
	m_sphereBoxCF = new(mem) btlSphereBoxCollisionAlgorithm::CreateFunc;
	mem = btlAlignedAlloc(sizeof(btlSphereBoxCollisionAlgorithm::CreateFunc),16);
	m_boxSphereCF = new (mem)btlSphereBoxCollisionAlgorithm::CreateFunc;
	m_boxSphereCF->m_swapped = true;
#endif //USE_BUGGY_SPHERE_BOX_ALGORITHM

	mem = btlAlignedAlloc(sizeof(btlSphereTriangleCollisionAlgorithm::CreateFunc),16);
	m_sphereTriangleCF = new (mem)btlSphereTriangleCollisionAlgorithm::CreateFunc;
	mem = btlAlignedAlloc(sizeof(btlSphereTriangleCollisionAlgorithm::CreateFunc),16);
	m_triangleSphereCF = new (mem)btlSphereTriangleCollisionAlgorithm::CreateFunc;
	m_triangleSphereCF->m_swapped = true;
	
	mem = btlAlignedAlloc(sizeof(btlBoxBoxCollisionAlgorithm::CreateFunc),16);
	m_boxBoxCF = new(mem)btlBoxBoxCollisionAlgorithm::CreateFunc;

	//convex versus plane
	mem = btlAlignedAlloc (sizeof(btlConvexPlaneCollisionAlgorithm::CreateFunc),16);
	m_convexPlaneCF = new (mem) btlConvexPlaneCollisionAlgorithm::CreateFunc;
	mem = btlAlignedAlloc (sizeof(btlConvexPlaneCollisionAlgorithm::CreateFunc),16);
	m_planeConvexCF = new (mem) btlConvexPlaneCollisionAlgorithm::CreateFunc;
	m_planeConvexCF->m_swapped = true;
	
	///calculate maximum element size, big enough to fit any collision algorithm in the memory pool
	int maxSize = sizeof(btlConvexConvexAlgorithm);
	int maxSize2 = sizeof(btlConvexConcaveCollisionAlgorithm);
	int maxSize3 = sizeof(btlCompoundCollisionAlgorithm);
	int maxSize4 = sizeof(btlCompoundCompoundCollisionAlgorithm);

	int	collisionAlgorithmMaxElementSize = btlMax(maxSize,constructionInfo.m_customCollisionAlgorithmMaxElementSize);
	collisionAlgorithmMaxElementSize = btlMax(collisionAlgorithmMaxElementSize,maxSize2);
	collisionAlgorithmMaxElementSize = btlMax(collisionAlgorithmMaxElementSize,maxSize3);
	collisionAlgorithmMaxElementSize = btlMax(collisionAlgorithmMaxElementSize,maxSize4);
		
	if (constructionInfo.m_persistentManifoldPool)
	{
		m_ownsPersistentManifoldPool = false;
		m_persistentManifoldPool = constructionInfo.m_persistentManifoldPool;
	} else
	{
		m_ownsPersistentManifoldPool = true;
		void* mem = btlAlignedAlloc(sizeof(btlPoolAllocator),16);
		m_persistentManifoldPool = new (mem) btlPoolAllocator(sizeof(btlPersistentManifold),constructionInfo.m_defaultMaxPersistentManifoldPoolSize);
	}
	
	if (constructionInfo.m_collisionAlgorithmPool)
	{
		m_ownsCollisionAlgorithmPool = false;
		m_collisionAlgorithmPool = constructionInfo.m_collisionAlgorithmPool;
	} else
	{
		m_ownsCollisionAlgorithmPool = true;
		void* mem = btlAlignedAlloc(sizeof(btlPoolAllocator),16);
		m_collisionAlgorithmPool = new(mem) btlPoolAllocator(collisionAlgorithmMaxElementSize,constructionInfo.m_defaultMaxCollisionAlgorithmPoolSize);
	}


}

btlDefaultCollisionConfiguration::~btlDefaultCollisionConfiguration()
{
	if (m_ownsCollisionAlgorithmPool)
	{
		m_collisionAlgorithmPool->~btlPoolAllocator();
		btlAlignedFree(m_collisionAlgorithmPool);
	}
	if (m_ownsPersistentManifoldPool)
	{
		m_persistentManifoldPool->~btlPoolAllocator();
		btlAlignedFree(m_persistentManifoldPool);
	}

	m_convexConvexCreateFunc->~btlCollisionAlgorithmCreateFunc();
	btlAlignedFree(	m_convexConvexCreateFunc);

	m_convexConcaveCreateFunc->~btlCollisionAlgorithmCreateFunc();
	btlAlignedFree( m_convexConcaveCreateFunc);
	m_swappedConvexConcaveCreateFunc->~btlCollisionAlgorithmCreateFunc();
	btlAlignedFree( m_swappedConvexConcaveCreateFunc);

	m_compoundCreateFunc->~btlCollisionAlgorithmCreateFunc();
	btlAlignedFree( m_compoundCreateFunc);

	m_compoundCompoundCreateFunc->~btlCollisionAlgorithmCreateFunc();
	btlAlignedFree(m_compoundCompoundCreateFunc);

	m_swappedCompoundCreateFunc->~btlCollisionAlgorithmCreateFunc();
	btlAlignedFree( m_swappedCompoundCreateFunc);

	m_emptyCreateFunc->~btlCollisionAlgorithmCreateFunc();
	btlAlignedFree( m_emptyCreateFunc);

	m_sphereSphereCF->~btlCollisionAlgorithmCreateFunc();
	btlAlignedFree( m_sphereSphereCF);

#ifdef USE_BUGGY_SPHERE_BOX_ALGORITHM
	m_sphereBoxCF->~btlCollisionAlgorithmCreateFunc();
	btlAlignedFree( m_sphereBoxCF);
	m_boxSphereCF->~btlCollisionAlgorithmCreateFunc();
	btlAlignedFree( m_boxSphereCF);
#endif //USE_BUGGY_SPHERE_BOX_ALGORITHM

	m_sphereTriangleCF->~btlCollisionAlgorithmCreateFunc();
	btlAlignedFree( m_sphereTriangleCF);
	m_triangleSphereCF->~btlCollisionAlgorithmCreateFunc();
	btlAlignedFree( m_triangleSphereCF);
	m_boxBoxCF->~btlCollisionAlgorithmCreateFunc();
	btlAlignedFree( m_boxBoxCF);

	m_convexPlaneCF->~btlCollisionAlgorithmCreateFunc();
	btlAlignedFree( m_convexPlaneCF);
	m_planeConvexCF->~btlCollisionAlgorithmCreateFunc();
	btlAlignedFree( m_planeConvexCF);

	m_simplexSolver->~btlVoronoiSimplexSolver();
	btlAlignedFree(m_simplexSolver);

	m_pdSolver->~btlConvexPenetrationDepthSolver();
	
	btlAlignedFree(m_pdSolver);


}


btlCollisionAlgorithmCreateFunc* btlDefaultCollisionConfiguration::getCollisionAlgorithmCreateFunc(int proxyType0,int proxyType1)
{



	if ((proxyType0 == SPHERE_SHAPE_PROXYTYPE) && (proxyType1==SPHERE_SHAPE_PROXYTYPE))
	{
		return	m_sphereSphereCF;
	}
#ifdef USE_BUGGY_SPHERE_BOX_ALGORITHM
	if ((proxyType0 == SPHERE_SHAPE_PROXYTYPE) && (proxyType1==BOX_SHAPE_PROXYTYPE))
	{
		return	m_sphereBoxCF;
	}

	if ((proxyType0 == BOX_SHAPE_PROXYTYPE ) && (proxyType1==SPHERE_SHAPE_PROXYTYPE))
	{
		return	m_boxSphereCF;
	}
#endif //USE_BUGGY_SPHERE_BOX_ALGORITHM


	if ((proxyType0 == SPHERE_SHAPE_PROXYTYPE ) && (proxyType1==TRIANGLE_SHAPE_PROXYTYPE))
	{
		return	m_sphereTriangleCF;
	}

	if ((proxyType0 == TRIANGLE_SHAPE_PROXYTYPE  ) && (proxyType1==SPHERE_SHAPE_PROXYTYPE))
	{
		return	m_triangleSphereCF;
	} 

	if ((proxyType0 == BOX_SHAPE_PROXYTYPE) && (proxyType1 == BOX_SHAPE_PROXYTYPE))
	{
		return m_boxBoxCF;
	}
	
	if (btlBroadphaseProxy::isConvex(proxyType0) && (proxyType1 == STATIC_PLANE_PROXYTYPE))
	{
		return m_convexPlaneCF;
	}

	if (btlBroadphaseProxy::isConvex(proxyType1) && (proxyType0 == STATIC_PLANE_PROXYTYPE))
	{
		return m_planeConvexCF;
	}
	


	if (btlBroadphaseProxy::isConvex(proxyType0) && btlBroadphaseProxy::isConvex(proxyType1))
	{
		return m_convexConvexCreateFunc;
	}

	if (btlBroadphaseProxy::isConvex(proxyType0) && btlBroadphaseProxy::isConcave(proxyType1))
	{
		return m_convexConcaveCreateFunc;
	}

	if (btlBroadphaseProxy::isConvex(proxyType1) && btlBroadphaseProxy::isConcave(proxyType0))
	{
		return m_swappedConvexConcaveCreateFunc;
	}


	if (btlBroadphaseProxy::isCompound(proxyType0) && btlBroadphaseProxy::isCompound(proxyType1))
	{
		return m_compoundCompoundCreateFunc;
	}

	if (btlBroadphaseProxy::isCompound(proxyType0))
	{
		return m_compoundCreateFunc;
	} else
	{
		if (btlBroadphaseProxy::isCompound(proxyType1))
		{
			return m_swappedCompoundCreateFunc;
		}
	}

	//failed to find an algorithm
	return m_emptyCreateFunc;
}

void btlDefaultCollisionConfiguration::setConvexConvexMultipointIterations(int numPerturbationIterations, int minimumPointsPerturbationThreshold)
{
	btlConvexConvexAlgorithm::CreateFunc* convexConvex = (btlConvexConvexAlgorithm::CreateFunc*) m_convexConvexCreateFunc;
	convexConvex->m_numPerturbationIterations = numPerturbationIterations;
	convexConvex->m_minimumPointsPerturbationThreshold = minimumPointsPerturbationThreshold;
}

void	btlDefaultCollisionConfiguration::setPlaneConvexMultipointIterations(int numPerturbationIterations, int minimumPointsPerturbationThreshold)
{
	btlConvexPlaneCollisionAlgorithm::CreateFunc* cpCF = (btlConvexPlaneCollisionAlgorithm::CreateFunc*)m_convexPlaneCF;
	cpCF->m_numPerturbationIterations = numPerturbationIterations;
	cpCF->m_minimumPointsPerturbationThreshold = minimumPointsPerturbationThreshold;
	
	btlConvexPlaneCollisionAlgorithm::CreateFunc* pcCF = (btlConvexPlaneCollisionAlgorithm::CreateFunc*)m_planeConvexCF;
	pcCF->m_numPerturbationIterations = numPerturbationIterations;
	pcCF->m_minimumPointsPerturbationThreshold = minimumPointsPerturbationThreshold;
}
