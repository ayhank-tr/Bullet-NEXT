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

#ifndef BT_DEFAULT_COLLISION_CONFIGURATION
#define BT_DEFAULT_COLLISION_CONFIGURATION

#include "btCollisionConfiguration.h"
class btlVoronoiSimplexSolver;
class btlConvexPenetrationDepthSolver;

struct	btlDefaultCollisionConstructionInfo
{
	btlPoolAllocator*	m_persistentManifoldPool;
	btlPoolAllocator*	m_collisionAlgorithmPool;
	int					m_defaultMaxPersistentManifoldPoolSize;
	int					m_defaultMaxCollisionAlgorithmPoolSize;
	int					m_customCollisionAlgorithmMaxElementSize;
	int					m_useEpaPenetrationAlgorithm;

	btlDefaultCollisionConstructionInfo()
		:m_persistentManifoldPool(0),
		m_collisionAlgorithmPool(0),
		m_defaultMaxPersistentManifoldPoolSize(4096),
		m_defaultMaxCollisionAlgorithmPoolSize(4096),
		m_customCollisionAlgorithmMaxElementSize(0),
		m_useEpaPenetrationAlgorithm(true)
	{
	}
};



///btCollisionConfiguration allows to configure Bullet collision detection
///stack allocator, pool memory allocators
///@todo: describe the meaning
class	btlDefaultCollisionConfiguration : public btlCollisionConfiguration
{

protected:

	int	m_persistentManifoldPoolSize;
	

	btlPoolAllocator*	m_persistentManifoldPool;
	bool	m_ownsPersistentManifoldPool;


	btlPoolAllocator*	m_collisionAlgorithmPool;
	bool	m_ownsCollisionAlgorithmPool;

	//default simplex/penetration depth solvers
	btlVoronoiSimplexSolver*	m_simplexSolver;
	btlConvexPenetrationDepthSolver*	m_pdSolver;
	
	//default CreationFunctions, filling the m_doubleDispatch table
	btlCollisionAlgorithmCreateFunc*	m_convexConvexCreateFunc;
	btlCollisionAlgorithmCreateFunc*	m_convexConcaveCreateFunc;
	btlCollisionAlgorithmCreateFunc*	m_swappedConvexConcaveCreateFunc;
	btlCollisionAlgorithmCreateFunc*	m_compoundCreateFunc;
	btlCollisionAlgorithmCreateFunc*	m_compoundCompoundCreateFunc;
	
	btlCollisionAlgorithmCreateFunc*	m_swappedCompoundCreateFunc;
	btlCollisionAlgorithmCreateFunc* m_emptyCreateFunc;
	btlCollisionAlgorithmCreateFunc* m_sphereSphereCF;
	btlCollisionAlgorithmCreateFunc* m_sphereBoxCF;
	btlCollisionAlgorithmCreateFunc* m_boxSphereCF;

	btlCollisionAlgorithmCreateFunc* m_boxBoxCF;
	btlCollisionAlgorithmCreateFunc*	m_sphereTriangleCF;
	btlCollisionAlgorithmCreateFunc*	m_triangleSphereCF;
	btlCollisionAlgorithmCreateFunc*	m_planeConvexCF;
	btlCollisionAlgorithmCreateFunc*	m_convexPlaneCF;
	
public:


	btlDefaultCollisionConfiguration(const btlDefaultCollisionConstructionInfo& constructionInfo = btlDefaultCollisionConstructionInfo());

	virtual ~btlDefaultCollisionConfiguration();

		///memory pools
	virtual btlPoolAllocator* getPersistentManifoldPool()
	{
		return m_persistentManifoldPool;
	}

	virtual btlPoolAllocator* getCollisionAlgorithmPool()
	{
		return m_collisionAlgorithmPool;
	}


	virtual	btlVoronoiSimplexSolver*	getSimplexSolver()
	{
		return m_simplexSolver;
	}


	virtual btlCollisionAlgorithmCreateFunc* getCollisionAlgorithmCreateFunc(int proxyType0,int proxyType1);

	///Use this method to allow to generate multiple contact points between at once, between two objects using the generic convex-convex algorithm.
	///By default, this feature is disabled for best performance.
	///@param numPerturbationIterations controls the number of collision queries. Set it to zero to disable the feature.
	///@param minimumPointsPerturbationThreshold is the minimum number of points in the contact cache, above which the feature is disabled
	///3 is a good value for both params, if you want to enable the feature. This is because the default contact cache contains a maximum of 4 points, and one collision query at the unperturbed orientation is performed first.
	///See Bullet/Demos/CollisionDemo for an example how this feature gathers multiple points.
	///@todo we could add a per-object setting of those parameters, for level-of-detail collision detection.
	void	setConvexConvexMultipointIterations(int numPerturbationIterations=3, int minimumPointsPerturbationThreshold = 3);

	void	setPlaneConvexMultipointIterations(int numPerturbationIterations=3, int minimumPointsPerturbationThreshold = 3);

};

#endif //BT_DEFAULT_COLLISION_CONFIGURATION

