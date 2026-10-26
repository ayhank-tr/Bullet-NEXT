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

#ifndef BT_CONVEX_2D_CONVEX_2D_ALGORITHM_H
#define BT_CONVEX_2D_CONVEX_2D_ALGORITHM_H

#include "BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/CollisionDispatch/btCollisionCreateFunc.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "LinearMath/btTransformUtil.h" //for btConvexSeparatingDistanceUtil

class btlConvexPenetrationDepthSolver;


///The convex2dConvex2dAlgorithm collision algorithm support 2d collision detection for btConvex2dShape
///Currently it requires the btMinkowskiPenetrationDepthSolver, it has support for 2d penetration depth computation
class btlConvex2dConvex2dAlgorithm : public btlActivatingCollisionAlgorithm
{
	btlSimplexSolverInterface*		m_simplexSolver;
	btlConvexPenetrationDepthSolver* m_pdSolver;

	
	bool	m_ownManifold;
	btlPersistentManifold*	m_manifoldPtr;
	bool			m_lowLevelOfDetail;
	
public:

	btlConvex2dConvex2dAlgorithm(btlPersistentManifold* mf,const btlCollisionAlgorithmConstructionInfo& ci,const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap, btlSimplexSolverInterface* simplexSolver, btlConvexPenetrationDepthSolver* pdSolver, int numPerturbationIterations, int minimumPointsPerturbationThreshold);


	virtual ~btlConvex2dConvex2dAlgorithm();

	virtual void processCollision (const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap,const btlDispatcherInfo& dispatchInfo,btlManifoldResult* resultOut);

	virtual btlScalar calculateTimeOfImpact(btlCollisionObject* body0,btlCollisionObject* body1,const btlDispatcherInfo& dispatchInfo,btlManifoldResult* resultOut);

	virtual	void	getAllContactManifolds(btlManifoldArray&	manifoldArray)
	{
		///should we use m_ownManifold to avoid adding duplicates?
		if (m_manifoldPtr && m_ownManifold)
			manifoldArray.push_back(m_manifoldPtr);
	}


	void	setLowLevelOfDetail(bool useLowLevel);


	const btlPersistentManifold*	getManifold()
	{
		return m_manifoldPtr;
	}

	struct CreateFunc :public 	btlCollisionAlgorithmCreateFunc
	{

		btlConvexPenetrationDepthSolver*		m_pdSolver;
		btlSimplexSolverInterface*			m_simplexSolver;
		int m_numPerturbationIterations;
		int m_minimumPointsPerturbationThreshold;

		CreateFunc(btlSimplexSolverInterface*			simplexSolver, btlConvexPenetrationDepthSolver* pdSolver);
		
		virtual ~CreateFunc();

		virtual	btlCollisionAlgorithm* CreateCollisionAlgorithm(btlCollisionAlgorithmConstructionInfo& ci, const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap)
		{
			void* mem = ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(btlConvex2dConvex2dAlgorithm));
			return new(mem) btlConvex2dConvex2dAlgorithm(ci.m_manifold,ci,body0Wrap,body1Wrap,m_simplexSolver,m_pdSolver,m_numPerturbationIterations,m_minimumPointsPerturbationThreshold);
		}
	};


};

#endif //BT_CONVEX_2D_CONVEX_2D_ALGORITHM_H
