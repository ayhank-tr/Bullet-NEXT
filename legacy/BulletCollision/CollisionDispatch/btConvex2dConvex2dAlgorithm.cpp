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

#include "btConvex2dConvex2dAlgorithm.h"

//#include <stdio.h>
#include "BulletCollision/NarrowPhaseCollision/btDiscreteCollisionDetectorInterface.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseInterface.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionShapes/btConvexShape.h"
#include "BulletCollision/CollisionShapes/btCapsuleShape.h"


#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionDispatch/btManifoldResult.h"

#include "BulletCollision/NarrowPhaseCollision/btConvexPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.h"
#include "BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkConvexCast.h"



#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"

#include "BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.h"

#include "BulletCollision/NarrowPhaseCollision/btGjkEpa2.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h"
#include "BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h"

btlConvex2dConvex2dAlgorithm::CreateFunc::CreateFunc(btlSimplexSolverInterface*			simplexSolver, btlConvexPenetrationDepthSolver* pdSolver)
{
	m_simplexSolver = simplexSolver;
	m_pdSolver = pdSolver;
}

btlConvex2dConvex2dAlgorithm::CreateFunc::~CreateFunc() 
{ 
}

btlConvex2dConvex2dAlgorithm::btlConvex2dConvex2dAlgorithm(btlPersistentManifold* mf,const btlCollisionAlgorithmConstructionInfo& ci,const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap,btlSimplexSolverInterface* simplexSolver, btlConvexPenetrationDepthSolver* pdSolver,int /* numPerturbationIterations */, int /* minimumPointsPerturbationThreshold */)
: btlActivatingCollisionAlgorithm(ci,body0Wrap,body1Wrap),
m_simplexSolver(simplexSolver),
m_pdSolver(pdSolver),
m_ownManifold (false),
m_manifoldPtr(mf),
m_lowLevelOfDetail(false)
{
	(void)body0Wrap;
	(void)body1Wrap;
}




btlConvex2dConvex2dAlgorithm::~btlConvex2dConvex2dAlgorithm()
{
	if (m_ownManifold)
	{
		if (m_manifoldPtr)
			m_dispatcher->releaseManifold(m_manifoldPtr);
	}
}

void	btlConvex2dConvex2dAlgorithm ::setLowLevelOfDetail(bool useLowLevel)
{
	m_lowLevelOfDetail = useLowLevel;
}



extern btlScalar btl_gContactBreakingThreshold;


//
// Convex-Convex collision algorithm
//
void btlConvex2dConvex2dAlgorithm ::processCollision (const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap,const btlDispatcherInfo& dispatchInfo,btlManifoldResult* resultOut)
{

	if (!m_manifoldPtr)
	{
		//swapped?
		m_manifoldPtr = m_dispatcher->getNewManifold(body0Wrap->getCollisionObject(),body1Wrap->getCollisionObject());
		m_ownManifold = true;
	}
	resultOut->setPersistentManifold(m_manifoldPtr);

	//comment-out next line to test multi-contact generation
	//resultOut->getPersistentManifold()->clearManifold();


	const btlConvexShape* min0 = static_cast<const btlConvexShape*>(body0Wrap->getCollisionShape());
	const btlConvexShape* min1 = static_cast<const btlConvexShape*>(body1Wrap->getCollisionShape());

	btlVector3  normalOnB;
	btlVector3  pointOnBWorld;

	{


		btlGjkPairDetector::ClosestPointInput input;

		btlGjkPairDetector	gjkPairDetector(min0,min1,m_simplexSolver,m_pdSolver);
		//TODO: if (dispatchInfo.m_useContinuous)
		gjkPairDetector.setMinkowskiA(min0);
		gjkPairDetector.setMinkowskiB(min1);

		{
			input.m_maximumDistanceSquared = min0->getMargin() + min1->getMargin() + m_manifoldPtr->getContactBreakingThreshold();
			input.m_maximumDistanceSquared*= input.m_maximumDistanceSquared;
		}

		input.m_transformA = body0Wrap->getWorldTransform();
		input.m_transformB = body1Wrap->getWorldTransform();

		gjkPairDetector.getClosestPoints(input,*resultOut,dispatchInfo.m_debugDraw);

		btlVector3 v0,v1;
		btlVector3 sepNormalWorldSpace;

	}

	if (m_ownManifold)
	{
		resultOut->refreshContactPoints();
	}

}




btlScalar	btlConvex2dConvex2dAlgorithm::calculateTimeOfImpact(btlCollisionObject* col0,btlCollisionObject* col1,const btlDispatcherInfo& dispatchInfo,btlManifoldResult* resultOut)
{
	(void)resultOut;
	(void)dispatchInfo;
	///Rather then checking ALL pairs, only calculate TOI when motion exceeds threshold

	///Linear motion for one of objects needs to exceed m_ccdSquareMotionThreshold
	///col0->m_worldTransform,
	btlScalar resultFraction = btlScalar(1.);


	btlScalar squareMot0 = (col0->getInterpolationWorldTransform().getOrigin() - col0->getWorldTransform().getOrigin()).length2();
	btlScalar squareMot1 = (col1->getInterpolationWorldTransform().getOrigin() - col1->getWorldTransform().getOrigin()).length2();

	if (squareMot0 < col0->getCcdSquareMotionThreshold() &&
		squareMot1 < col1->getCcdSquareMotionThreshold())
		return resultFraction;


	//An adhoc way of testing the Continuous Collision Detection algorithms
	//One object is approximated as a sphere, to simplify things
	//Starting in penetration should report no time of impact
	//For proper CCD, better accuracy and handling of 'allowed' penetration should be added
	//also the mainloop of the physics should have a kind of toi queue (something like Brian Mirtich's application of Timewarp for Rigidbodies)


	/// Convex0 against sphere for Convex1
	{
		btlConvexShape* convex0 = static_cast<btlConvexShape*>(col0->getCollisionShape());

		btlSphereShape	sphere1(col1->getCcdSweptSphereRadius()); //todo: allow non-zero sphere sizes, for better approximation
		btlConvexCast::CastResult result;
		btlVoronoiSimplexSolver voronoiSimplex;
		//SubsimplexConvexCast ccd0(&sphere,min0,&voronoiSimplex);
		///Simplification, one object is simplified as a sphere
		btlGjkConvexCast ccd1( convex0 ,&sphere1,&voronoiSimplex);
		//ContinuousConvexCollision ccd(min0,min1,&voronoiSimplex,0);
		if (ccd1.calcTimeOfImpact(col0->getWorldTransform(),col0->getInterpolationWorldTransform(),
			col1->getWorldTransform(),col1->getInterpolationWorldTransform(),result))
		{

			//store result.m_fraction in both bodies

			if (col0->getHitFraction()> result.m_fraction)
				col0->setHitFraction( result.m_fraction );

			if (col1->getHitFraction() > result.m_fraction)
				col1->setHitFraction( result.m_fraction);

			if (resultFraction > result.m_fraction)
				resultFraction = result.m_fraction;

		}




	}

	/// Sphere (for convex0) against Convex1
	{
		btlConvexShape* convex1 = static_cast<btlConvexShape*>(col1->getCollisionShape());

		btlSphereShape	sphere0(col0->getCcdSweptSphereRadius()); //todo: allow non-zero sphere sizes, for better approximation
		btlConvexCast::CastResult result;
		btlVoronoiSimplexSolver voronoiSimplex;
		//SubsimplexConvexCast ccd0(&sphere,min0,&voronoiSimplex);
		///Simplification, one object is simplified as a sphere
		btlGjkConvexCast ccd1(&sphere0,convex1,&voronoiSimplex);
		//ContinuousConvexCollision ccd(min0,min1,&voronoiSimplex,0);
		if (ccd1.calcTimeOfImpact(col0->getWorldTransform(),col0->getInterpolationWorldTransform(),
			col1->getWorldTransform(),col1->getInterpolationWorldTransform(),result))
		{

			//store result.m_fraction in both bodies

			if (col0->getHitFraction()	> result.m_fraction)
				col0->setHitFraction( result.m_fraction);

			if (col1->getHitFraction() > result.m_fraction)
				col1->setHitFraction( result.m_fraction);

			if (resultFraction > result.m_fraction)
				resultFraction = result.m_fraction;

		}
	}

	return resultFraction;

}

