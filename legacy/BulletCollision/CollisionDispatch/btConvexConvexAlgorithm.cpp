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

///Specialized capsule-capsule collision algorithm has been added for Bullet 2.75 release to increase ragdoll performance
///If you experience problems with capsule-capsule collision, try to define BT_DISABLE_CAPSULE_CAPSULE_COLLIDER and report it in the Bullet forums
///with reproduction case
//define BT_DISABLE_CAPSULE_CAPSULE_COLLIDER 1
//#define ZERO_MARGIN

#include "btConvexConvexAlgorithm.h"

//#include <stdio.h>
#include "BulletCollision/NarrowPhaseCollision/btDiscreteCollisionDetectorInterface.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseInterface.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionShapes/btConvexShape.h"
#include "BulletCollision/CollisionShapes/btCapsuleShape.h"
#include "BulletCollision/CollisionShapes/btTriangleShape.h"



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
#include "BulletCollision/NarrowPhaseCollision/btPolyhedralContactClipping.h"
#include "BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h"

///////////



static SIMD_FORCE_INLINE void segmentsClosestPoints(
	btlVector3& ptsVector,
	btlVector3& offsetA,
	btlVector3& offsetB,
	btlScalar& tA, btlScalar& tB,
	const btlVector3& translation,
	const btlVector3& dirA, btlScalar hlenA,
	const btlVector3& dirB, btlScalar hlenB )
{
	// compute the parameters of the closest points on each line segment

	btlScalar dirA_dot_dirB = btlDot(dirA,dirB);
	btlScalar dirA_dot_trans = btlDot(dirA,translation);
	btlScalar dirB_dot_trans = btlDot(dirB,translation);

	btlScalar denom = 1.0f - dirA_dot_dirB * dirA_dot_dirB;

	if ( denom == 0.0f ) {
		tA = 0.0f;
	} else {
		tA = ( dirA_dot_trans - dirB_dot_trans * dirA_dot_dirB ) / denom;
		if ( tA < -hlenA )
			tA = -hlenA;
		else if ( tA > hlenA )
			tA = hlenA;
	}

	tB = tA * dirA_dot_dirB - dirB_dot_trans;

	if ( tB < -hlenB ) {
		tB = -hlenB;
		tA = tB * dirA_dot_dirB + dirA_dot_trans;

		if ( tA < -hlenA )
			tA = -hlenA;
		else if ( tA > hlenA )
			tA = hlenA;
	} else if ( tB > hlenB ) {
		tB = hlenB;
		tA = tB * dirA_dot_dirB + dirA_dot_trans;

		if ( tA < -hlenA )
			tA = -hlenA;
		else if ( tA > hlenA )
			tA = hlenA;
	}

	// compute the closest points relative to segment centers.

	offsetA = dirA * tA;
	offsetB = dirB * tB;

	ptsVector = translation - offsetA + offsetB;
}


static SIMD_FORCE_INLINE btlScalar capsuleCapsuleDistance(
	btlVector3& normalOnB,
	btlVector3& pointOnB,
	btlScalar capsuleLengthA,
	btlScalar	capsuleRadiusA,
	btlScalar capsuleLengthB,
	btlScalar	capsuleRadiusB,
	int capsuleAxisA,
	int capsuleAxisB,
	const btlTransform& transformA,
	const btlTransform& transformB,
	btlScalar distanceThreshold )
{
	btlVector3 directionA = transformA.getBasis().getColumn(capsuleAxisA);
	btlVector3 translationA = transformA.getOrigin();
	btlVector3 directionB = transformB.getBasis().getColumn(capsuleAxisB);
	btlVector3 translationB = transformB.getOrigin();

	// translation between centers

	btlVector3 translation = translationB - translationA;

	// compute the closest points of the capsule line segments

	btlVector3 ptsVector;           // the vector between the closest points
	
	btlVector3 offsetA, offsetB;    // offsets from segment centers to their closest points
	btlScalar tA, tB;              // parameters on line segment

	segmentsClosestPoints( ptsVector, offsetA, offsetB, tA, tB, translation,
						   directionA, capsuleLengthA, directionB, capsuleLengthB );

	btlScalar distance = ptsVector.length() - capsuleRadiusA - capsuleRadiusB;

	if ( distance > distanceThreshold )
		return distance;

	btlScalar lenSqr = ptsVector.length2();
	if (lenSqr<= (SIMD_EPSILON*SIMD_EPSILON))
	{
		//degenerate case where 2 capsules are likely at the same location: take a vector tangential to 'directionA'
		btlVector3 q;
		btlPlaneSpace1(directionA,normalOnB,q);
	} else
	{
		// compute the contact normal
		normalOnB = ptsVector*-btlRecipSqrt(lenSqr);
	}
	pointOnB = transformB.getOrigin()+offsetB + normalOnB * capsuleRadiusB;

	return distance;
}







//////////





btlConvexConvexAlgorithm::CreateFunc::CreateFunc(btlSimplexSolverInterface*			simplexSolver, btlConvexPenetrationDepthSolver* pdSolver)
{
	m_numPerturbationIterations = 0;
	m_minimumPointsPerturbationThreshold = 3;
	m_simplexSolver = simplexSolver;
	m_pdSolver = pdSolver;
}

btlConvexConvexAlgorithm::CreateFunc::~CreateFunc() 
{ 
}

btlConvexConvexAlgorithm::btlConvexConvexAlgorithm(btlPersistentManifold* mf,const btlCollisionAlgorithmConstructionInfo& ci,const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap,btlSimplexSolverInterface* simplexSolver, btlConvexPenetrationDepthSolver* pdSolver,int numPerturbationIterations, int minimumPointsPerturbationThreshold)
: btlActivatingCollisionAlgorithm(ci,body0Wrap,body1Wrap),
m_simplexSolver(simplexSolver),
m_pdSolver(pdSolver),
m_ownManifold (false),
m_manifoldPtr(mf),
m_lowLevelOfDetail(false),
#ifdef USE_SEPDISTANCE_UTIL2
m_sepDistance((static_cast<btlConvexShape*>(body0->getCollisionShape()))->getAngularMotionDisc(),
			  (static_cast<btlConvexShape*>(body1->getCollisionShape()))->getAngularMotionDisc()),
#endif
m_numPerturbationIterations(numPerturbationIterations),
m_minimumPointsPerturbationThreshold(minimumPointsPerturbationThreshold)
{
	(void)body0Wrap;
	(void)body1Wrap;
}




btlConvexConvexAlgorithm::~btlConvexConvexAlgorithm()
{
	if (m_ownManifold)
	{
		if (m_manifoldPtr)
			m_dispatcher->releaseManifold(m_manifoldPtr);
	}
}

void	btlConvexConvexAlgorithm ::setLowLevelOfDetail(bool useLowLevel)
{
	m_lowLevelOfDetail = useLowLevel;
}


struct btlPerturbedContactResult : public btlManifoldResult
{
	btlManifoldResult* m_originalManifoldResult;
	btlTransform m_transformA;
	btlTransform m_transformB;
	btlTransform	m_unPerturbedTransform;
	bool	m_perturbA;
	btlIDebugDraw*	m_debugDrawer;


	btlPerturbedContactResult(btlManifoldResult* originalResult,const btlTransform& transformA,const btlTransform& transformB,const btlTransform& unPerturbedTransform,bool perturbA,btlIDebugDraw* debugDrawer)
		:m_originalManifoldResult(originalResult),
		m_transformA(transformA),
		m_transformB(transformB),
		m_unPerturbedTransform(unPerturbedTransform),
		m_perturbA(perturbA),
		m_debugDrawer(debugDrawer)
	{
	}
	virtual ~ btlPerturbedContactResult()
	{
	}

	virtual void addContactPoint(const btlVector3& normalOnBInWorld,const btlVector3& pointInWorld,btlScalar orgDepth)
	{
		btlVector3 endPt,startPt;
		btlScalar newDepth;
		btlVector3 newNormal;

		if (m_perturbA)
		{
			btlVector3 endPtOrg = pointInWorld + normalOnBInWorld*orgDepth;
			endPt = (m_unPerturbedTransform*m_transformA.inverse())(endPtOrg);
			newDepth = (endPt -  pointInWorld).dot(normalOnBInWorld);
			startPt = endPt+normalOnBInWorld*newDepth;
		} else
		{
			endPt = pointInWorld + normalOnBInWorld*orgDepth;
			startPt = (m_unPerturbedTransform*m_transformB.inverse())(pointInWorld);
			newDepth = (endPt -  startPt).dot(normalOnBInWorld);
			
		}

//#define DEBUG_CONTACTS 1
#ifdef DEBUG_CONTACTS
		m_debugDrawer->drawLine(startPt,endPt,btlVector3(1,0,0));
		m_debugDrawer->drawSphere(startPt,0.05,btlVector3(0,1,0));
		m_debugDrawer->drawSphere(endPt,0.05,btlVector3(0,0,1));
#endif //DEBUG_CONTACTS

		
		m_originalManifoldResult->addContactPoint(normalOnBInWorld,startPt,newDepth);
	}

};

extern btlScalar btl_gContactBreakingThreshold;


//
// Convex-Convex collision algorithm
//
void btlConvexConvexAlgorithm ::processCollision (const btlCollisionObjectWrapper* body0Wrap,const btlCollisionObjectWrapper* body1Wrap,const btlDispatcherInfo& dispatchInfo,btlManifoldResult* resultOut)
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
#ifndef BT_DISABLE_CAPSULE_CAPSULE_COLLIDER
	if ((min0->getShapeType() == CAPSULE_SHAPE_PROXYTYPE) && (min1->getShapeType() == CAPSULE_SHAPE_PROXYTYPE))
	{
		btlCapsuleShape* capsuleA = (btlCapsuleShape*) min0;
		btlCapsuleShape* capsuleB = (btlCapsuleShape*) min1;
	//	btVector3 localScalingA = capsuleA->getLocalScaling();
	//	btVector3 localScalingB = capsuleB->getLocalScaling();
		
		btlScalar threshold = m_manifoldPtr->getContactBreakingThreshold();

		btlScalar dist = capsuleCapsuleDistance(normalOnB,	pointOnBWorld,capsuleA->getHalfHeight(),capsuleA->getRadius(),
			capsuleB->getHalfHeight(),capsuleB->getRadius(),capsuleA->getUpAxis(),capsuleB->getUpAxis(),
			body0Wrap->getWorldTransform(),body1Wrap->getWorldTransform(),threshold);

		if (dist<threshold)
		{
			btlAssert(normalOnB.length2()>=(SIMD_EPSILON*SIMD_EPSILON));
			resultOut->addContactPoint(normalOnB,pointOnBWorld,dist);	
		}
		resultOut->refreshContactPoints();
		return;
	}
#endif //BT_DISABLE_CAPSULE_CAPSULE_COLLIDER




#ifdef USE_SEPDISTANCE_UTIL2
	if (dispatchInfo.m_useConvexConservativeDistanceUtil)
	{
		m_sepDistance.updateSeparatingDistance(body0->getWorldTransform(),body1->getWorldTransform());
	}

	if (!dispatchInfo.m_useConvexConservativeDistanceUtil || m_sepDistance.getConservativeSeparatingDistance()<=0.f)
#endif //USE_SEPDISTANCE_UTIL2

	{

	
	btlGjkPairDetector::ClosestPointInput input;

	btlGjkPairDetector	gjkPairDetector(min0,min1,m_simplexSolver,m_pdSolver);
	//TODO: if (dispatchInfo.m_useContinuous)
	gjkPairDetector.setMinkowskiA(min0);
	gjkPairDetector.setMinkowskiB(min1);

#ifdef USE_SEPDISTANCE_UTIL2
	if (dispatchInfo.m_useConvexConservativeDistanceUtil)
	{
		input.m_maximumDistanceSquared = BT_LARGE_FLOAT;
	} else
#endif //USE_SEPDISTANCE_UTIL2
	{
		//if (dispatchInfo.m_convexMaxDistanceUseCPT)
		//{
		//	input.m_maximumDistanceSquared = min0->getMargin() + min1->getMargin() + m_manifoldPtr->getContactProcessingThreshold();
		//} else
		//{
		input.m_maximumDistanceSquared = min0->getMargin() + min1->getMargin() + m_manifoldPtr->getContactBreakingThreshold();
//		}

		input.m_maximumDistanceSquared*= input.m_maximumDistanceSquared;
	}

	input.m_transformA = body0Wrap->getWorldTransform();
	input.m_transformB = body1Wrap->getWorldTransform();



	

#ifdef USE_SEPDISTANCE_UTIL2
	btlScalar sepDist = 0.f;
	if (dispatchInfo.m_useConvexConservativeDistanceUtil)
	{
		sepDist = gjkPairDetector.getCachedSeparatingDistance();
		if (sepDist>SIMD_EPSILON)
		{
			sepDist += dispatchInfo.m_convexConservativeDistanceThreshold;
			//now perturbe directions to get multiple contact points
			
		}
	}
#endif //USE_SEPDISTANCE_UTIL2

	if (min0->isPolyhedral() && min1->isPolyhedral())
	{


		struct btlDummyResult : public btlDiscreteCollisionDetectorInterface::Result
		{
			virtual void setShapeIdentifiersA(int partId0,int index0){}
			virtual void setShapeIdentifiersB(int partId1,int index1){}
			virtual void addContactPoint(const btlVector3& normalOnBInWorld,const btlVector3& pointInWorld,btlScalar depth) 
			{
			}
		};

		
		struct btlWithoutMarginResult : public btlDiscreteCollisionDetectorInterface::Result
		{
			btlDiscreteCollisionDetectorInterface::Result* m_originalResult;
			btlVector3	m_reportedNormalOnWorld;
			btlScalar m_marginOnA;
			btlScalar m_marginOnB;
			btlScalar	m_reportedDistance;
			
			bool		m_foundResult;
			btlWithoutMarginResult(btlDiscreteCollisionDetectorInterface::Result* result, btlScalar marginOnA, btlScalar marginOnB)
			:m_originalResult(result),
			m_marginOnA(marginOnA),
			m_marginOnB(marginOnB),
			m_foundResult(false)
			{
			}
			
			virtual void setShapeIdentifiersA(int partId0,int index0){}
			virtual void setShapeIdentifiersB(int partId1,int index1){}
			virtual void addContactPoint(const btlVector3& normalOnBInWorld,const btlVector3& pointInWorldOrg,btlScalar depthOrg) 
			{
				m_reportedDistance = depthOrg;
				m_reportedNormalOnWorld = normalOnBInWorld;
				
				btlVector3 adjustedPointB = pointInWorldOrg - normalOnBInWorld*m_marginOnB;
				m_reportedDistance = depthOrg+(m_marginOnA+m_marginOnB);
				if (m_reportedDistance<0.f)
				{
					m_foundResult = true;					
				}
				m_originalResult->addContactPoint(normalOnBInWorld,adjustedPointB,m_reportedDistance);
			}
		};

		
		btlDummyResult dummy;

///btBoxShape is an exception: its vertices are created WITH margin so don't subtract it

		btlScalar min0Margin = min0->getShapeType()==BOX_SHAPE_PROXYTYPE? 0.f : min0->getMargin();
		btlScalar min1Margin = min1->getShapeType()==BOX_SHAPE_PROXYTYPE? 0.f : min1->getMargin();

		btlWithoutMarginResult	withoutMargin(resultOut, min0Margin,min1Margin);

		btlPolyhedralConvexShape* polyhedronA = (btlPolyhedralConvexShape*) min0;
		btlPolyhedralConvexShape* polyhedronB = (btlPolyhedralConvexShape*) min1;
		if (polyhedronA->getConvexPolyhedron() && polyhedronB->getConvexPolyhedron())
		{


			

			btlScalar threshold = m_manifoldPtr->getContactBreakingThreshold();

			btlScalar minDist = -1e30f;
			btlVector3 sepNormalWorldSpace;
			bool foundSepAxis  = true;

			if (dispatchInfo.m_enableSatConvex)
			{
				foundSepAxis = btlPolyhedralContactClipping::findSeparatingAxis(
					*polyhedronA->getConvexPolyhedron(), *polyhedronB->getConvexPolyhedron(),
					body0Wrap->getWorldTransform(), 
					body1Wrap->getWorldTransform(),
					sepNormalWorldSpace,*resultOut);
			} else
			{
#ifdef ZERO_MARGIN
				gjkPairDetector.setIgnoreMargin(true);
				gjkPairDetector.getClosestPoints(input,*resultOut,dispatchInfo.m_debugDraw);
#else


				gjkPairDetector.getClosestPoints(input,withoutMargin,dispatchInfo.m_debugDraw);
				//gjkPairDetector.getClosestPoints(input,dummy,dispatchInfo.m_debugDraw);
#endif //ZERO_MARGIN
				//btScalar l2 = gjkPairDetector.getCachedSeparatingAxis().length2();
				//if (l2>SIMD_EPSILON)
				{
					sepNormalWorldSpace = withoutMargin.m_reportedNormalOnWorld;//gjkPairDetector.getCachedSeparatingAxis()*(1.f/l2);
					//minDist = -1e30f;//gjkPairDetector.getCachedSeparatingDistance();
					minDist = withoutMargin.m_reportedDistance;//gjkPairDetector.getCachedSeparatingDistance()+min0->getMargin()+min1->getMargin();
	
#ifdef ZERO_MARGIN
					foundSepAxis = true;//gjkPairDetector.getCachedSeparatingDistance()<0.f;
#else
					foundSepAxis = withoutMargin.m_foundResult && minDist<0;//-(min0->getMargin()+min1->getMargin());
#endif
				}
			}
			if (foundSepAxis)
			{
				
//				printf("sepNormalWorldSpace=%f,%f,%f\n",sepNormalWorldSpace.getX(),sepNormalWorldSpace.getY(),sepNormalWorldSpace.getZ());

				btlPolyhedralContactClipping::clipHullAgainstHull(sepNormalWorldSpace, *polyhedronA->getConvexPolyhedron(), *polyhedronB->getConvexPolyhedron(),
					body0Wrap->getWorldTransform(), 
					body1Wrap->getWorldTransform(), minDist-threshold, threshold, *resultOut);
 				
			}
			if (m_ownManifold)
			{
				resultOut->refreshContactPoints();
			}
			return;

		} else
		{
			//we can also deal with convex versus triangle (without connectivity data)
			if (polyhedronA->getConvexPolyhedron() && polyhedronB->getShapeType()==TRIANGLE_SHAPE_PROXYTYPE)
			{

				btlVertexArray vertices;
				btlTriangleShape* tri = (btlTriangleShape*)polyhedronB;
				vertices.push_back(	body1Wrap->getWorldTransform()*tri->m_vertices1[0]);
				vertices.push_back(	body1Wrap->getWorldTransform()*tri->m_vertices1[1]);
				vertices.push_back(	body1Wrap->getWorldTransform()*tri->m_vertices1[2]);
				
				//tri->initializePolyhedralFeatures();

				btlScalar threshold = m_manifoldPtr->getContactBreakingThreshold();

				btlVector3 sepNormalWorldSpace;
				btlScalar minDist =-1e30f;
				btlScalar maxDist = threshold;
				
				bool foundSepAxis = false;
				if (0)
				{
					polyhedronB->initializePolyhedralFeatures();
					 foundSepAxis = btlPolyhedralContactClipping::findSeparatingAxis(
					*polyhedronA->getConvexPolyhedron(), *polyhedronB->getConvexPolyhedron(),
					body0Wrap->getWorldTransform(), 
					body1Wrap->getWorldTransform(),
					sepNormalWorldSpace,*resultOut);
				//	 printf("sepNormalWorldSpace=%f,%f,%f\n",sepNormalWorldSpace.getX(),sepNormalWorldSpace.getY(),sepNormalWorldSpace.getZ());

				} else
				{
#ifdef ZERO_MARGIN
					gjkPairDetector.setIgnoreMargin(true);
					gjkPairDetector.getClosestPoints(input,*resultOut,dispatchInfo.m_debugDraw);
#else
					gjkPairDetector.getClosestPoints(input,dummy,dispatchInfo.m_debugDraw);
#endif//ZERO_MARGIN
					
					btlScalar l2 = gjkPairDetector.getCachedSeparatingAxis().length2();
					if (l2>SIMD_EPSILON)
					{
						sepNormalWorldSpace = gjkPairDetector.getCachedSeparatingAxis()*(1.f/l2);
						//minDist = gjkPairDetector.getCachedSeparatingDistance();
						//maxDist = threshold;
						minDist = gjkPairDetector.getCachedSeparatingDistance()-min0->getMargin()-min1->getMargin();
						foundSepAxis = true;
					}
				}

				
			if (foundSepAxis)
			{
				btlPolyhedralContactClipping::clipFaceAgainstHull(sepNormalWorldSpace, *polyhedronA->getConvexPolyhedron(), 
					body0Wrap->getWorldTransform(), vertices, minDist-threshold, maxDist, *resultOut);
			}
				
				
				if (m_ownManifold)
				{
					resultOut->refreshContactPoints();
				}
				
				return;
			}
			
		}


	}
	
	gjkPairDetector.getClosestPoints(input,*resultOut,dispatchInfo.m_debugDraw);

	//now perform 'm_numPerturbationIterations' collision queries with the perturbated collision objects
	
	//perform perturbation when more then 'm_minimumPointsPerturbationThreshold' points
	if (m_numPerturbationIterations && resultOut->getPersistentManifold()->getNumContacts() < m_minimumPointsPerturbationThreshold)
	{
		
		int i;
		btlVector3 v0,v1;
		btlVector3 sepNormalWorldSpace;
		btlScalar l2 = gjkPairDetector.getCachedSeparatingAxis().length2();
	
		if (l2>SIMD_EPSILON)
		{
			sepNormalWorldSpace = gjkPairDetector.getCachedSeparatingAxis()*(1.f/l2);
			
			btlPlaneSpace1(sepNormalWorldSpace,v0,v1);


			bool perturbeA = true;
			const btlScalar angleLimit = 0.125f * SIMD_PI;
			btlScalar perturbeAngle;
			btlScalar radiusA = min0->getAngularMotionDisc();
			btlScalar radiusB = min1->getAngularMotionDisc();
			if (radiusA < radiusB)
			{
				perturbeAngle = btl_gContactBreakingThreshold /radiusA;
				perturbeA = true;
			} else
			{
				perturbeAngle = btl_gContactBreakingThreshold / radiusB;
				perturbeA = false;
			}
			if ( perturbeAngle > angleLimit ) 
					perturbeAngle = angleLimit;

			btlTransform unPerturbedTransform;
			if (perturbeA)
			{
				unPerturbedTransform = input.m_transformA;
			} else
			{
				unPerturbedTransform = input.m_transformB;
			}
			
			for ( i=0;i<m_numPerturbationIterations;i++)
			{
				if (v0.length2()>SIMD_EPSILON)
				{
				btlQuaternion perturbeRot(v0,perturbeAngle);
				btlScalar iterationAngle = i*(SIMD_2_PI/btlScalar(m_numPerturbationIterations));
				btlQuaternion rotq(sepNormalWorldSpace,iterationAngle);
				
				
				if (perturbeA)
				{
					input.m_transformA.setBasis(  btlMatrix3x3(rotq.inverse()*perturbeRot*rotq)*body0Wrap->getWorldTransform().getBasis());
					input.m_transformB = body1Wrap->getWorldTransform();
	#ifdef DEBUG_CONTACTS
					dispatchInfo.m_debugDraw->drawTransform(input.m_transformA,10.0);
	#endif //DEBUG_CONTACTS
				} else
				{
					input.m_transformA = body0Wrap->getWorldTransform();
					input.m_transformB.setBasis( btlMatrix3x3(rotq.inverse()*perturbeRot*rotq)*body1Wrap->getWorldTransform().getBasis());
	#ifdef DEBUG_CONTACTS
					dispatchInfo.m_debugDraw->drawTransform(input.m_transformB,10.0);
	#endif
				}
				
				btlPerturbedContactResult perturbedResultOut(resultOut,input.m_transformA,input.m_transformB,unPerturbedTransform,perturbeA,dispatchInfo.m_debugDraw);
				gjkPairDetector.getClosestPoints(input,perturbedResultOut,dispatchInfo.m_debugDraw);
				}
			}
		}
	}

	

#ifdef USE_SEPDISTANCE_UTIL2
	if (dispatchInfo.m_useConvexConservativeDistanceUtil && (sepDist>SIMD_EPSILON))
	{
		m_sepDistance.initSeparatingDistance(gjkPairDetector.getCachedSeparatingAxis(),sepDist,body0->getWorldTransform(),body1->getWorldTransform());
	}
#endif //USE_SEPDISTANCE_UTIL2


	}

	if (m_ownManifold)
	{
		resultOut->refreshContactPoints();
	}

}



bool btl_disableCcd = false;
btlScalar	btlConvexConvexAlgorithm::calculateTimeOfImpact(btlCollisionObject* col0,btlCollisionObject* col1,const btlDispatcherInfo& dispatchInfo,btlManifoldResult* resultOut)
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

	if (btl_disableCcd)
		return btlScalar(1.);


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

