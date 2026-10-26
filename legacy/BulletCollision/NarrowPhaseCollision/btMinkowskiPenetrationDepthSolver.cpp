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

#include "btMinkowskiPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.h"
#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
#include "BulletCollision/CollisionShapes/btConvexShape.h"

#define NUM_UNITSPHERE_POINTS 42


bool btlMinkowskiPenetrationDepthSolver::calcPenDepth(btlSimplexSolverInterface& simplexSolver,
												   const btlConvexShape* convexA,const btlConvexShape* convexB,
												   const btlTransform& transA,const btlTransform& transB,
												   btlVector3& v, btlVector3& pa, btlVector3& pb,
												   class btlIDebugDraw* debugDraw
												   )
{

	(void)v;
	
	bool check2d= convexA->isConvex2d() && convexB->isConvex2d();

	struct btlIntermediateResult : public btlDiscreteCollisionDetectorInterface::Result
	{

		btlIntermediateResult():m_hasResult(false)
		{
		}
		
		btlVector3 m_normalOnBInWorld;
		btlVector3 m_pointInWorld;
		btlScalar m_depth;
		bool	m_hasResult;

		virtual void setShapeIdentifiersA(int partId0,int index0)
		{
			(void)partId0;
			(void)index0;
		}
		virtual void setShapeIdentifiersB(int partId1,int index1)
		{
			(void)partId1;
			(void)index1;
		}
		void addContactPoint(const btlVector3& normalOnBInWorld,const btlVector3& pointInWorld,btlScalar depth)
		{
			m_normalOnBInWorld = normalOnBInWorld;
			m_pointInWorld = pointInWorld;
			m_depth = depth;
			m_hasResult = true;
		}
	};

	//just take fixed number of orientation, and sample the penetration depth in that direction
	btlScalar minProj = btlScalar(BT_LARGE_FLOAT);
	btlVector3 minNorm(btlScalar(0.), btlScalar(0.), btlScalar(0.));
	btlVector3 minA,minB;
	btlVector3 seperatingAxisInA,seperatingAxisInB;
	btlVector3 pInA,qInB,pWorld,qWorld,w;

#ifndef __SPU__
#define USE_BATCHED_SUPPORT 1
#endif
#ifdef USE_BATCHED_SUPPORT

	btlVector3	supportVerticesABatch[NUM_UNITSPHERE_POINTS+MAX_PREFERRED_PENETRATION_DIRECTIONS*2];
	btlVector3	supportVerticesBBatch[NUM_UNITSPHERE_POINTS+MAX_PREFERRED_PENETRATION_DIRECTIONS*2];
	btlVector3	seperatingAxisInABatch[NUM_UNITSPHERE_POINTS+MAX_PREFERRED_PENETRATION_DIRECTIONS*2];
	btlVector3	seperatingAxisInBBatch[NUM_UNITSPHERE_POINTS+MAX_PREFERRED_PENETRATION_DIRECTIONS*2];
	int i;

	int numSampleDirections = NUM_UNITSPHERE_POINTS;

	for (i=0;i<numSampleDirections;i++)
	{
		btlVector3 norm = getPenetrationDirections()[i];
		seperatingAxisInABatch[i] =  (-norm) * transA.getBasis() ;
		seperatingAxisInBBatch[i] =  norm   * transB.getBasis() ;
	}

	{
		int numPDA = convexA->getNumPreferredPenetrationDirections();
		if (numPDA)
		{
			for (int i=0;i<numPDA;i++)
			{
				btlVector3 norm;
				convexA->getPreferredPenetrationDirection(i,norm);
				norm  = transA.getBasis() * norm;
				getPenetrationDirections()[numSampleDirections] = norm;
				seperatingAxisInABatch[numSampleDirections] = (-norm) * transA.getBasis();
				seperatingAxisInBBatch[numSampleDirections] = norm * transB.getBasis();
				numSampleDirections++;
			}
		}
	}

	{
		int numPDB = convexB->getNumPreferredPenetrationDirections();
		if (numPDB)
		{
			for (int i=0;i<numPDB;i++)
			{
				btlVector3 norm;
				convexB->getPreferredPenetrationDirection(i,norm);
				norm  = transB.getBasis() * norm;
				getPenetrationDirections()[numSampleDirections] = norm;
				seperatingAxisInABatch[numSampleDirections] = (-norm) * transA.getBasis();
				seperatingAxisInBBatch[numSampleDirections] = norm * transB.getBasis();
				numSampleDirections++;
			}
		}
	}




	convexA->batchedUnitVectorGetSupportingVertexWithoutMargin(seperatingAxisInABatch,supportVerticesABatch,numSampleDirections);
	convexB->batchedUnitVectorGetSupportingVertexWithoutMargin(seperatingAxisInBBatch,supportVerticesBBatch,numSampleDirections);

	for (i=0;i<numSampleDirections;i++)
	{
		btlVector3 norm = getPenetrationDirections()[i];
		if (check2d)
		{
			norm[2] = 0.f;
		}
		if (norm.length2()>0.01)
		{

			seperatingAxisInA = seperatingAxisInABatch[i];
			seperatingAxisInB = seperatingAxisInBBatch[i];

			pInA = supportVerticesABatch[i];
			qInB = supportVerticesBBatch[i];

			pWorld = transA(pInA);	
			qWorld = transB(qInB);
			if (check2d)
			{
				pWorld[2] = 0.f;
				qWorld[2] = 0.f;
			}

			w	= qWorld - pWorld;
			btlScalar delta = norm.dot(w);
			//find smallest delta
			if (delta < minProj)
			{
				minProj = delta;
				minNorm = norm;
				minA = pWorld;
				minB = qWorld;
			}
		}
	}	
#else

	int numSampleDirections = NUM_UNITSPHERE_POINTS;

#ifndef __SPU__
	{
		int numPDA = convexA->getNumPreferredPenetrationDirections();
		if (numPDA)
		{
			for (int i=0;i<numPDA;i++)
			{
				btlVector3 norm;
				convexA->getPreferredPenetrationDirection(i,norm);
				norm  = transA.getBasis() * norm;
				getPenetrationDirections()[numSampleDirections] = norm;
				numSampleDirections++;
			}
		}
	}

	{
		int numPDB = convexB->getNumPreferredPenetrationDirections();
		if (numPDB)
		{
			for (int i=0;i<numPDB;i++)
			{
				btlVector3 norm;
				convexB->getPreferredPenetrationDirection(i,norm);
				norm  = transB.getBasis() * norm;
				getPenetrationDirections()[numSampleDirections] = norm;
				numSampleDirections++;
			}
		}
	}
#endif // __SPU__

	for (int i=0;i<numSampleDirections;i++)
	{
		const btlVector3& norm = getPenetrationDirections()[i];
		seperatingAxisInA = (-norm)* transA.getBasis();
		seperatingAxisInB = norm* transB.getBasis();
		pInA = convexA->localGetSupportVertexWithoutMarginNonVirtual(seperatingAxisInA);
		qInB = convexB->localGetSupportVertexWithoutMarginNonVirtual(seperatingAxisInB);
		pWorld = transA(pInA);	
		qWorld = transB(qInB);
		w	= qWorld - pWorld;
		btlScalar delta = norm.dot(w);
		//find smallest delta
		if (delta < minProj)
		{
			minProj = delta;
			minNorm = norm;
			minA = pWorld;
			minB = qWorld;
		}
	}
#endif //USE_BATCHED_SUPPORT

	//add the margins

	minA += minNorm*convexA->getMarginNonVirtual();
	minB -= minNorm*convexB->getMarginNonVirtual();
	//no penetration
	if (minProj < btlScalar(0.))
		return false;

	btlScalar extraSeparation = 0.5f;///scale dependent
	minProj += extraSeparation+(convexA->getMarginNonVirtual() + convexB->getMarginNonVirtual());





//#define DEBUG_DRAW 1
#ifdef DEBUG_DRAW
	if (debugDraw)
	{
		btlVector3 color(0,1,0);
		debugDraw->drawLine(minA,minB,color);
		color = btlVector3 (1,1,1);
		btlVector3 vec = minB-minA;
		btlScalar prj2 = minNorm.dot(vec);
		debugDraw->drawLine(minA,minA+(minNorm*minProj),color);

	}
#endif //DEBUG_DRAW

	

	btlGjkPairDetector gjkdet(convexA,convexB,&simplexSolver,0);

	btlScalar offsetDist = minProj;
	btlVector3 offset = minNorm * offsetDist;
	


	btlGjkPairDetector::ClosestPointInput input;
		
	btlVector3 newOrg = transA.getOrigin() + offset;

	btlTransform displacedTrans = transA;
	displacedTrans.setOrigin(newOrg);

	input.m_transformA = displacedTrans;
	input.m_transformB = transB;
	input.m_maximumDistanceSquared = btlScalar(BT_LARGE_FLOAT);//minProj;
	
	btlIntermediateResult res;
	gjkdet.setCachedSeperatingAxis(-minNorm);
	gjkdet.getClosestPoints(input,res,debugDraw);

	btlScalar correctedMinNorm = minProj - res.m_depth;


	//the penetration depth is over-estimated, relax it
	btlScalar penetration_relaxation= btlScalar(1.);
	minNorm*=penetration_relaxation;
	

	if (res.m_hasResult)
	{

		pa = res.m_pointInWorld - minNorm * correctedMinNorm;
		pb = res.m_pointInWorld;
		v = minNorm;
		
#ifdef DEBUG_DRAW
		if (debugDraw)
		{
			btlVector3 color(1,0,0);
			debugDraw->drawLine(pa,pb,color);
		}
#endif//DEBUG_DRAW


	}
	return res.m_hasResult;
}

btlVector3*	btlMinkowskiPenetrationDepthSolver::getPenetrationDirections()
{
	static btlVector3	sPenetrationDirections[NUM_UNITSPHERE_POINTS+MAX_PREFERRED_PENETRATION_DIRECTIONS*2] = 
	{
	btlVector3(btlScalar(0.000000) , btlScalar(-0.000000),btlScalar(-1.000000)),
	btlVector3(btlScalar(0.723608) , btlScalar(-0.525725),btlScalar(-0.447219)),
	btlVector3(btlScalar(-0.276388) , btlScalar(-0.850649),btlScalar(-0.447219)),
	btlVector3(btlScalar(-0.894426) , btlScalar(-0.000000),btlScalar(-0.447216)),
	btlVector3(btlScalar(-0.276388) , btlScalar(0.850649),btlScalar(-0.447220)),
	btlVector3(btlScalar(0.723608) , btlScalar(0.525725),btlScalar(-0.447219)),
	btlVector3(btlScalar(0.276388) , btlScalar(-0.850649),btlScalar(0.447220)),
	btlVector3(btlScalar(-0.723608) , btlScalar(-0.525725),btlScalar(0.447219)),
	btlVector3(btlScalar(-0.723608) , btlScalar(0.525725),btlScalar(0.447219)),
	btlVector3(btlScalar(0.276388) , btlScalar(0.850649),btlScalar(0.447219)),
	btlVector3(btlScalar(0.894426) , btlScalar(0.000000),btlScalar(0.447216)),
	btlVector3(btlScalar(-0.000000) , btlScalar(0.000000),btlScalar(1.000000)),
	btlVector3(btlScalar(0.425323) , btlScalar(-0.309011),btlScalar(-0.850654)),
	btlVector3(btlScalar(-0.162456) , btlScalar(-0.499995),btlScalar(-0.850654)),
	btlVector3(btlScalar(0.262869) , btlScalar(-0.809012),btlScalar(-0.525738)),
	btlVector3(btlScalar(0.425323) , btlScalar(0.309011),btlScalar(-0.850654)),
	btlVector3(btlScalar(0.850648) , btlScalar(-0.000000),btlScalar(-0.525736)),
	btlVector3(btlScalar(-0.525730) , btlScalar(-0.000000),btlScalar(-0.850652)),
	btlVector3(btlScalar(-0.688190) , btlScalar(-0.499997),btlScalar(-0.525736)),
	btlVector3(btlScalar(-0.162456) , btlScalar(0.499995),btlScalar(-0.850654)),
	btlVector3(btlScalar(-0.688190) , btlScalar(0.499997),btlScalar(-0.525736)),
	btlVector3(btlScalar(0.262869) , btlScalar(0.809012),btlScalar(-0.525738)),
	btlVector3(btlScalar(0.951058) , btlScalar(0.309013),btlScalar(0.000000)),
	btlVector3(btlScalar(0.951058) , btlScalar(-0.309013),btlScalar(0.000000)),
	btlVector3(btlScalar(0.587786) , btlScalar(-0.809017),btlScalar(0.000000)),
	btlVector3(btlScalar(0.000000) , btlScalar(-1.000000),btlScalar(0.000000)),
	btlVector3(btlScalar(-0.587786) , btlScalar(-0.809017),btlScalar(0.000000)),
	btlVector3(btlScalar(-0.951058) , btlScalar(-0.309013),btlScalar(-0.000000)),
	btlVector3(btlScalar(-0.951058) , btlScalar(0.309013),btlScalar(-0.000000)),
	btlVector3(btlScalar(-0.587786) , btlScalar(0.809017),btlScalar(-0.000000)),
	btlVector3(btlScalar(-0.000000) , btlScalar(1.000000),btlScalar(-0.000000)),
	btlVector3(btlScalar(0.587786) , btlScalar(0.809017),btlScalar(-0.000000)),
	btlVector3(btlScalar(0.688190) , btlScalar(-0.499997),btlScalar(0.525736)),
	btlVector3(btlScalar(-0.262869) , btlScalar(-0.809012),btlScalar(0.525738)),
	btlVector3(btlScalar(-0.850648) , btlScalar(0.000000),btlScalar(0.525736)),
	btlVector3(btlScalar(-0.262869) , btlScalar(0.809012),btlScalar(0.525738)),
	btlVector3(btlScalar(0.688190) , btlScalar(0.499997),btlScalar(0.525736)),
	btlVector3(btlScalar(0.525730) , btlScalar(0.000000),btlScalar(0.850652)),
	btlVector3(btlScalar(0.162456) , btlScalar(-0.499995),btlScalar(0.850654)),
	btlVector3(btlScalar(-0.425323) , btlScalar(-0.309011),btlScalar(0.850654)),
	btlVector3(btlScalar(-0.425323) , btlScalar(0.309011),btlScalar(0.850654)),
	btlVector3(btlScalar(0.162456) , btlScalar(0.499995),btlScalar(0.850654))
	};

	return sPenetrationDirections;
}


