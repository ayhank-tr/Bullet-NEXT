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


#include "btContinuousConvexCollision.h"
#include "BulletCollision/CollisionShapes/btConvexShape.h"
#include "BulletCollision/NarrowPhaseCollision/btSimplexSolverInterface.h"
#include "LinearMath/btTransformUtil.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"

#include "btGjkPairDetector.h"
#include "btPointCollector.h"
#include "BulletCollision/CollisionShapes/btStaticPlaneShape.h"



btlContinuousConvexCollision::btlContinuousConvexCollision ( const btlConvexShape*	convexA,const btlConvexShape*	convexB,btlSimplexSolverInterface* simplexSolver, btlConvexPenetrationDepthSolver* penetrationDepthSolver)
:m_simplexSolver(simplexSolver),
m_penetrationDepthSolver(penetrationDepthSolver),
m_convexA(convexA),m_convexB1(convexB),m_planeShape(0)
{
}


btlContinuousConvexCollision::btlContinuousConvexCollision( const btlConvexShape*	convexA,const btlStaticPlaneShape*	plane)
:m_simplexSolver(0),
m_penetrationDepthSolver(0),
m_convexA(convexA),m_convexB1(0),m_planeShape(plane)
{
}


/// This maximum should not be necessary. It allows for untested/degenerate cases in production code.
/// You don't want your game ever to lock-up.
#define MAX_ITERATIONS 64

void btlContinuousConvexCollision::computeClosestPoints( const btlTransform& transA, const btlTransform& transB,btlPointCollector& pointCollector)
{
	if (m_convexB1)
	{
		m_simplexSolver->reset();
		btlGjkPairDetector gjk(m_convexA,m_convexB1,m_convexA->getShapeType(),m_convexB1->getShapeType(),m_convexA->getMargin(),m_convexB1->getMargin(),m_simplexSolver,m_penetrationDepthSolver);		
		btlGjkPairDetector::ClosestPointInput input;
		input.m_transformA = transA;
		input.m_transformB = transB;
		gjk.getClosestPoints(input,pointCollector,0);
	} else
	{
		//convex versus plane
		const btlConvexShape* convexShape = m_convexA;
		const btlStaticPlaneShape* planeShape = m_planeShape;
		
		const btlVector3& planeNormal = planeShape->getPlaneNormal();
		const btlScalar& planeConstant = planeShape->getPlaneConstant();
		
		btlTransform convexWorldTransform = transA;
		btlTransform convexInPlaneTrans;
		convexInPlaneTrans= transB.inverse() * convexWorldTransform;
		btlTransform planeInConvex;
		planeInConvex= convexWorldTransform.inverse() * transB;
		
		btlVector3 vtx = convexShape->localGetSupportingVertex(planeInConvex.getBasis()*-planeNormal);

		btlVector3 vtxInPlane = convexInPlaneTrans(vtx);
		btlScalar distance = (planeNormal.dot(vtxInPlane) - planeConstant);

		btlVector3 vtxInPlaneProjected = vtxInPlane - distance*planeNormal;
		btlVector3 vtxInPlaneWorld = transB * vtxInPlaneProjected;
		btlVector3 normalOnSurfaceB = transB.getBasis() * planeNormal;

		pointCollector.addContactPoint(
			normalOnSurfaceB,
			vtxInPlaneWorld,
			distance);
	}
}

bool	btlContinuousConvexCollision::calcTimeOfImpact(
				const btlTransform& fromA,
				const btlTransform& toA,
				const btlTransform& fromB,
				const btlTransform& toB,
				CastResult& result)
{


	/// compute linear and angular velocity for this interval, to interpolate
	btlVector3 linVelA,angVelA,linVelB,angVelB;
	btlTransformUtil::calculateVelocity(fromA,toA,btlScalar(1.),linVelA,angVelA);
	btlTransformUtil::calculateVelocity(fromB,toB,btlScalar(1.),linVelB,angVelB);


	btlScalar boundingRadiusA = m_convexA->getAngularMotionDisc();
	btlScalar boundingRadiusB = m_convexB1?m_convexB1->getAngularMotionDisc():0.f;

	btlScalar maxAngularProjectedVelocity = angVelA.length() * boundingRadiusA + angVelB.length() * boundingRadiusB;
	btlVector3 relLinVel = (linVelB-linVelA);

	btlScalar relLinVelocLength = (linVelB-linVelA).length();
	
	if ((relLinVelocLength+maxAngularProjectedVelocity) == 0.f)
		return false;



	btlScalar lambda = btlScalar(0.);
	btlVector3 v(1,0,0);

	int maxIter = MAX_ITERATIONS;

	btlVector3 n;
	n.setValue(btlScalar(0.),btlScalar(0.),btlScalar(0.));
	bool hasResult = false;
	btlVector3 c;

	btlScalar lastLambda = lambda;
	//btScalar epsilon = btScalar(0.001);

	int numIter = 0;
	//first solution, using GJK


	btlScalar radius = 0.001f;
//	result.drawCoordSystem(sphereTr);

	btlPointCollector	pointCollector1;

	{
	
		computeClosestPoints(fromA,fromB,pointCollector1);

		hasResult = pointCollector1.m_hasResult;
		c = pointCollector1.m_pointInWorld;
	}

	if (hasResult)
	{
		btlScalar dist;
		dist = pointCollector1.m_distance + result.m_allowedPenetration;
		n = pointCollector1.m_normalOnBInWorld;
		btlScalar projectedLinearVelocity = relLinVel.dot(n);
		if ((projectedLinearVelocity+ maxAngularProjectedVelocity)<=SIMD_EPSILON)
			return false;

		//not close enough
		while (dist > radius)
		{
			if (result.m_debugDrawer)
			{
				result.m_debugDrawer->drawSphere(c,0.2f,btlVector3(1,1,1));
			}
			btlScalar dLambda = btlScalar(0.);

			projectedLinearVelocity = relLinVel.dot(n);

			
			//don't report time of impact for motion away from the contact normal (or causes minor penetration)
			if ((projectedLinearVelocity+ maxAngularProjectedVelocity)<=SIMD_EPSILON)
				return false;
			
			dLambda = dist / (projectedLinearVelocity+ maxAngularProjectedVelocity);

			
			
			lambda = lambda + dLambda;

			if (lambda > btlScalar(1.))
				return false;

			if (lambda < btlScalar(0.))
				return false;


			//todo: next check with relative epsilon
			if (lambda <= lastLambda)
			{
				return false;
				//n.setValue(0,0,0);
				break;
			}
			lastLambda = lambda;

			

			//interpolate to next lambda
			btlTransform interpolatedTransA,interpolatedTransB,relativeTrans;

			btlTransformUtil::integrateTransform(fromA,linVelA,angVelA,lambda,interpolatedTransA);
			btlTransformUtil::integrateTransform(fromB,linVelB,angVelB,lambda,interpolatedTransB);
			relativeTrans = interpolatedTransB.inverseTimes(interpolatedTransA);

			if (result.m_debugDrawer)
			{
				result.m_debugDrawer->drawSphere(interpolatedTransA.getOrigin(),0.2f,btlVector3(1,0,0));
			}

			result.DebugDraw( lambda );

			btlPointCollector	pointCollector;
			computeClosestPoints(interpolatedTransA,interpolatedTransB,pointCollector);

			if (pointCollector.m_hasResult)
			{
				dist = pointCollector.m_distance+result.m_allowedPenetration;
				c = pointCollector.m_pointInWorld;		
				n = pointCollector.m_normalOnBInWorld;
			} else
			{
				result.reportFailure(-1, numIter);
				return false;
			}

			numIter++;
			if (numIter > maxIter)
			{
				result.reportFailure(-2, numIter);
				return false;
			}
		}
	
		result.m_fraction = lambda;
		result.m_normal = n;
		result.m_hitPoint = c;
		return true;
	}

	return false;

}

