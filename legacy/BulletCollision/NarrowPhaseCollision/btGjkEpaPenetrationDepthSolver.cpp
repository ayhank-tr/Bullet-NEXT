/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

EPA Copyright (c) Ricardo Padrela 2006

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "BulletCollision/CollisionShapes/btConvexShape.h"
#include "btGjkEpaPenetrationDepthSolver.h"


#include "BulletCollision/NarrowPhaseCollision/btGjkEpa2.h"

bool btlGjkEpaPenetrationDepthSolver::calcPenDepth( btlSimplexSolverInterface& simplexSolver,
											  const btlConvexShape* pConvexA, const btlConvexShape* pConvexB,
											  const btlTransform& transformA, const btlTransform& transformB,
											  btlVector3& v, btlVector3& wWitnessOnA, btlVector3& wWitnessOnB,
											  class btlIDebugDraw* debugDraw)
{

	(void)debugDraw;
	(void)v;
	(void)simplexSolver;

//	const btScalar				radialmargin(btScalar(0.));
	
	btlVector3	guessVector(transformB.getOrigin()-transformA.getOrigin());
	btlGjkEpaSolver2::sResults	results;
	

	if(btlGjkEpaSolver2::Penetration(pConvexA,transformA,
								pConvexB,transformB,
								guessVector,results))
	
		{
	//	debugDraw->drawLine(results.witnesses[1],results.witnesses[1]+results.normal,btVector3(255,0,0));
		//resultOut->addContactPoint(results.normal,results.witnesses[1],-results.depth);
		wWitnessOnA = results.witnesses[0];
		wWitnessOnB = results.witnesses[1];
		v = results.normal;
		return true;		
		} else
	{
		if(btlGjkEpaSolver2::Distance(pConvexA,transformA,pConvexB,transformB,guessVector,results))
		{
			wWitnessOnA = results.witnesses[0];
			wWitnessOnB = results.witnesses[1];
			v = results.normal;
			return false;
		}
	}

	return false;
}


