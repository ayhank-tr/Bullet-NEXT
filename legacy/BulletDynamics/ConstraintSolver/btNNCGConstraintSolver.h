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

#ifndef BT_NNCG_CONSTRAINT_SOLVER_H
#define BT_NNCG_CONSTRAINT_SOLVER_H

#include "btSequentialImpulseConstraintSolver.h"

ATTRIBUTE_ALIGNED16(class) btlNNCGConstraintSolver : public btlSequentialImpulseConstraintSolver
{
protected:

	btlScalar m_deltafLengthSqrPrev;

	btlAlignedObjectArray<btlScalar> m_pNC;  // p for None Contact constraints
	btlAlignedObjectArray<btlScalar> m_pC;   // p for Contact constraints
	btlAlignedObjectArray<btlScalar> m_pCF;  // p for ContactFriction constraints
	btlAlignedObjectArray<btlScalar> m_pCRF; // p for ContactRollingFriction constraints

	//These are recalculated in every iterations. We just keep these to prevent reallocation in each iteration.
	btlAlignedObjectArray<btlScalar> m_deltafNC;  // deltaf for NoneContact constraints
	btlAlignedObjectArray<btlScalar> m_deltafC;   // deltaf for Contact constraints
	btlAlignedObjectArray<btlScalar> m_deltafCF;  // deltaf for ContactFriction constraints
	btlAlignedObjectArray<btlScalar> m_deltafCRF; // deltaf for ContactRollingFriction constraints

		
protected:

	virtual btlScalar solveGroupCacheFriendlyFinish(btlCollisionObject** bodies,int numBodies,const btlContactSolverInfo& infoGlobal);
	virtual btlScalar solveSingleIteration(int iteration, btlCollisionObject** bodies ,int numBodies,btlPersistentManifold** manifoldPtr, int numManifolds,btlTypedConstraint** constraints,int numConstraints,const btlContactSolverInfo& infoGlobal,btlIDebugDraw* debugDrawer);

	virtual btlScalar solveGroupCacheFriendlySetup(btlCollisionObject** bodies,int numBodies,btlPersistentManifold** manifoldPtr, int numManifolds,btlTypedConstraint** constraints,int numConstraints,const btlContactSolverInfo& infoGlobal,btlIDebugDraw* debugDrawer);

public:

	BT_DECLARE_ALIGNED_ALLOCATOR();

	btlNNCGConstraintSolver() : btlSequentialImpulseConstraintSolver(), m_onlyForNoneContact(false) {}

	virtual btlConstraintSolverType getSolverType() const
	{
		return BT_NNCG_SOLVER;
	}

	bool m_onlyForNoneContact;
};




#endif //BT_NNCG_CONSTRAINT_SOLVER_H

