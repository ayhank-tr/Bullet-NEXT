/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2013 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
///original version written by Erwin Coumans, October 2013

#ifndef BT_MLCP_SOLVER_H
#define BT_MLCP_SOLVER_H

#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "LinearMath/btMatrixX.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolverInterface.h"

class btlMLCPSolver : public btlSequentialImpulseConstraintSolver
{

protected:
	
	btlMatrixXu m_A;
	btlVectorXu m_b;
	btlVectorXu m_x;
	btlVectorXu m_lo;
	btlVectorXu m_hi;
	
	///when using 'split impulse' we solve two separate (M)LCPs
	btlVectorXu m_bSplit;
	btlVectorXu m_xSplit;
	btlVectorXu m_bSplit1;
	btlVectorXu m_xSplit2;

	btlAlignedObjectArray<int> m_limitDependencies;
	btlAlignedObjectArray<btlSolverConstraint*>	m_allConstraintPtrArray;
	btlMLCPSolverInterface* m_solver;
	int m_fallback;
	btlScalar m_cfm;

	virtual btlScalar solveGroupCacheFriendlySetup(btlCollisionObject** bodies, int numBodies, btlPersistentManifold** manifoldPtr, int numManifolds,btlTypedConstraint** constraints,int numConstraints,const btlContactSolverInfo& infoGlobal,btlIDebugDraw* debugDrawer);
	virtual btlScalar solveGroupCacheFriendlyIterations(btlCollisionObject** bodies ,int numBodies,btlPersistentManifold** manifoldPtr, int numManifolds,btlTypedConstraint** constraints,int numConstraints,const btlContactSolverInfo& infoGlobal,btlIDebugDraw* debugDrawer);


	virtual void createMLCP(const btlContactSolverInfo& infoGlobal);
	virtual void createMLCPFast(const btlContactSolverInfo& infoGlobal);

	//return true is it solves the problem successfully
	virtual bool solveMLCP(const btlContactSolverInfo& infoGlobal);

public:

	btlMLCPSolver(	 btlMLCPSolverInterface* solver);
	virtual ~btlMLCPSolver();

	void setMLCPSolver(btlMLCPSolverInterface* solver)
	{
		m_solver = solver;
	}

	int getNumFallbacks() const
	{
		return m_fallback;
	}
	void setNumFallbacks(int num)
	{
		m_fallback = num;
	}

	btlScalar	getCfm() const
	{
		return m_cfm;
	}
	void setCfm(btlScalar cfm)
	{
		m_cfm = cfm;
	}

	virtual btlConstraintSolverType	getSolverType() const
	{
		return BT_MLCP_SOLVER;
	}

};


#endif //BT_MLCP_SOLVER_H
