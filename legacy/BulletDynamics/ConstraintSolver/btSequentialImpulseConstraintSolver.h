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

#ifndef BT_SEQUENTIAL_IMPULSE_CONSTRAINT_SOLVER_H
#define BT_SEQUENTIAL_IMPULSE_CONSTRAINT_SOLVER_H

class btlIDebugDraw;
class btlPersistentManifold;
class btlDispatcher;
class btlCollisionObject;
#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"
#include "BulletDynamics/ConstraintSolver/btContactSolverInfo.h"
#include "BulletDynamics/ConstraintSolver/btSolverBody.h"
#include "BulletDynamics/ConstraintSolver/btSolverConstraint.h"
#include "BulletCollision/NarrowPhaseCollision/btManifoldPoint.h"
#include "BulletDynamics/ConstraintSolver/btConstraintSolver.h"

typedef btlSimdScalar(*btlSingleConstraintRowSolver)(btlSolverBody&, btlSolverBody&, const btlSolverConstraint&);

///The btSequentialImpulseConstraintSolver is a fast SIMD implementation of the Projected Gauss Seidel (iterative LCP) method.
ATTRIBUTE_ALIGNED16(class) btlSequentialImpulseConstraintSolver : public btlConstraintSolver
{
protected:
	btlAlignedObjectArray<btlSolverBody>      m_tmpSolverBodyPool;
	btlConstraintArray			m_tmpSolverContactConstraintPool;
	btlConstraintArray			m_tmpSolverNonContactConstraintPool;
	btlConstraintArray			m_tmpSolverContactFrictionConstraintPool;
	btlConstraintArray			m_tmpSolverContactRollingFrictionConstraintPool;

	btlAlignedObjectArray<int>	m_orderTmpConstraintPool;
	btlAlignedObjectArray<int>	m_orderNonContactConstraintPool;
	btlAlignedObjectArray<int>	m_orderFrictionConstraintPool;
	btlAlignedObjectArray<btlTypedConstraint::btlConstraintInfo1> m_tmpConstraintSizesPool;
	int							m_maxOverrideNumSolverIterations;
	int m_fixedBodyId;

	btlSingleConstraintRowSolver m_resolveSingleConstraintRowGeneric;
	btlSingleConstraintRowSolver m_resolveSingleConstraintRowLowerLimit;

	void setupFrictionConstraint(	btlSolverConstraint& solverConstraint, const btlVector3& normalAxis,int solverBodyIdA,int  solverBodyIdB,
									btlManifoldPoint& cp,const btlVector3& rel_pos1,const btlVector3& rel_pos2,
									btlCollisionObject* colObj0,btlCollisionObject* colObj1, btlScalar relaxation, 
									btlScalar desiredVelocity=0., btlScalar cfmSlip=0.);

	void setupRollingFrictionConstraint(	btlSolverConstraint& solverConstraint, const btlVector3& normalAxis,int solverBodyIdA,int  solverBodyIdB,
									btlManifoldPoint& cp,const btlVector3& rel_pos1,const btlVector3& rel_pos2,
									btlCollisionObject* colObj0,btlCollisionObject* colObj1, btlScalar relaxation, 
									btlScalar desiredVelocity=0., btlScalar cfmSlip=0.);

	btlSolverConstraint&	addFrictionConstraint(const btlVector3& normalAxis,int solverBodyIdA,int solverBodyIdB,int frictionIndex,btlManifoldPoint& cp,const btlVector3& rel_pos1,const btlVector3& rel_pos2,btlCollisionObject* colObj0,btlCollisionObject* colObj1, btlScalar relaxation, btlScalar desiredVelocity=0., btlScalar cfmSlip=0.);
	btlSolverConstraint&	addRollingFrictionConstraint(const btlVector3& normalAxis,int solverBodyIdA,int solverBodyIdB,int frictionIndex,btlManifoldPoint& cp,const btlVector3& rel_pos1,const btlVector3& rel_pos2,btlCollisionObject* colObj0,btlCollisionObject* colObj1, btlScalar relaxation, btlScalar desiredVelocity=0, btlScalar cfmSlip=0.f);

	
	void setupContactConstraint(btlSolverConstraint& solverConstraint, int solverBodyIdA, int solverBodyIdB, btlManifoldPoint& cp, 
								const btlContactSolverInfo& infoGlobal,btlScalar& relaxation, const btlVector3& rel_pos1, const btlVector3& rel_pos2);

	static void	applyAnisotropicFriction(btlCollisionObject* colObj,btlVector3& frictionDirection, int frictionMode);

	void setFrictionConstraintImpulse( btlSolverConstraint& solverConstraint, int solverBodyIdA,int solverBodyIdB, 
										 btlManifoldPoint& cp, const btlContactSolverInfo& infoGlobal);

	///m_btSeed2 is used for re-arranging the constraint rows. improves convergence/quality of friction
	unsigned long	m_btSeed2;

	
	btlScalar restitutionCurve(btlScalar rel_vel, btlScalar restitution);

	virtual void convertContacts(btlPersistentManifold** manifoldPtr, int numManifolds, const btlContactSolverInfo& infoGlobal);

	void	convertContact(btlPersistentManifold* manifold,const btlContactSolverInfo& infoGlobal);


	void	resolveSplitPenetrationSIMD(
     btlSolverBody& bodyA,btlSolverBody& bodyB,
        const btlSolverConstraint& contactConstraint);

	void	resolveSplitPenetrationImpulseCacheFriendly(
       btlSolverBody& bodyA,btlSolverBody& bodyB,
        const btlSolverConstraint& contactConstraint);

	//internal method
	int		getOrInitSolverBody(btlCollisionObject& body,btlScalar timeStep);
	void	initSolverBody(btlSolverBody* solverBody, btlCollisionObject* collisionObject, btlScalar timeStep);

	btlSimdScalar	resolveSingleConstraintRowGeneric(btlSolverBody& bodyA,btlSolverBody& bodyB,const btlSolverConstraint& contactConstraint);
	btlSimdScalar	resolveSingleConstraintRowGenericSIMD(btlSolverBody& bodyA,btlSolverBody& bodyB,const btlSolverConstraint& contactConstraint);
	btlSimdScalar	resolveSingleConstraintRowLowerLimit(btlSolverBody& bodyA,btlSolverBody& bodyB,const btlSolverConstraint& contactConstraint);
	btlSimdScalar	resolveSingleConstraintRowLowerLimitSIMD(btlSolverBody& bodyA,btlSolverBody& bodyB,const btlSolverConstraint& contactConstraint);
		
protected:
	
	
	virtual void solveGroupCacheFriendlySplitImpulseIterations(btlCollisionObject** bodies,int numBodies,btlPersistentManifold** manifoldPtr, int numManifolds,btlTypedConstraint** constraints,int numConstraints,const btlContactSolverInfo& infoGlobal,btlIDebugDraw* debugDrawer);
	virtual btlScalar solveGroupCacheFriendlyFinish(btlCollisionObject** bodies,int numBodies,const btlContactSolverInfo& infoGlobal);
	virtual btlScalar solveSingleIteration(int iteration, btlCollisionObject** bodies ,int numBodies,btlPersistentManifold** manifoldPtr, int numManifolds,btlTypedConstraint** constraints,int numConstraints,const btlContactSolverInfo& infoGlobal,btlIDebugDraw* debugDrawer);

	virtual btlScalar solveGroupCacheFriendlySetup(btlCollisionObject** bodies,int numBodies,btlPersistentManifold** manifoldPtr, int numManifolds,btlTypedConstraint** constraints,int numConstraints,const btlContactSolverInfo& infoGlobal,btlIDebugDraw* debugDrawer);
	virtual btlScalar solveGroupCacheFriendlyIterations(btlCollisionObject** bodies,int numBodies,btlPersistentManifold** manifoldPtr, int numManifolds,btlTypedConstraint** constraints,int numConstraints,const btlContactSolverInfo& infoGlobal,btlIDebugDraw* debugDrawer);


public:

	BT_DECLARE_ALIGNED_ALLOCATOR();
	
	btlSequentialImpulseConstraintSolver();
	virtual ~btlSequentialImpulseConstraintSolver();

	virtual btlScalar solveGroup(btlCollisionObject** bodies,int numBodies,btlPersistentManifold** manifold,int numManifolds,btlTypedConstraint** constraints,int numConstraints,const btlContactSolverInfo& info, btlIDebugDraw* debugDrawer,btlDispatcher* dispatcher);
		
	///clear internal cached data and reset random seed
	virtual	void	reset();
	
	unsigned long btlRand2();

	int btlRandInt2 (int n);

	void	setRandSeed(unsigned long seed)
	{
		m_btSeed2 = seed;
	}
	unsigned long	getRandSeed() const
	{
		return m_btSeed2;
	}

	
	virtual btlConstraintSolverType	getSolverType() const
	{
		return BT_SEQUENTIAL_IMPULSE_SOLVER;
	}

	btlSingleConstraintRowSolver	getActiveConstraintRowSolverGeneric()
	{
		return m_resolveSingleConstraintRowGeneric;
	}
	void setConstraintRowSolverGeneric(btlSingleConstraintRowSolver rowSolver)
	{
		m_resolveSingleConstraintRowGeneric = rowSolver;
	}
	btlSingleConstraintRowSolver	getActiveConstraintRowSolverLowerLimit()
	{
		return m_resolveSingleConstraintRowLowerLimit;
	}
	void setConstraintRowSolverLowerLimit(btlSingleConstraintRowSolver rowSolver)
	{
		m_resolveSingleConstraintRowLowerLimit = rowSolver;
	}

	///Various implementations of solving a single constraint row using a generic equality constraint, using scalar reference, SSE2 or SSE4
	btlSingleConstraintRowSolver	getScalarConstraintRowSolverGeneric();
	btlSingleConstraintRowSolver	getSSE2ConstraintRowSolverGeneric();
	btlSingleConstraintRowSolver	getSSE4_1ConstraintRowSolverGeneric();

	///Various implementations of solving a single constraint row using an inequality (lower limit) constraint, using scalar reference, SSE2 or SSE4
	btlSingleConstraintRowSolver	getScalarConstraintRowSolverLowerLimit();
	btlSingleConstraintRowSolver	getSSE2ConstraintRowSolverLowerLimit();
	btlSingleConstraintRowSolver	getSSE4_1ConstraintRowSolverLowerLimit();
};




#endif //BT_SEQUENTIAL_IMPULSE_CONSTRAINT_SOLVER_H

