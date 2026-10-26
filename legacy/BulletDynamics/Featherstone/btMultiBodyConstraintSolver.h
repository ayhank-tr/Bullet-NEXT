/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2013 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_MULTIBODY_CONSTRAINT_SOLVER_H
#define BT_MULTIBODY_CONSTRAINT_SOLVER_H

#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "btMultiBodySolverConstraint.h"

#define DIRECTLY_UPDATE_VELOCITY_DURING_SOLVER_ITERATIONS

class btlMultiBody;

#include "btMultiBodyConstraint.h"



ATTRIBUTE_ALIGNED16(class) btlMultiBodyConstraintSolver : public btlSequentialImpulseConstraintSolver
{

protected:

	btlMultiBodyConstraintArray			m_multiBodyNonContactConstraints;

	btlMultiBodyConstraintArray			m_multiBodyNormalContactConstraints;
	btlMultiBodyConstraintArray			m_multiBodyFrictionContactConstraints;

	btlMultiBodyJacobianData				m_data;
	
	//temp storage for multi body constraints for a specific island/group called by 'solveGroup'
	btlMultiBodyConstraint**					m_tmpMultiBodyConstraints;
	int										m_tmpNumMultiBodyConstraints;

	void resolveSingleConstraintRowGeneric(const btlMultiBodySolverConstraint& c);
	

	void convertContacts(btlPersistentManifold** manifoldPtr,int numManifolds, const btlContactSolverInfo& infoGlobal);
	btlMultiBodySolverConstraint&	addMultiBodyFrictionConstraint(const btlVector3& normalAxis,btlPersistentManifold* manifold,int frictionIndex,btlManifoldPoint& cp,btlCollisionObject* colObj0,btlCollisionObject* colObj1, btlScalar relaxation, const btlContactSolverInfo& infoGlobal, btlScalar desiredVelocity=0, btlScalar cfmSlip=0);


	void setupMultiBodyJointLimitConstraint(btlMultiBodySolverConstraint& constraintRow, 
																 btlScalar* jacA,btlScalar* jacB,
																 btlScalar penetration,btlScalar combinedFrictionCoeff, btlScalar combinedRestitutionCoeff,
																 const btlContactSolverInfo& infoGlobal);

	void setupMultiBodyContactConstraint(btlMultiBodySolverConstraint& solverConstraint, 
																 const btlVector3& contactNormal,
																 btlManifoldPoint& cp, const btlContactSolverInfo& infoGlobal,
																 btlScalar& relaxation,
																 bool isFriction, btlScalar desiredVelocity=0, btlScalar cfmSlip=0);

	void convertMultiBodyContact(btlPersistentManifold* manifold,const btlContactSolverInfo& infoGlobal);
	virtual btlScalar solveGroupCacheFriendlySetup(btlCollisionObject** bodies,int numBodies,btlPersistentManifold** manifoldPtr, int numManifolds,btlTypedConstraint** constraints,int numConstraints,const btlContactSolverInfo& infoGlobal,btlIDebugDraw* debugDrawer);
//	virtual btScalar solveGroupCacheFriendlyIterations(btCollisionObject** bodies,int numBodies,btPersistentManifold** manifoldPtr, int numManifolds,btTypedConstraint** constraints,int numConstraints,const btContactSolverInfo& infoGlobal,btIDebugDraw* debugDrawer);

	virtual btlScalar solveSingleIteration(int iteration, btlCollisionObject** bodies ,int numBodies,btlPersistentManifold** manifoldPtr, int numManifolds,btlTypedConstraint** constraints,int numConstraints,const btlContactSolverInfo& infoGlobal,btlIDebugDraw* debugDrawer);
	void	applyDeltaVee(btlScalar* deltaV, btlScalar impulse, int velocityIndex, int ndof);
	void writeBackSolverBodyToMultiBody(btlMultiBodySolverConstraint& constraint, btlScalar deltaTime);
public:

	BT_DECLARE_ALIGNED_ALLOCATOR();

	///this method should not be called, it was just used during porting/integration of Featherstone btMultiBody, providing backwards compatibility but no support for btMultiBodyConstraint (only contact constraints)
	virtual btlScalar solveGroup(btlCollisionObject** bodies,int numBodies,btlPersistentManifold** manifold,int numManifolds,btlTypedConstraint** constraints,int numConstraints,const btlContactSolverInfo& info, btlIDebugDraw* debugDrawer,btlDispatcher* dispatcher);
	virtual btlScalar solveGroupCacheFriendlyFinish(btlCollisionObject** bodies,int numBodies,const btlContactSolverInfo& infoGlobal);
	
	virtual void solveMultiBodyGroup(btlCollisionObject** bodies,int numBodies,btlPersistentManifold** manifold,int numManifolds,btlTypedConstraint** constraints,int numConstraints,btlMultiBodyConstraint** multiBodyConstraints, int numMultiBodyConstraints, const btlContactSolverInfo& info, btlIDebugDraw* debugDrawer,btlDispatcher* dispatcher);
};

	
	


#endif //BT_MULTIBODY_CONSTRAINT_SOLVER_H

