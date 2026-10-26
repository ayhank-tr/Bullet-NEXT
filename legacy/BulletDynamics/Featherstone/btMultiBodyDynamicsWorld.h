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

#ifndef BT_MULTIBODY_DYNAMICS_WORLD_H
#define BT_MULTIBODY_DYNAMICS_WORLD_H

#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"

#define BT_USE_VIRTUAL_CLEARFORCES_AND_GRAVITY

class btlMultiBody;
class btlMultiBodyConstraint;
class btlMultiBodyConstraintSolver;
struct btl_MultiBodyInplaceSolverIslandCallback;

///The btMultiBodyDynamicsWorld adds Featherstone multi body dynamics to Bullet
///This implementation is still preliminary/experimental.
class btlMultiBodyDynamicsWorld : public btlDiscreteDynamicsWorld
{
protected:
	btlAlignedObjectArray<btlMultiBody*> m_multiBodies;
	btlAlignedObjectArray<btlMultiBodyConstraint*> m_multiBodyConstraints;
	btlAlignedObjectArray<btlMultiBodyConstraint*> m_sortedMultiBodyConstraints;
	btlMultiBodyConstraintSolver*	m_multiBodyConstraintSolver;
	btl_MultiBodyInplaceSolverIslandCallback*	m_solverMultiBodyIslandCallback;

	virtual void	calculateSimulationIslands();
	virtual void	updateActivationState(btlScalar timeStep);
	virtual void	solveConstraints(btlContactSolverInfo& solverInfo);
	
	virtual void	serializeMultiBodies(btlSerializer* serializer);

public:

	btlMultiBodyDynamicsWorld(btlDispatcher* dispatcher,btlBroadphaseInterface* pairCache,btlMultiBodyConstraintSolver* constraintSolver,btlCollisionConfiguration* collisionConfiguration);

	virtual ~btlMultiBodyDynamicsWorld ();

	virtual void	addMultiBody(btlMultiBody* body, short group= btlBroadphaseProxy::DefaultFilter, short mask=btlBroadphaseProxy::AllFilter);

	virtual void	removeMultiBody(btlMultiBody* body);

	virtual int		getNumMultibodies() const
	{
		return m_multiBodies.size();
	}

	btlMultiBody*	getMultiBody(int mbIndex)
	{
		return m_multiBodies[mbIndex];
	}

	virtual void	addMultiBodyConstraint( btlMultiBodyConstraint* constraint);

	virtual int     getNumMultiBodyConstraints() const
	{
        return m_multiBodyConstraints.size();
	}

	virtual btlMultiBodyConstraint*	getMultiBodyConstraint( int constraintIndex)
	{
        return m_multiBodyConstraints[constraintIndex];
	}

	virtual const btlMultiBodyConstraint*	getMultiBodyConstraint( int constraintIndex) const
	{
        return m_multiBodyConstraints[constraintIndex];
	}

	virtual void	removeMultiBodyConstraint( btlMultiBodyConstraint* constraint);

	virtual void	integrateTransforms(btlScalar timeStep);

	virtual void	debugDrawWorld();

	virtual void	debugDrawMultiBodyConstraint(btlMultiBodyConstraint* constraint);
	
	void	forwardKinematics();
	virtual void clearForces();
	virtual void clearMultiBodyConstraintForces();
	virtual void clearMultiBodyForces();
	virtual void applyGravity();
	
	virtual	void	serialize(btlSerializer* serializer);

};
#endif //BT_MULTIBODY_DYNAMICS_WORLD_H
