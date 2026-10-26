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

#ifndef BT_SIMPLE_DYNAMICS_WORLD_H
#define BT_SIMPLE_DYNAMICS_WORLD_H

#include "btDynamicsWorld.h"

class btlDispatcher;
class btlOverlappingPairCache;
class btlConstraintSolver;

///The btSimpleDynamicsWorld serves as unit-test and to verify more complicated and optimized dynamics worlds.
///Please use btDiscreteDynamicsWorld instead
class btlSimpleDynamicsWorld : public btlDynamicsWorld
{
protected:

	btlConstraintSolver*	m_constraintSolver;

	bool	m_ownsConstraintSolver;

	void	predictUnconstraintMotion(btlScalar timeStep);
	
	void	integrateTransforms(btlScalar timeStep);
		
	btlVector3	m_gravity;
	
public:



	///this btSimpleDynamicsWorld constructor creates dispatcher, broadphase pairCache and constraintSolver
	btlSimpleDynamicsWorld(btlDispatcher* dispatcher,btlBroadphaseInterface* pairCache,btlConstraintSolver* constraintSolver,btlCollisionConfiguration* collisionConfiguration);

	virtual ~btlSimpleDynamicsWorld();
		
	///maxSubSteps/fixedTimeStep for interpolation is currently ignored for btSimpleDynamicsWorld, use btDiscreteDynamicsWorld instead
	virtual int	stepSimulation( btlScalar timeStep,int maxSubSteps=1, btlScalar fixedTimeStep=btlScalar(1.)/btlScalar(60.));

	virtual void	setGravity(const btlVector3& gravity);

	virtual btlVector3 getGravity () const;

	virtual void	addRigidBody(btlRigidBody* body);

	virtual void	addRigidBody(btlRigidBody* body, short group, short mask);

	virtual void	removeRigidBody(btlRigidBody* body);

	virtual void	debugDrawWorld();
				
	virtual void	addAction(btlActionInterface* action);

	virtual void	removeAction(btlActionInterface* action);

	///removeCollisionObject will first check if it is a rigid body, if so call removeRigidBody otherwise call btCollisionWorld::removeCollisionObject
	virtual void	removeCollisionObject(btlCollisionObject* collisionObject);
	
	virtual void	updateAabbs();

	virtual void	synchronizeMotionStates();

	virtual void	setConstraintSolver(btlConstraintSolver* solver);

	virtual btlConstraintSolver* getConstraintSolver();

	virtual btlDynamicsWorldType	getWorldType() const
	{
		return BT_SIMPLE_DYNAMICS_WORLD;
	}

	virtual void	clearForces();

};

#endif //BT_SIMPLE_DYNAMICS_WORLD_H
