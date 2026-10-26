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

#include "btSimpleDynamicsWorld.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btSimpleBroadphase.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "BulletDynamics/ConstraintSolver/btContactSolverInfo.h"


/*
  Make sure this dummy function never changes so that it
  can be used by probes that are checking whether the
  library is actually installed.
*/
extern "C" 
{
	void btlBulletDynamicsProbe ();
	void btlBulletDynamicsProbe () {}
}




btlSimpleDynamicsWorld::btlSimpleDynamicsWorld(btlDispatcher* dispatcher,btlBroadphaseInterface* pairCache,btlConstraintSolver* constraintSolver,btlCollisionConfiguration* collisionConfiguration)
:btlDynamicsWorld(dispatcher,pairCache,collisionConfiguration),
m_constraintSolver(constraintSolver),
m_ownsConstraintSolver(false),
m_gravity(0,0,-10)
{

}


btlSimpleDynamicsWorld::~btlSimpleDynamicsWorld()
{
	if (m_ownsConstraintSolver)
		btlAlignedFree( m_constraintSolver);
}

int		btlSimpleDynamicsWorld::stepSimulation( btlScalar timeStep,int maxSubSteps, btlScalar fixedTimeStep)
{
	(void)fixedTimeStep;
	(void)maxSubSteps;


	///apply gravity, predict motion
	predictUnconstraintMotion(timeStep);

	btlDispatcherInfo&	dispatchInfo = getDispatchInfo();
	dispatchInfo.m_timeStep = timeStep;
	dispatchInfo.m_stepCount = 0;
	dispatchInfo.m_debugDraw = getDebugDrawer();

	///perform collision detection
	performDiscreteCollisionDetection();

	///solve contact constraints
	int numManifolds = m_dispatcher1->getNumManifolds();
	if (numManifolds)
	{
		btlPersistentManifold** manifoldPtr = ((btlCollisionDispatcher*)m_dispatcher1)->getInternalManifoldPointer();
		
		btlContactSolverInfo infoGlobal;
		infoGlobal.m_timeStep = timeStep;
		m_constraintSolver->prepareSolve(0,numManifolds);
		m_constraintSolver->solveGroup(&getCollisionObjectArray()[0],getNumCollisionObjects(),manifoldPtr, numManifolds,0,0,infoGlobal,m_debugDrawer, m_dispatcher1);
		m_constraintSolver->allSolved(infoGlobal,m_debugDrawer);
	}

	///integrate transforms
	integrateTransforms(timeStep);
		
	updateAabbs();

	synchronizeMotionStates();

	clearForces();

	return 1;

}

void	btlSimpleDynamicsWorld::clearForces()
{
	///@todo: iterate over awake simulation islands!
	for ( int i=0;i<m_collisionObjects.size();i++)
	{
		btlCollisionObject* colObj = m_collisionObjects[i];
		
		btlRigidBody* body = btlRigidBody::upcast(colObj);
		if (body)
		{
			body->clearForces();
		}
	}
}	


void	btlSimpleDynamicsWorld::setGravity(const btlVector3& gravity)
{
	m_gravity = gravity;
	for ( int i=0;i<m_collisionObjects.size();i++)
	{
		btlCollisionObject* colObj = m_collisionObjects[i];
		btlRigidBody* body = btlRigidBody::upcast(colObj);
		if (body)
		{
			body->setGravity(gravity);
		}
	}
}

btlVector3 btlSimpleDynamicsWorld::getGravity () const
{
	return m_gravity;
}

void	btlSimpleDynamicsWorld::removeRigidBody(btlRigidBody* body)
{
	btlCollisionWorld::removeCollisionObject(body);
}

void	btlSimpleDynamicsWorld::removeCollisionObject(btlCollisionObject* collisionObject)
{
	btlRigidBody* body = btlRigidBody::upcast(collisionObject);
	if (body)
		removeRigidBody(body);
	else
		btlCollisionWorld::removeCollisionObject(collisionObject);
}


void	btlSimpleDynamicsWorld::addRigidBody(btlRigidBody* body)
{
	body->setGravity(m_gravity);

	if (body->getCollisionShape())
	{
		addCollisionObject(body);
	}
}

void	btlSimpleDynamicsWorld::addRigidBody(btlRigidBody* body, short group, short mask)
{
	body->setGravity(m_gravity);

	if (body->getCollisionShape())
	{
		addCollisionObject(body,group,mask);
	}
}


void	btlSimpleDynamicsWorld::debugDrawWorld()
{

}
				
void	btlSimpleDynamicsWorld::addAction(btlActionInterface* action)
{

}

void	btlSimpleDynamicsWorld::removeAction(btlActionInterface* action)
{

}


void	btlSimpleDynamicsWorld::updateAabbs()
{
	btlTransform predictedTrans;
	for ( int i=0;i<m_collisionObjects.size();i++)
	{
		btlCollisionObject* colObj = m_collisionObjects[i];
		btlRigidBody* body = btlRigidBody::upcast(colObj);
		if (body)
		{
			if (body->isActive() && (!body->isStaticObject()))
			{
				btlVector3 minAabb,maxAabb;
				colObj->getCollisionShape()->getAabb(colObj->getWorldTransform(), minAabb,maxAabb);
				btlBroadphaseInterface* bp = getBroadphase();
				bp->setAabb(body->getBroadphaseHandle(),minAabb,maxAabb, m_dispatcher1);
			}
		}
	}
}

void	btlSimpleDynamicsWorld::integrateTransforms(btlScalar timeStep)
{
	btlTransform predictedTrans;
	for ( int i=0;i<m_collisionObjects.size();i++)
	{
		btlCollisionObject* colObj = m_collisionObjects[i];
		btlRigidBody* body = btlRigidBody::upcast(colObj);
		if (body)
		{
			if (body->isActive() && (!body->isStaticObject()))
			{
				body->predictIntegratedTransform(timeStep, predictedTrans);
				body->proceedToTransform( predictedTrans);
			}
		}
	}
}



void	btlSimpleDynamicsWorld::predictUnconstraintMotion(btlScalar timeStep)
{
	for ( int i=0;i<m_collisionObjects.size();i++)
	{
		btlCollisionObject* colObj = m_collisionObjects[i];
		btlRigidBody* body = btlRigidBody::upcast(colObj);
		if (body)
		{
			if (!body->isStaticObject())
			{
				if (body->isActive())
				{
					body->applyGravity();
					body->integrateVelocities( timeStep);
					body->applyDamping(timeStep);
					body->predictIntegratedTransform(timeStep,body->getInterpolationWorldTransform());
				}
			}
		}
	}
}


void	btlSimpleDynamicsWorld::synchronizeMotionStates()
{
	///@todo: iterate over awake simulation islands!
	for ( int i=0;i<m_collisionObjects.size();i++)
	{
		btlCollisionObject* colObj = m_collisionObjects[i];
		btlRigidBody* body = btlRigidBody::upcast(colObj);
		if (body && body->getMotionState())
		{
			if (body->getActivationState() != ISLAND_SLEEPING)
			{
				body->getMotionState()->setWorldTransform(body->getWorldTransform());
			}
		}
	}

}


void	btlSimpleDynamicsWorld::setConstraintSolver(btlConstraintSolver* solver)
{
	if (m_ownsConstraintSolver)
	{
		btlAlignedFree(m_constraintSolver);
	}
	m_ownsConstraintSolver = false;
	m_constraintSolver = solver;
}

btlConstraintSolver* btlSimpleDynamicsWorld::getConstraintSolver()
{
	return m_constraintSolver;
}
