/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#include "btDiscreteDynamicsWorld.h"

//collision detection
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/BroadphaseCollision/btSimpleBroadphase.h"
#include "BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
#include "BulletCollision/CollisionDispatch/btSimulationIslandManager.h"
#include "LinearMath/btTransformUtil.h"
#include "LinearMath/btQuickprof.h"

//rigidbody & constraints
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "BulletDynamics/ConstraintSolver/btContactSolverInfo.h"
#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"
#include "BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h"
#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h"
#include "BulletDynamics/ConstraintSolver/btConeTwistConstraint.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h"
#include "BulletDynamics/ConstraintSolver/btSliderConstraint.h"
#include "BulletDynamics/ConstraintSolver/btContactConstraint.h"


#include "LinearMath/btIDebugDraw.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"


#include "BulletDynamics/Dynamics/btActionInterface.h"
#include "LinearMath/btQuickprof.h"
#include "LinearMath/btMotionState.h"

#include "LinearMath/btSerializer.h"

#if 0
btlAlignedObjectArray<btlVector3> debugContacts;
btlAlignedObjectArray<btlVector3> debugNormals;
int startHit=2;
int firstHit=startHit;
#endif

SIMD_FORCE_INLINE	int	btlGetConstraintIslandId(const btlTypedConstraint* lhs)
{
	int islandId;

	const btlCollisionObject& rcolObj0 = lhs->getRigidBodyA();
	const btlCollisionObject& rcolObj1 = lhs->getRigidBodyB();
	islandId= rcolObj0.getIslandTag()>=0?rcolObj0.getIslandTag():rcolObj1.getIslandTag();
	return islandId;

}


class btlSortConstraintOnIslandPredicate
{
	public:

		bool operator() ( const btlTypedConstraint* lhs, const btlTypedConstraint* rhs ) const
		{
			int rIslandId0,lIslandId0;
			rIslandId0 = btlGetConstraintIslandId(rhs);
			lIslandId0 = btlGetConstraintIslandId(lhs);
			return lIslandId0 < rIslandId0;
		}
};

struct btl_InplaceSolverIslandCallback : public btlSimulationIslandManager::IslandCallback
{
	btlContactSolverInfo*	m_solverInfo;
	btlConstraintSolver*		m_solver;
	btlTypedConstraint**		m_sortedConstraints;
	int						m_numConstraints;
	btlIDebugDraw*			m_debugDrawer;
	btlDispatcher*			m_dispatcher;

	btlAlignedObjectArray<btlCollisionObject*> m_bodies;
	btlAlignedObjectArray<btlPersistentManifold*> m_manifolds;
	btlAlignedObjectArray<btlTypedConstraint*> m_constraints;


	btl_InplaceSolverIslandCallback(
		btlConstraintSolver*	solver,
		btlStackAlloc* stackAlloc,
		btlDispatcher* dispatcher)
		:m_solverInfo(NULL),
		m_solver(solver),
		m_sortedConstraints(NULL),
		m_numConstraints(0),
		m_debugDrawer(NULL),
		m_dispatcher(dispatcher)
	{

	}

	btl_InplaceSolverIslandCallback& operator=(btl_InplaceSolverIslandCallback& other)
	{
		btlAssert(0);
		(void)other;
		return *this;
	}

	SIMD_FORCE_INLINE void setup ( btlContactSolverInfo* solverInfo, btlTypedConstraint** sortedConstraints,	int	numConstraints,	btlIDebugDraw* debugDrawer)
	{
		btlAssert(solverInfo);
		m_solverInfo = solverInfo;
		m_sortedConstraints = sortedConstraints;
		m_numConstraints = numConstraints;
		m_debugDrawer = debugDrawer;
		m_bodies.resize (0);
		m_manifolds.resize (0);
		m_constraints.resize (0);
	}


	virtual	void	processIsland(btlCollisionObject** bodies,int numBodies,btlPersistentManifold**	manifolds,int numManifolds, int islandId)
	{
		if (islandId<0)
		{
			///we don't split islands, so all constraints/contact manifolds/bodies are passed into the solver regardless the island id
			m_solver->solveGroup( bodies,numBodies,manifolds, numManifolds,&m_sortedConstraints[0],m_numConstraints,*m_solverInfo,m_debugDrawer,m_dispatcher);
		} else
		{
				//also add all non-contact constraints/joints for this island
			btlTypedConstraint** startConstraint = 0;
			int numCurConstraints = 0;
			int i;

			//find the first constraint for this island
			for (i=0;i<m_numConstraints;i++)
			{
				if (btlGetConstraintIslandId(m_sortedConstraints[i]) == islandId)
				{
					startConstraint = &m_sortedConstraints[i];
					break;
				}
			}
			//count the number of constraints in this island
			for (;i<m_numConstraints;i++)
			{
				if (btlGetConstraintIslandId(m_sortedConstraints[i]) == islandId)
				{
					numCurConstraints++;
				}
			}

			if (m_solverInfo->m_minimumSolverBatchSize<=1)
			{
				m_solver->solveGroup( bodies,numBodies,manifolds, numManifolds,startConstraint,numCurConstraints,*m_solverInfo,m_debugDrawer,m_dispatcher);
			} else
			{

				for (i=0;i<numBodies;i++)
					m_bodies.push_back(bodies[i]);
				for (i=0;i<numManifolds;i++)
					m_manifolds.push_back(manifolds[i]);
				for (i=0;i<numCurConstraints;i++)
					m_constraints.push_back(startConstraint[i]);
				if ((m_constraints.size()+m_manifolds.size())>m_solverInfo->m_minimumSolverBatchSize)
				{
					processConstraints();
				} else
				{
					//printf("deferred\n");
				}
			}
		}
	}
	void	processConstraints()
	{

		btlCollisionObject** bodies = m_bodies.size()? &m_bodies[0]:0;
		btlPersistentManifold** manifold = m_manifolds.size()?&m_manifolds[0]:0;
		btlTypedConstraint** constraints = m_constraints.size()?&m_constraints[0]:0;

		m_solver->solveGroup( bodies,m_bodies.size(),manifold, m_manifolds.size(),constraints, m_constraints.size() ,*m_solverInfo,m_debugDrawer,m_dispatcher);
		m_bodies.resize(0);
		m_manifolds.resize(0);
		m_constraints.resize(0);

	}

};



btlDiscreteDynamicsWorld::btlDiscreteDynamicsWorld(btlDispatcher* dispatcher,btlBroadphaseInterface* pairCache,btlConstraintSolver* constraintSolver, btlCollisionConfiguration* collisionConfiguration)
:btlDynamicsWorld(dispatcher,pairCache,collisionConfiguration),
m_sortedConstraints	(),
m_solverIslandCallback ( NULL ),
m_constraintSolver(constraintSolver),
m_gravity(0,-10,0),
m_localTime(0),
m_fixedTimeStep(0),
m_synchronizeAllMotionStates(false),
m_applySpeculativeContactRestitution(false),
m_profileTimings(0),
m_latencyMotionStateInterpolation(true)

{
	if (!m_constraintSolver)
	{
		void* mem = btlAlignedAlloc(sizeof(btlSequentialImpulseConstraintSolver),16);
		m_constraintSolver = new (mem) btlSequentialImpulseConstraintSolver;
		m_ownsConstraintSolver = true;
	} else
	{
		m_ownsConstraintSolver = false;
	}

	{
		void* mem = btlAlignedAlloc(sizeof(btlSimulationIslandManager),16);
		m_islandManager = new (mem) btlSimulationIslandManager();
	}

	m_ownsIslandManager = true;

	{
		void* mem = btlAlignedAlloc(sizeof(btl_InplaceSolverIslandCallback),16);
		m_solverIslandCallback = new (mem) btl_InplaceSolverIslandCallback (m_constraintSolver, 0, dispatcher);
	}
}


btlDiscreteDynamicsWorld::~btlDiscreteDynamicsWorld()
{
	//only delete it when we created it
	if (m_ownsIslandManager)
	{
		m_islandManager->~btlSimulationIslandManager();
		btlAlignedFree( m_islandManager);
	}
	if (m_solverIslandCallback)
	{
		m_solverIslandCallback->~btl_InplaceSolverIslandCallback();
		btlAlignedFree(m_solverIslandCallback);
	}
	if (m_ownsConstraintSolver)
	{

		m_constraintSolver->~btlConstraintSolver();
		btlAlignedFree(m_constraintSolver);
	}
}

void	btlDiscreteDynamicsWorld::saveKinematicState(btlScalar timeStep)
{
///would like to iterate over m_nonStaticRigidBodies, but unfortunately old API allows
///to switch status _after_ adding kinematic objects to the world
///fix it for Bullet 3.x release
	for (int i=0;i<m_collisionObjects.size();i++)
	{
		btlCollisionObject* colObj = m_collisionObjects[i];
		btlRigidBody* body = btlRigidBody::upcast(colObj);
		if (body && body->getActivationState() != ISLAND_SLEEPING)
		{
			if (body->isKinematicObject())
			{
				//to calculate velocities next frame
				body->saveKinematicState(timeStep);
			}
		}
	}

}

void	btlDiscreteDynamicsWorld::debugDrawWorld()
{
	BT_PROFILE("debugDrawWorld");

	btlCollisionWorld::debugDrawWorld();

	bool drawConstraints = false;
	if (getDebugDrawer())
	{
		int mode = getDebugDrawer()->getDebugMode();
		if(mode  & (btlIDebugDraw::DBG_DrawConstraints | btlIDebugDraw::DBG_DrawConstraintLimits))
		{
			drawConstraints = true;
		}
	}
	if(drawConstraints)
	{
		for(int i = getNumConstraints()-1; i>=0 ;i--)
		{
			btlTypedConstraint* constraint = getConstraint(i);
			debugDrawConstraint(constraint);
		}
	}



    if (getDebugDrawer() && (getDebugDrawer()->getDebugMode() & (btlIDebugDraw::DBG_DrawWireframe | btlIDebugDraw::DBG_DrawAabb | btlIDebugDraw::DBG_DrawNormals)))
	{
		int i;

		if (getDebugDrawer() && getDebugDrawer()->getDebugMode())
		{
			for (i=0;i<m_actions.size();i++)
			{
				m_actions[i]->debugDraw(m_debugDrawer);
			}
		}
	}
    if (getDebugDrawer())
        getDebugDrawer()->flushLines();

}

void	btlDiscreteDynamicsWorld::clearForces()
{
	///@todo: iterate over awake simulation islands!
	for ( int i=0;i<m_nonStaticRigidBodies.size();i++)
	{
		btlRigidBody* body = m_nonStaticRigidBodies[i];
		//need to check if next line is ok
		//it might break backward compatibility (people applying forces on sleeping objects get never cleared and accumulate on wake-up
		body->clearForces();
	}
}

///apply gravity, call this once per timestep
void	btlDiscreteDynamicsWorld::applyGravity()
{
	///@todo: iterate over awake simulation islands!
	for ( int i=0;i<m_nonStaticRigidBodies.size();i++)
	{
		btlRigidBody* body = m_nonStaticRigidBodies[i];
		if (body->isActive())
		{
			body->applyGravity();
		}
	}
}


void	btlDiscreteDynamicsWorld::synchronizeSingleMotionState(btlRigidBody* body)
{
	btlAssert(body);

	if (body->getMotionState() && !body->isStaticOrKinematicObject())
	{
		//we need to call the update at least once, even for sleeping objects
		//otherwise the 'graphics' transform never updates properly
		///@todo: add 'dirty' flag
		//if (body->getActivationState() != ISLAND_SLEEPING)
		{
			btlTransform interpolatedTransform;
			btlTransformUtil::integrateTransform(body->getInterpolationWorldTransform(),
				body->getInterpolationLinearVelocity(),body->getInterpolationAngularVelocity(),
				(m_latencyMotionStateInterpolation && m_fixedTimeStep) ? m_localTime - m_fixedTimeStep : m_localTime*body->getHitFraction(),
				interpolatedTransform);
			body->getMotionState()->setWorldTransform(interpolatedTransform);
		}
	}
}


void	btlDiscreteDynamicsWorld::synchronizeMotionStates()
{
	BT_PROFILE("synchronizeMotionStates");
	if (m_synchronizeAllMotionStates)
	{
		//iterate  over all collision objects
		for ( int i=0;i<m_collisionObjects.size();i++)
		{
			btlCollisionObject* colObj = m_collisionObjects[i];
			btlRigidBody* body = btlRigidBody::upcast(colObj);
			if (body)
				synchronizeSingleMotionState(body);
		}
	} else
	{
		//iterate over all active rigid bodies
		for ( int i=0;i<m_nonStaticRigidBodies.size();i++)
		{
			btlRigidBody* body = m_nonStaticRigidBodies[i];
			if (body->isActive())
				synchronizeSingleMotionState(body);
		}
	}
}


int	btlDiscreteDynamicsWorld::stepSimulation( btlScalar timeStep,int maxSubSteps, btlScalar fixedTimeStep)
{
	startProfiling(timeStep);

	BT_PROFILE("stepSimulation");

	int numSimulationSubSteps = 0;

	if (maxSubSteps)
	{
		//fixed timestep with interpolation
		m_fixedTimeStep = fixedTimeStep;
		m_localTime += timeStep;
		if (m_localTime >= fixedTimeStep)
		{
			numSimulationSubSteps = int( m_localTime / fixedTimeStep);
			m_localTime -= numSimulationSubSteps * fixedTimeStep;
		}
	} else
	{
		//variable timestep
		fixedTimeStep = timeStep;
		m_localTime = m_latencyMotionStateInterpolation ? 0 : timeStep;
		m_fixedTimeStep = 0;
		if (btlFuzzyZero(timeStep))
		{
			numSimulationSubSteps = 0;
			maxSubSteps = 0;
		} else
		{
			numSimulationSubSteps = 1;
			maxSubSteps = 1;
		}
	}

	//process some debugging flags
	if (getDebugDrawer())
	{
		btlIDebugDraw* debugDrawer = getDebugDrawer ();
		btl_gDisableDeactivation = (debugDrawer->getDebugMode() & btlIDebugDraw::DBG_NoDeactivation) != 0;
	}
	if (numSimulationSubSteps)
	{

		//clamp the number of substeps, to prevent simulation grinding spiralling down to a halt
		int clampedSimulationSteps = (numSimulationSubSteps > maxSubSteps)? maxSubSteps : numSimulationSubSteps;

		saveKinematicState(fixedTimeStep*clampedSimulationSteps);

		applyGravity();



		for (int i=0;i<clampedSimulationSteps;i++)
		{
			internalSingleStepSimulation(fixedTimeStep);
			synchronizeMotionStates();
		}

	} else
	{
		synchronizeMotionStates();
	}

	clearForces();

#ifndef BT_NO_PROFILE
	btl_CProfileManager::Increment_Frame_Counter();
#endif //BT_NO_PROFILE

	return numSimulationSubSteps;
}

void	btlDiscreteDynamicsWorld::internalSingleStepSimulation(btlScalar timeStep)
{

	BT_PROFILE("internalSingleStepSimulation");

	if(0 != m_internalPreTickCallback) {
		(*m_internalPreTickCallback)(this, timeStep);
	}

	///apply gravity, predict motion
	predictUnconstraintMotion(timeStep);

	btlDispatcherInfo& dispatchInfo = getDispatchInfo();

	dispatchInfo.m_timeStep = timeStep;
	dispatchInfo.m_stepCount = 0;
	dispatchInfo.m_debugDraw = getDebugDrawer();


    createPredictiveContacts(timeStep);

	///perform collision detection
	performDiscreteCollisionDetection();

	calculateSimulationIslands();


	getSolverInfo().m_timeStep = timeStep;



	///solve contact and other joint constraints
	solveConstraints(getSolverInfo());

	///CallbackTriggers();

	///integrate transforms

	integrateTransforms(timeStep);

	///update vehicle simulation
	updateActions(timeStep);

	updateActivationState( timeStep );

	if(0 != m_internalTickCallback) {
		(*m_internalTickCallback)(this, timeStep);
	}
}

void	btlDiscreteDynamicsWorld::setGravity(const btlVector3& gravity)
{
	m_gravity = gravity;
	for ( int i=0;i<m_nonStaticRigidBodies.size();i++)
	{
		btlRigidBody* body = m_nonStaticRigidBodies[i];
		if (body->isActive() && !(body->getFlags() &BT_DISABLE_WORLD_GRAVITY))
		{
			body->setGravity(gravity);
		}
	}
}

btlVector3 btlDiscreteDynamicsWorld::getGravity () const
{
	return m_gravity;
}

void	btlDiscreteDynamicsWorld::addCollisionObject(btlCollisionObject* collisionObject,short int collisionFilterGroup,short int collisionFilterMask)
{
	btlCollisionWorld::addCollisionObject(collisionObject,collisionFilterGroup,collisionFilterMask);
}

void	btlDiscreteDynamicsWorld::removeCollisionObject(btlCollisionObject* collisionObject)
{
	btlRigidBody* body = btlRigidBody::upcast(collisionObject);
	if (body)
		removeRigidBody(body);
	else
		btlCollisionWorld::removeCollisionObject(collisionObject);
}

void	btlDiscreteDynamicsWorld::removeRigidBody(btlRigidBody* body)
{
	m_nonStaticRigidBodies.remove(body);
	btlCollisionWorld::removeCollisionObject(body);
}


void	btlDiscreteDynamicsWorld::addRigidBody(btlRigidBody* body)
{
	if (!body->isStaticOrKinematicObject() && !(body->getFlags() &BT_DISABLE_WORLD_GRAVITY))
	{
		body->setGravity(m_gravity);
	}

	if (body->getCollisionShape())
	{
		if (!body->isStaticObject())
		{
			m_nonStaticRigidBodies.push_back(body);
		} else
		{
			body->setActivationState(ISLAND_SLEEPING);
		}

		bool isDynamic = !(body->isStaticObject() || body->isKinematicObject());
		short collisionFilterGroup = isDynamic? short(btlBroadphaseProxy::DefaultFilter) : short(btlBroadphaseProxy::StaticFilter);
		short collisionFilterMask = isDynamic? 	short(btlBroadphaseProxy::AllFilter) : 	short(btlBroadphaseProxy::AllFilter ^ btlBroadphaseProxy::StaticFilter);

		addCollisionObject(body,collisionFilterGroup,collisionFilterMask);
	}
}

void	btlDiscreteDynamicsWorld::addRigidBody(btlRigidBody* body, short group, short mask)
{
	if (!body->isStaticOrKinematicObject() && !(body->getFlags() &BT_DISABLE_WORLD_GRAVITY))
	{
		body->setGravity(m_gravity);
	}

	if (body->getCollisionShape())
	{
		if (!body->isStaticObject())
		{
			m_nonStaticRigidBodies.push_back(body);
		}
		 else
		{
			body->setActivationState(ISLAND_SLEEPING);
		}
		addCollisionObject(body,group,mask);
	}
}


void	btlDiscreteDynamicsWorld::updateActions(btlScalar timeStep)
{
	BT_PROFILE("updateActions");

	for ( int i=0;i<m_actions.size();i++)
	{
		m_actions[i]->updateAction( this, timeStep);
	}
}


void	btlDiscreteDynamicsWorld::updateActivationState(btlScalar timeStep)
{
	BT_PROFILE("updateActivationState");

	for ( int i=0;i<m_nonStaticRigidBodies.size();i++)
	{
		btlRigidBody* body = m_nonStaticRigidBodies[i];
		if (body)
		{
			body->updateDeactivation(timeStep);

			if (body->wantsSleeping())
			{
				if (body->isStaticOrKinematicObject())
				{
					body->setActivationState(ISLAND_SLEEPING);
				} else
				{
					if (body->getActivationState() == ACTIVE_TAG)
						body->setActivationState( WANTS_DEACTIVATION );
					if (body->getActivationState() == ISLAND_SLEEPING)
					{
						body->setAngularVelocity(btlVector3(0,0,0));
						body->setLinearVelocity(btlVector3(0,0,0));
					}

				}
			} else
			{
				if (body->getActivationState() != DISABLE_DEACTIVATION)
					body->setActivationState( ACTIVE_TAG );
			}
		}
	}
}

void	btlDiscreteDynamicsWorld::addConstraint(btlTypedConstraint* constraint,bool disableCollisionsBetweenLinkedBodies)
{
	m_constraints.push_back(constraint);
    //Make sure the two bodies of a type constraint are different (possibly add this to the btTypedConstraint constructor?)
    btlAssert(&constraint->getRigidBodyA()!=&constraint->getRigidBodyB());
    
	if (disableCollisionsBetweenLinkedBodies)
	{
		constraint->getRigidBodyA().addConstraintRef(constraint);
		constraint->getRigidBodyB().addConstraintRef(constraint);
	}
}

void	btlDiscreteDynamicsWorld::removeConstraint(btlTypedConstraint* constraint)
{
	m_constraints.remove(constraint);
	constraint->getRigidBodyA().removeConstraintRef(constraint);
	constraint->getRigidBodyB().removeConstraintRef(constraint);
}

void	btlDiscreteDynamicsWorld::addAction(btlActionInterface* action)
{
	m_actions.push_back(action);
}

void	btlDiscreteDynamicsWorld::removeAction(btlActionInterface* action)
{
	m_actions.remove(action);
}


void	btlDiscreteDynamicsWorld::addVehicle(btlActionInterface* vehicle)
{
	addAction(vehicle);
}

void	btlDiscreteDynamicsWorld::removeVehicle(btlActionInterface* vehicle)
{
	removeAction(vehicle);
}

void	btlDiscreteDynamicsWorld::addCharacter(btlActionInterface* character)
{
	addAction(character);
}

void	btlDiscreteDynamicsWorld::removeCharacter(btlActionInterface* character)
{
	removeAction(character);
}




void	btlDiscreteDynamicsWorld::solveConstraints(btlContactSolverInfo& solverInfo)
{
	BT_PROFILE("solveConstraints");

	m_sortedConstraints.resize( m_constraints.size());
	int i;
	for (i=0;i<getNumConstraints();i++)
	{
		m_sortedConstraints[i] = m_constraints[i];
	}

//	btAssert(0);



	m_sortedConstraints.quickSort(btlSortConstraintOnIslandPredicate());

	btlTypedConstraint** constraintsPtr = getNumConstraints() ? &m_sortedConstraints[0] : 0;

	m_solverIslandCallback->setup(&solverInfo,constraintsPtr,m_sortedConstraints.size(),getDebugDrawer());
	m_constraintSolver->prepareSolve(getCollisionWorld()->getNumCollisionObjects(), getCollisionWorld()->getDispatcher()->getNumManifolds());

	/// solve all the constraints for this island
	m_islandManager->buildAndProcessIslands(getCollisionWorld()->getDispatcher(),getCollisionWorld(),m_solverIslandCallback);

	m_solverIslandCallback->processConstraints();

	m_constraintSolver->allSolved(solverInfo, m_debugDrawer);
}


void	btlDiscreteDynamicsWorld::calculateSimulationIslands()
{
	BT_PROFILE("calculateSimulationIslands");

	getSimulationIslandManager()->updateActivationState(getCollisionWorld(),getCollisionWorld()->getDispatcher());

    {
        //merge islands based on speculative contact manifolds too
        for (int i=0;i<this->m_predictiveManifolds.size();i++)
        {
            btlPersistentManifold* manifold = m_predictiveManifolds[i];

            const btlCollisionObject* colObj0 = manifold->getBody0();
            const btlCollisionObject* colObj1 = manifold->getBody1();

            if (((colObj0) && (!(colObj0)->isStaticOrKinematicObject())) &&
                ((colObj1) && (!(colObj1)->isStaticOrKinematicObject())))
            {
				getSimulationIslandManager()->getUnionFind().unite((colObj0)->getIslandTag(),(colObj1)->getIslandTag());
            }
        }
    }

	{
		int i;
		int numConstraints = int(m_constraints.size());
		for (i=0;i< numConstraints ; i++ )
		{
			btlTypedConstraint* constraint = m_constraints[i];
			if (constraint->isEnabled())
			{
				const btlRigidBody* colObj0 = &constraint->getRigidBodyA();
				const btlRigidBody* colObj1 = &constraint->getRigidBodyB();

				if (((colObj0) && (!(colObj0)->isStaticOrKinematicObject())) &&
					((colObj1) && (!(colObj1)->isStaticOrKinematicObject())))
				{
					getSimulationIslandManager()->getUnionFind().unite((colObj0)->getIslandTag(),(colObj1)->getIslandTag());
				}
			}
		}
	}

	//Store the island id in each body
	getSimulationIslandManager()->storeIslandActivationState(getCollisionWorld());


}




class btlClosestNotMeConvexResultCallback : public btlCollisionWorld::ClosestConvexResultCallback
{
public:

	btlCollisionObject* m_me;
	btlScalar m_allowedPenetration;
	btlOverlappingPairCache* m_pairCache;
	btlDispatcher* m_dispatcher;

public:
	btlClosestNotMeConvexResultCallback (btlCollisionObject* me,const btlVector3& fromA,const btlVector3& toA,btlOverlappingPairCache* pairCache,btlDispatcher* dispatcher) :
	  btlCollisionWorld::ClosestConvexResultCallback(fromA,toA),
		m_me(me),
		m_allowedPenetration(0.0f),
		m_pairCache(pairCache),
		m_dispatcher(dispatcher)
	{
	}

	virtual btlScalar addSingleResult(btlCollisionWorld::LocalConvexResult& convexResult,bool normalInWorldSpace)
	{
		if (convexResult.m_hitCollisionObject == m_me)
			return 1.0f;

		//ignore result if there is no contact response
		if(!convexResult.m_hitCollisionObject->hasContactResponse())
			return 1.0f;

		btlVector3 linVelA,linVelB;
		linVelA = m_convexToWorld-m_convexFromWorld;
		linVelB = btlVector3(0,0,0);//toB.getOrigin()-fromB.getOrigin();

		btlVector3 relativeVelocity = (linVelA-linVelB);
		//don't report time of impact for motion away from the contact normal (or causes minor penetration)
		if (convexResult.m_hitNormalLocal.dot(relativeVelocity)>=-m_allowedPenetration)
			return 1.f;

		return ClosestConvexResultCallback::addSingleResult (convexResult, normalInWorldSpace);
	}

	virtual bool needsCollision(btlBroadphaseProxy* proxy0) const
	{
		//don't collide with itself
		if (proxy0->m_clientObject == m_me)
			return false;

		///don't do CCD when the collision filters are not matching
		if (!ClosestConvexResultCallback::needsCollision(proxy0))
			return false;

		btlCollisionObject* otherObj = (btlCollisionObject*) proxy0->m_clientObject;

		//call needsResponse, see http://code.google.com/p/bullet/issues/detail?id=179
		if (m_dispatcher->needsResponse(m_me,otherObj))
		{
#if 0
			///don't do CCD when there are already contact points (touching contact/penetration)
			btlAlignedObjectArray<btlPersistentManifold*> manifoldArray;
			btlBroadphasePair* collisionPair = m_pairCache->findPair(m_me->getBroadphaseHandle(),proxy0);
			if (collisionPair)
			{
				if (collisionPair->m_algorithm)
				{
					manifoldArray.resize(0);
					collisionPair->m_algorithm->getAllContactManifolds(manifoldArray);
					for (int j=0;j<manifoldArray.size();j++)
					{
						btlPersistentManifold* manifold = manifoldArray[j];
						if (manifold->getNumContacts()>0)
							return false;
					}
				}
			}
#endif
			return true;
		}

		return false;
	}


};

///internal debugging variable. this value shouldn't be too high
int btl_gNumClampedCcdMotions=0;


void	btlDiscreteDynamicsWorld::createPredictiveContacts(btlScalar timeStep)
{
	BT_PROFILE("createPredictiveContacts");

	{
		BT_PROFILE("release predictive contact manifolds");

		for (int i=0;i<m_predictiveManifolds.size();i++)
		{
			btlPersistentManifold* manifold = m_predictiveManifolds[i];
			this->m_dispatcher1->releaseManifold(manifold);
		}
		m_predictiveManifolds.clear();
	}

	btlTransform predictedTrans;
	for ( int i=0;i<m_nonStaticRigidBodies.size();i++)
	{
		btlRigidBody* body = m_nonStaticRigidBodies[i];
		body->setHitFraction(1.f);

		if (body->isActive() && (!body->isStaticOrKinematicObject()))
		{

			body->predictIntegratedTransform(timeStep, predictedTrans);

			btlScalar squareMotion = (predictedTrans.getOrigin()-body->getWorldTransform().getOrigin()).length2();

			if (getDispatchInfo().m_useContinuous && body->getCcdSquareMotionThreshold() && body->getCcdSquareMotionThreshold() < squareMotion)
			{
				BT_PROFILE("predictive convexSweepTest");
				if (body->getCollisionShape()->isConvex())
				{
					btl_gNumClampedCcdMotions++;
#ifdef PREDICTIVE_CONTACT_USE_STATIC_ONLY
					class StaticOnlyCallback : public btlClosestNotMeConvexResultCallback
					{
					public:

						StaticOnlyCallback (btlCollisionObject* me,const btlVector3& fromA,const btlVector3& toA,btlOverlappingPairCache* pairCache,btlDispatcher* dispatcher) :
						  btlClosestNotMeConvexResultCallback(me,fromA,toA,pairCache,dispatcher)
						{
						}

					  	virtual bool needsCollision(btlBroadphaseProxy* proxy0) const
						{
							btlCollisionObject* otherObj = (btlCollisionObject*) proxy0->m_clientObject;
							if (!otherObj->isStaticOrKinematicObject())
								return false;
							return btlClosestNotMeConvexResultCallback::needsCollision(proxy0);
						}
					};

					StaticOnlyCallback sweepResults(body,body->getWorldTransform().getOrigin(),predictedTrans.getOrigin(),getBroadphase()->getOverlappingPairCache(),getDispatcher());
#else
					btlClosestNotMeConvexResultCallback sweepResults(body,body->getWorldTransform().getOrigin(),predictedTrans.getOrigin(),getBroadphase()->getOverlappingPairCache(),getDispatcher());
#endif
					//btConvexShape* convexShape = static_cast<btConvexShape*>(body->getCollisionShape());
					btlSphereShape tmpSphere(body->getCcdSweptSphereRadius());//btConvexShape* convexShape = static_cast<btConvexShape*>(body->getCollisionShape());
					sweepResults.m_allowedPenetration=getDispatchInfo().m_allowedCcdPenetration;

					sweepResults.m_collisionFilterGroup = body->getBroadphaseProxy()->m_collisionFilterGroup;
					sweepResults.m_collisionFilterMask  = body->getBroadphaseProxy()->m_collisionFilterMask;
					btlTransform modifiedPredictedTrans = predictedTrans;
					modifiedPredictedTrans.setBasis(body->getWorldTransform().getBasis());

					convexSweepTest(&tmpSphere,body->getWorldTransform(),modifiedPredictedTrans,sweepResults);
					if (sweepResults.hasHit() && (sweepResults.m_closestHitFraction < 1.f))
					{

						btlVector3 distVec = (predictedTrans.getOrigin()-body->getWorldTransform().getOrigin())*sweepResults.m_closestHitFraction;
						btlScalar distance = distVec.dot(-sweepResults.m_hitNormalWorld);


						btlPersistentManifold* manifold = m_dispatcher1->getNewManifold(body,sweepResults.m_hitCollisionObject);
						m_predictiveManifolds.push_back(manifold);

						btlVector3 worldPointB = body->getWorldTransform().getOrigin()+distVec;
						btlVector3 localPointB = sweepResults.m_hitCollisionObject->getWorldTransform().inverse()*worldPointB;

						btlManifoldPoint newPoint(btlVector3(0,0,0), localPointB,sweepResults.m_hitNormalWorld,distance);

						bool isPredictive = true;
						int index = manifold->addManifoldPoint(newPoint, isPredictive);
						btlManifoldPoint& pt = manifold->getContactPoint(index);
						pt.m_combinedRestitution = 0;
						pt.m_combinedFriction = btlManifoldResult::calculateCombinedFriction(body,sweepResults.m_hitCollisionObject);
						pt.m_positionWorldOnA = body->getWorldTransform().getOrigin();
						pt.m_positionWorldOnB = worldPointB;

					}
				}
			}
		}
	}
}
void	btlDiscreteDynamicsWorld::integrateTransforms(btlScalar timeStep)
{
	BT_PROFILE("integrateTransforms");
	btlTransform predictedTrans;
	for ( int i=0;i<m_nonStaticRigidBodies.size();i++)
	{
		btlRigidBody* body = m_nonStaticRigidBodies[i];
		body->setHitFraction(1.f);

		if (body->isActive() && (!body->isStaticOrKinematicObject()))
		{

			body->predictIntegratedTransform(timeStep, predictedTrans);

			btlScalar squareMotion = (predictedTrans.getOrigin()-body->getWorldTransform().getOrigin()).length2();



			if (getDispatchInfo().m_useContinuous && body->getCcdSquareMotionThreshold() && body->getCcdSquareMotionThreshold() < squareMotion)
			{
				BT_PROFILE("CCD motion clamping");
				if (body->getCollisionShape()->isConvex())
				{
					btl_gNumClampedCcdMotions++;
#ifdef USE_STATIC_ONLY
					class StaticOnlyCallback : public btlClosestNotMeConvexResultCallback
					{
					public:

						StaticOnlyCallback (btlCollisionObject* me,const btlVector3& fromA,const btlVector3& toA,btlOverlappingPairCache* pairCache,btlDispatcher* dispatcher) :
						  btlClosestNotMeConvexResultCallback(me,fromA,toA,pairCache,dispatcher)
						{
						}

					  	virtual bool needsCollision(btlBroadphaseProxy* proxy0) const
						{
							btlCollisionObject* otherObj = (btlCollisionObject*) proxy0->m_clientObject;
							if (!otherObj->isStaticOrKinematicObject())
								return false;
							return btlClosestNotMeConvexResultCallback::needsCollision(proxy0);
						}
					};

					StaticOnlyCallback sweepResults(body,body->getWorldTransform().getOrigin(),predictedTrans.getOrigin(),getBroadphase()->getOverlappingPairCache(),getDispatcher());
#else
					btlClosestNotMeConvexResultCallback sweepResults(body,body->getWorldTransform().getOrigin(),predictedTrans.getOrigin(),getBroadphase()->getOverlappingPairCache(),getDispatcher());
#endif
					//btConvexShape* convexShape = static_cast<btConvexShape*>(body->getCollisionShape());
					btlSphereShape tmpSphere(body->getCcdSweptSphereRadius());//btConvexShape* convexShape = static_cast<btConvexShape*>(body->getCollisionShape());
					sweepResults.m_allowedPenetration=getDispatchInfo().m_allowedCcdPenetration;

					sweepResults.m_collisionFilterGroup = body->getBroadphaseProxy()->m_collisionFilterGroup;
					sweepResults.m_collisionFilterMask  = body->getBroadphaseProxy()->m_collisionFilterMask;
					btlTransform modifiedPredictedTrans = predictedTrans;
					modifiedPredictedTrans.setBasis(body->getWorldTransform().getBasis());

					convexSweepTest(&tmpSphere,body->getWorldTransform(),modifiedPredictedTrans,sweepResults);
					if (sweepResults.hasHit() && (sweepResults.m_closestHitFraction < 1.f))
					{

						//printf("clamped integration to hit fraction = %f\n",fraction);
						body->setHitFraction(sweepResults.m_closestHitFraction);
						body->predictIntegratedTransform(timeStep*body->getHitFraction(), predictedTrans);
						body->setHitFraction(0.f);
						body->proceedToTransform( predictedTrans);

#if 0
						btlVector3 linVel = body->getLinearVelocity();

						btlScalar maxSpeed = body->getCcdMotionThreshold()/getSolverInfo().m_timeStep;
						btlScalar maxSpeedSqr = maxSpeed*maxSpeed;
						if (linVel.length2()>maxSpeedSqr)
						{
							linVel.normalize();
							linVel*= maxSpeed;
							body->setLinearVelocity(linVel);
							btlScalar ms2 = body->getLinearVelocity().length2();
							body->predictIntegratedTransform(timeStep, predictedTrans);

							btlScalar sm2 = (predictedTrans.getOrigin()-body->getWorldTransform().getOrigin()).length2();
							btlScalar smt = body->getCcdSquareMotionThreshold();
							printf("sm2=%f\n",sm2);
						}
#else

						//don't apply the collision response right now, it will happen next frame
						//if you really need to, you can uncomment next 3 lines. Note that is uses zero restitution.
						//btScalar appliedImpulse = 0.f;
						//btScalar depth = 0.f;
						//appliedImpulse = resolveSingleCollision(body,(btCollisionObject*)sweepResults.m_hitCollisionObject,sweepResults.m_hitPointWorld,sweepResults.m_hitNormalWorld,getSolverInfo(), depth);


#endif

        				continue;
					}
				}
			}


			body->proceedToTransform( predictedTrans);

		}

	}

	///this should probably be switched on by default, but it is not well tested yet
	if (m_applySpeculativeContactRestitution)
	{
		BT_PROFILE("apply speculative contact restitution");
		for (int i=0;i<m_predictiveManifolds.size();i++)
		{
			btlPersistentManifold* manifold = m_predictiveManifolds[i];
			btlRigidBody* body0 = btlRigidBody::upcast((btlCollisionObject*)manifold->getBody0());
			btlRigidBody* body1 = btlRigidBody::upcast((btlCollisionObject*)manifold->getBody1());

			for (int p=0;p<manifold->getNumContacts();p++)
			{
				const btlManifoldPoint& pt = manifold->getContactPoint(p);
				btlScalar combinedRestitution = btlManifoldResult::calculateCombinedRestitution(body0, body1);

				if (combinedRestitution>0 && pt.m_appliedImpulse != 0.f)
				//if (pt.getDistance()>0 && combinedRestitution>0 && pt.m_appliedImpulse != 0.f)
				{
					btlVector3 imp = -pt.m_normalWorldOnB * pt.m_appliedImpulse* combinedRestitution;

					const btlVector3& pos1 = pt.getPositionWorldOnA();
					const btlVector3& pos2 = pt.getPositionWorldOnB();

					btlVector3 rel_pos0 = pos1 - body0->getWorldTransform().getOrigin();
					btlVector3 rel_pos1 = pos2 - body1->getWorldTransform().getOrigin();

					if (body0)
						body0->applyImpulse(imp,rel_pos0);
					if (body1)
						body1->applyImpulse(-imp,rel_pos1);
				}
			}
		}
	}

}






void	btlDiscreteDynamicsWorld::predictUnconstraintMotion(btlScalar timeStep)
{
	BT_PROFILE("predictUnconstraintMotion");
	for ( int i=0;i<m_nonStaticRigidBodies.size();i++)
	{
		btlRigidBody* body = m_nonStaticRigidBodies[i];
		if (!body->isStaticOrKinematicObject())
		{
			//don't integrate/update velocities here, it happens in the constraint solver

			body->applyDamping(timeStep);

			body->predictIntegratedTransform(timeStep,body->getInterpolationWorldTransform());
		}
	}
}


void	btlDiscreteDynamicsWorld::startProfiling(btlScalar timeStep)
{
	(void)timeStep;

#ifndef BT_NO_PROFILE
	btl_CProfileManager::Reset();
#endif //BT_NO_PROFILE

}






void btlDiscreteDynamicsWorld::debugDrawConstraint(btlTypedConstraint* constraint)
{
	bool drawFrames = (getDebugDrawer()->getDebugMode() & btlIDebugDraw::DBG_DrawConstraints) != 0;
	bool drawLimits = (getDebugDrawer()->getDebugMode() & btlIDebugDraw::DBG_DrawConstraintLimits) != 0;
	btlScalar dbgDrawSize = constraint->getDbgDrawSize();
	if(dbgDrawSize <= btlScalar(0.f))
	{
		return;
	}

	switch(constraint->getConstraintType())
	{
		case POINT2POINT_CONSTRAINT_TYPE:
			{
				btlPoint2PointConstraint* p2pC = (btlPoint2PointConstraint*)constraint;
				btlTransform tr;
				tr.setIdentity();
				btlVector3 pivot = p2pC->getPivotInA();
				pivot = p2pC->getRigidBodyA().getCenterOfMassTransform() * pivot;
				tr.setOrigin(pivot);
				getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				// that ideally should draw the same frame
				pivot = p2pC->getPivotInB();
				pivot = p2pC->getRigidBodyB().getCenterOfMassTransform() * pivot;
				tr.setOrigin(pivot);
				if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
			}
			break;
		case HINGE_CONSTRAINT_TYPE:
			{
				btlHingeConstraint* pHinge = (btlHingeConstraint*)constraint;
				btlTransform tr = pHinge->getRigidBodyA().getCenterOfMassTransform() * pHinge->getAFrame();
				if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				tr = pHinge->getRigidBodyB().getCenterOfMassTransform() * pHinge->getBFrame();
				if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				btlScalar minAng = pHinge->getLowerLimit();
				btlScalar maxAng = pHinge->getUpperLimit();
				if(minAng == maxAng)
				{
					break;
				}
				bool drawSect = true;
				if(!pHinge->hasLimit())
				{
					minAng = btlScalar(0.f);
					maxAng = SIMD_2_PI;
					drawSect = false;
				}
				if(drawLimits)
				{
					btlVector3& center = tr.getOrigin();
					btlVector3 normal = tr.getBasis().getColumn(2);
					btlVector3 axis = tr.getBasis().getColumn(0);
					getDebugDrawer()->drawArc(center, normal, axis, dbgDrawSize, dbgDrawSize, minAng, maxAng, btlVector3(0,0,0), drawSect);
				}
			}
			break;
		case CONETWIST_CONSTRAINT_TYPE:
			{
				btlConeTwistConstraint* pCT = (btlConeTwistConstraint*)constraint;
				btlTransform tr = pCT->getRigidBodyA().getCenterOfMassTransform() * pCT->getAFrame();
				if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				tr = pCT->getRigidBodyB().getCenterOfMassTransform() * pCT->getBFrame();
				if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				if(drawLimits)
				{
					//const btScalar length = btScalar(5);
					const btlScalar length = dbgDrawSize;
					static int nSegments = 8*4;
					btlScalar fAngleInRadians = btlScalar(2.*3.1415926) * (btlScalar)(nSegments-1)/btlScalar(nSegments);
					btlVector3 pPrev = pCT->GetPointForAngle(fAngleInRadians, length);
					pPrev = tr * pPrev;
					for (int i=0; i<nSegments; i++)
					{
						fAngleInRadians = btlScalar(2.*3.1415926) * (btlScalar)i/btlScalar(nSegments);
						btlVector3 pCur = pCT->GetPointForAngle(fAngleInRadians, length);
						pCur = tr * pCur;
						getDebugDrawer()->drawLine(pPrev, pCur, btlVector3(0,0,0));

						if (i%(nSegments/8) == 0)
							getDebugDrawer()->drawLine(tr.getOrigin(), pCur, btlVector3(0,0,0));

						pPrev = pCur;
					}
					btlScalar tws = pCT->getTwistSpan();
					btlScalar twa = pCT->getTwistAngle();
					bool useFrameB = (pCT->getRigidBodyB().getInvMass() > btlScalar(0.f));
					if(useFrameB)
					{
						tr = pCT->getRigidBodyB().getCenterOfMassTransform() * pCT->getBFrame();
					}
					else
					{
						tr = pCT->getRigidBodyA().getCenterOfMassTransform() * pCT->getAFrame();
					}
					btlVector3 pivot = tr.getOrigin();
					btlVector3 normal = tr.getBasis().getColumn(0);
					btlVector3 axis1 = tr.getBasis().getColumn(1);
					getDebugDrawer()->drawArc(pivot, normal, axis1, dbgDrawSize, dbgDrawSize, -twa-tws, -twa+tws, btlVector3(0,0,0), true);

				}
			}
			break;
		case D6_SPRING_CONSTRAINT_TYPE:
		case D6_CONSTRAINT_TYPE:
			{
				btlGeneric6DofConstraint* p6DOF = (btlGeneric6DofConstraint*)constraint;
				btlTransform tr = p6DOF->getCalculatedTransformA();
				if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				tr = p6DOF->getCalculatedTransformB();
				if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				if(drawLimits)
				{
					tr = p6DOF->getCalculatedTransformA();
					const btlVector3& center = p6DOF->getCalculatedTransformB().getOrigin();
					btlVector3 up = tr.getBasis().getColumn(2);
					btlVector3 axis = tr.getBasis().getColumn(0);
					btlScalar minTh = p6DOF->getRotationalLimitMotor(1)->m_loLimit;
					btlScalar maxTh = p6DOF->getRotationalLimitMotor(1)->m_hiLimit;
					btlScalar minPs = p6DOF->getRotationalLimitMotor(2)->m_loLimit;
					btlScalar maxPs = p6DOF->getRotationalLimitMotor(2)->m_hiLimit;
					getDebugDrawer()->drawSpherePatch(center, up, axis, dbgDrawSize * btlScalar(.9f), minTh, maxTh, minPs, maxPs, btlVector3(0,0,0));
					axis = tr.getBasis().getColumn(1);
					btlScalar ay = p6DOF->getAngle(1);
					btlScalar az = p6DOF->getAngle(2);
					btlScalar cy = btlCos(ay);
					btlScalar sy = btlSin(ay);
					btlScalar cz = btlCos(az);
					btlScalar sz = btlSin(az);
					btlVector3 ref;
					ref[0] = cy*cz*axis[0] + cy*sz*axis[1] - sy*axis[2];
					ref[1] = -sz*axis[0] + cz*axis[1];
					ref[2] = cz*sy*axis[0] + sz*sy*axis[1] + cy*axis[2];
					tr = p6DOF->getCalculatedTransformB();
					btlVector3 normal = -tr.getBasis().getColumn(0);
					btlScalar minFi = p6DOF->getRotationalLimitMotor(0)->m_loLimit;
					btlScalar maxFi = p6DOF->getRotationalLimitMotor(0)->m_hiLimit;
					if(minFi > maxFi)
					{
						getDebugDrawer()->drawArc(center, normal, ref, dbgDrawSize, dbgDrawSize, -SIMD_PI, SIMD_PI, btlVector3(0,0,0), false);
					}
					else if(minFi < maxFi)
					{
						getDebugDrawer()->drawArc(center, normal, ref, dbgDrawSize, dbgDrawSize, minFi, maxFi, btlVector3(0,0,0), true);
					}
					tr = p6DOF->getCalculatedTransformA();
					btlVector3 bbMin = p6DOF->getTranslationalLimitMotor()->m_lowerLimit;
					btlVector3 bbMax = p6DOF->getTranslationalLimitMotor()->m_upperLimit;
					getDebugDrawer()->drawBox(bbMin, bbMax, tr, btlVector3(0,0,0));
				}
			}
			break;
		///note: the code for D6_SPRING_2_CONSTRAINT_TYPE is identical to D6_CONSTRAINT_TYPE, the D6_CONSTRAINT_TYPE+D6_SPRING_CONSTRAINT_TYPE will likely become obsolete/deprecated at some stage
		case D6_SPRING_2_CONSTRAINT_TYPE:
		{
			{
				btlGeneric6DofSpring2Constraint* p6DOF = (btlGeneric6DofSpring2Constraint*)constraint;
				btlTransform tr = p6DOF->getCalculatedTransformA();
				if (drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				tr = p6DOF->getCalculatedTransformB();
				if (drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				if (drawLimits)
				{
					tr = p6DOF->getCalculatedTransformA();
					const btlVector3& center = p6DOF->getCalculatedTransformB().getOrigin();
					btlVector3 up = tr.getBasis().getColumn(2);
					btlVector3 axis = tr.getBasis().getColumn(0);
					btlScalar minTh = p6DOF->getRotationalLimitMotor(1)->m_loLimit;
					btlScalar maxTh = p6DOF->getRotationalLimitMotor(1)->m_hiLimit;
					btlScalar minPs = p6DOF->getRotationalLimitMotor(2)->m_loLimit;
					btlScalar maxPs = p6DOF->getRotationalLimitMotor(2)->m_hiLimit;
					getDebugDrawer()->drawSpherePatch(center, up, axis, dbgDrawSize * btlScalar(.9f), minTh, maxTh, minPs, maxPs, btlVector3(0, 0, 0));
					axis = tr.getBasis().getColumn(1);
					btlScalar ay = p6DOF->getAngle(1);
					btlScalar az = p6DOF->getAngle(2);
					btlScalar cy = btlCos(ay);
					btlScalar sy = btlSin(ay);
					btlScalar cz = btlCos(az);
					btlScalar sz = btlSin(az);
					btlVector3 ref;
					ref[0] = cy*cz*axis[0] + cy*sz*axis[1] - sy*axis[2];
					ref[1] = -sz*axis[0] + cz*axis[1];
					ref[2] = cz*sy*axis[0] + sz*sy*axis[1] + cy*axis[2];
					tr = p6DOF->getCalculatedTransformB();
					btlVector3 normal = -tr.getBasis().getColumn(0);
					btlScalar minFi = p6DOF->getRotationalLimitMotor(0)->m_loLimit;
					btlScalar maxFi = p6DOF->getRotationalLimitMotor(0)->m_hiLimit;
					if (minFi > maxFi)
					{
						getDebugDrawer()->drawArc(center, normal, ref, dbgDrawSize, dbgDrawSize, -SIMD_PI, SIMD_PI, btlVector3(0, 0, 0), false);
					}
					else if (minFi < maxFi)
					{
						getDebugDrawer()->drawArc(center, normal, ref, dbgDrawSize, dbgDrawSize, minFi, maxFi, btlVector3(0, 0, 0), true);
					}
					tr = p6DOF->getCalculatedTransformA();
					btlVector3 bbMin = p6DOF->getTranslationalLimitMotor()->m_lowerLimit;
					btlVector3 bbMax = p6DOF->getTranslationalLimitMotor()->m_upperLimit;
					getDebugDrawer()->drawBox(bbMin, bbMax, tr, btlVector3(0, 0, 0));
				}
			}
			break;
		}
		case SLIDER_CONSTRAINT_TYPE:
			{
				btlSliderConstraint* pSlider = (btlSliderConstraint*)constraint;
				btlTransform tr = pSlider->getCalculatedTransformA();
				if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				tr = pSlider->getCalculatedTransformB();
				if(drawFrames) getDebugDrawer()->drawTransform(tr, dbgDrawSize);
				if(drawLimits)
				{
					btlTransform tr = pSlider->getUseLinearReferenceFrameA() ? pSlider->getCalculatedTransformA() : pSlider->getCalculatedTransformB();
					btlVector3 li_min = tr * btlVector3(pSlider->getLowerLinLimit(), 0.f, 0.f);
					btlVector3 li_max = tr * btlVector3(pSlider->getUpperLinLimit(), 0.f, 0.f);
					getDebugDrawer()->drawLine(li_min, li_max, btlVector3(0, 0, 0));
					btlVector3 normal = tr.getBasis().getColumn(0);
					btlVector3 axis = tr.getBasis().getColumn(1);
					btlScalar a_min = pSlider->getLowerAngLimit();
					btlScalar a_max = pSlider->getUpperAngLimit();
					const btlVector3& center = pSlider->getCalculatedTransformB().getOrigin();
					getDebugDrawer()->drawArc(center, normal, axis, dbgDrawSize, dbgDrawSize, a_min, a_max, btlVector3(0,0,0), true);
				}
			}
			break;
		default :
			break;
	}
	return;
}





void	btlDiscreteDynamicsWorld::setConstraintSolver(btlConstraintSolver* solver)
{
	if (m_ownsConstraintSolver)
	{
		btlAlignedFree( m_constraintSolver);
	}
	m_ownsConstraintSolver = false;
	m_constraintSolver = solver;
	m_solverIslandCallback->m_solver = solver;
}

btlConstraintSolver* btlDiscreteDynamicsWorld::getConstraintSolver()
{
	return m_constraintSolver;
}


int		btlDiscreteDynamicsWorld::getNumConstraints() const
{
	return int(m_constraints.size());
}
btlTypedConstraint* btlDiscreteDynamicsWorld::getConstraint(int index)
{
	return m_constraints[index];
}
const btlTypedConstraint* btlDiscreteDynamicsWorld::getConstraint(int index) const
{
	return m_constraints[index];
}



void	btlDiscreteDynamicsWorld::serializeRigidBodies(btlSerializer* serializer)
{
	int i;
	//serialize all collision objects
	for (i=0;i<m_collisionObjects.size();i++)
	{
		btlCollisionObject* colObj = m_collisionObjects[i];
		if (colObj->getInternalType() & btlCollisionObject::CO_RIGID_BODY)
		{
			int len = colObj->calculateSerializeBufferSize();
			btlChunk* chunk = serializer->allocate(len,1);
			const char* structType = colObj->serialize(chunk->m_oldPtr, serializer);
			serializer->finalizeChunk(chunk,structType,BT_RIGIDBODY_CODE,colObj);
		}
	}

	for (i=0;i<m_constraints.size();i++)
	{
		btlTypedConstraint* constraint = m_constraints[i];
		int size = constraint->calculateSerializeBufferSize();
		btlChunk* chunk = serializer->allocate(size,1);
		const char* structType = constraint->serialize(chunk->m_oldPtr,serializer);
		serializer->finalizeChunk(chunk,structType,BT_CONSTRAINT_CODE,constraint);
	}
}




void	btlDiscreteDynamicsWorld::serializeDynamicsWorldInfo(btlSerializer* serializer)
{
#ifdef BT_USE_DOUBLE_PRECISION
		int len = sizeof(btlDynamicsWorldDoubleData);
		btlChunk* chunk = serializer->allocate(len,1);
		btlDynamicsWorldDoubleData* worldInfo = (btlDynamicsWorldDoubleData*)chunk->m_oldPtr;
#else//BT_USE_DOUBLE_PRECISION
		int len = sizeof(btlDynamicsWorldFloatData);
		btlChunk* chunk = serializer->allocate(len,1);
		btlDynamicsWorldFloatData* worldInfo = (btlDynamicsWorldFloatData*)chunk->m_oldPtr;
#endif//BT_USE_DOUBLE_PRECISION

		memset(worldInfo ,0x00,len);

		m_gravity.serialize(worldInfo->m_gravity);
		worldInfo->m_solverInfo.m_tau = getSolverInfo().m_tau;
		worldInfo->m_solverInfo.m_damping = getSolverInfo().m_damping;
		worldInfo->m_solverInfo.m_friction = getSolverInfo().m_friction;
		worldInfo->m_solverInfo.m_timeStep = getSolverInfo().m_timeStep;

		worldInfo->m_solverInfo.m_restitution = getSolverInfo().m_restitution;
		worldInfo->m_solverInfo.m_maxErrorReduction = getSolverInfo().m_maxErrorReduction;
		worldInfo->m_solverInfo.m_sor = getSolverInfo().m_sor;
		worldInfo->m_solverInfo.m_erp = getSolverInfo().m_erp;

		worldInfo->m_solverInfo.m_erp2 = getSolverInfo().m_erp2;
		worldInfo->m_solverInfo.m_globalCfm = getSolverInfo().m_globalCfm;
		worldInfo->m_solverInfo.m_splitImpulsePenetrationThreshold = getSolverInfo().m_splitImpulsePenetrationThreshold;
		worldInfo->m_solverInfo.m_splitImpulseTurnErp = getSolverInfo().m_splitImpulseTurnErp;

		worldInfo->m_solverInfo.m_linearSlop = getSolverInfo().m_linearSlop;
		worldInfo->m_solverInfo.m_warmstartingFactor = getSolverInfo().m_warmstartingFactor;
		worldInfo->m_solverInfo.m_maxGyroscopicForce = getSolverInfo().m_maxGyroscopicForce;
		worldInfo->m_solverInfo.m_singleAxisRollingFrictionThreshold = getSolverInfo().m_singleAxisRollingFrictionThreshold;

		worldInfo->m_solverInfo.m_numIterations = getSolverInfo().m_numIterations;
		worldInfo->m_solverInfo.m_solverMode = getSolverInfo().m_solverMode;
		worldInfo->m_solverInfo.m_restingContactRestitutionThreshold = getSolverInfo().m_restingContactRestitutionThreshold;
		worldInfo->m_solverInfo.m_minimumSolverBatchSize = getSolverInfo().m_minimumSolverBatchSize;

		worldInfo->m_solverInfo.m_splitImpulse = getSolverInfo().m_splitImpulse;

#ifdef BT_USE_DOUBLE_PRECISION
		const char* structType = "btDynamicsWorldDoubleData";
#else//BT_USE_DOUBLE_PRECISION
		const char* structType = "btDynamicsWorldFloatData";
#endif//BT_USE_DOUBLE_PRECISION
		serializer->finalizeChunk(chunk,structType,BT_DYNAMICSWORLD_CODE,worldInfo);
}

void	btlDiscreteDynamicsWorld::serialize(btlSerializer* serializer)
{

	serializer->startSerialization();

	serializeDynamicsWorldInfo(serializer);

	serializeCollisionObjects(serializer);

	serializeRigidBodies(serializer);

	serializer->finishSerialization();
}

