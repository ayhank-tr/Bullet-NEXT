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


#ifndef BT_DISCRETE_DYNAMICS_WORLD_H
#define BT_DISCRETE_DYNAMICS_WORLD_H

#include "btDynamicsWorld.h"

class btlDispatcher;
class btlOverlappingPairCache;
class btlConstraintSolver;
class btlSimulationIslandManager;
class btlTypedConstraint;
class btlActionInterface;
class btlPersistentManifold;
class btlIDebugDraw;
struct btl_InplaceSolverIslandCallback;

#include "LinearMath/btAlignedObjectArray.h"


///btDiscreteDynamicsWorld provides discrete rigid body simulation
///those classes replace the obsolete CcdPhysicsEnvironment/CcdPhysicsController
ATTRIBUTE_ALIGNED16(class) btlDiscreteDynamicsWorld : public btlDynamicsWorld
{
protected:
	
    btlAlignedObjectArray<btlTypedConstraint*>	m_sortedConstraints;
	btl_InplaceSolverIslandCallback* 	m_solverIslandCallback;

	btlConstraintSolver*	m_constraintSolver;

	btlSimulationIslandManager*	m_islandManager;

	btlAlignedObjectArray<btlTypedConstraint*> m_constraints;

	btlAlignedObjectArray<btlRigidBody*> m_nonStaticRigidBodies;

	btlVector3	m_gravity;

	//for variable timesteps
	btlScalar	m_localTime;
	btlScalar	m_fixedTimeStep;
	//for variable timesteps

	bool	m_ownsIslandManager;
	bool	m_ownsConstraintSolver;
	bool	m_synchronizeAllMotionStates;
	bool	m_applySpeculativeContactRestitution;

	btlAlignedObjectArray<btlActionInterface*>	m_actions;
	
	int	m_profileTimings;

	bool	m_latencyMotionStateInterpolation;

	btlAlignedObjectArray<btlPersistentManifold*>	m_predictiveManifolds;

	virtual void	predictUnconstraintMotion(btlScalar timeStep);
	
	virtual void	integrateTransforms(btlScalar timeStep);
		
	virtual void	calculateSimulationIslands();

	virtual void	solveConstraints(btlContactSolverInfo& solverInfo);
	
	virtual void	updateActivationState(btlScalar timeStep);

	void	updateActions(btlScalar timeStep);

	void	startProfiling(btlScalar timeStep);

	virtual void	internalSingleStepSimulation( btlScalar timeStep);

	void	createPredictiveContacts(btlScalar timeStep);

	virtual void	saveKinematicState(btlScalar timeStep);

	void	serializeRigidBodies(btlSerializer* serializer);

	void	serializeDynamicsWorldInfo(btlSerializer* serializer);

public:


	BT_DECLARE_ALIGNED_ALLOCATOR();

	///this btDiscreteDynamicsWorld constructor gets created objects from the user, and will not delete those
	btlDiscreteDynamicsWorld(btlDispatcher* dispatcher,btlBroadphaseInterface* pairCache,btlConstraintSolver* constraintSolver,btlCollisionConfiguration* collisionConfiguration);

	virtual ~btlDiscreteDynamicsWorld();

	///if maxSubSteps > 0, it will interpolate motion between fixedTimeStep's
	virtual int	stepSimulation( btlScalar timeStep,int maxSubSteps=1, btlScalar fixedTimeStep=btlScalar(1.)/btlScalar(60.));


	virtual void	synchronizeMotionStates();

	///this can be useful to synchronize a single rigid body -> graphics object
	void	synchronizeSingleMotionState(btlRigidBody* body);

	virtual void	addConstraint(btlTypedConstraint* constraint, bool disableCollisionsBetweenLinkedBodies=false);

	virtual void	removeConstraint(btlTypedConstraint* constraint);

	virtual void	addAction(btlActionInterface*);

	virtual void	removeAction(btlActionInterface*);
	
	btlSimulationIslandManager*	getSimulationIslandManager()
	{
		return m_islandManager;
	}

	const btlSimulationIslandManager*	getSimulationIslandManager() const 
	{
		return m_islandManager;
	}

	btlCollisionWorld*	getCollisionWorld()
	{
		return this;
	}

	virtual void	setGravity(const btlVector3& gravity);

	virtual btlVector3 getGravity () const;

	virtual void	addCollisionObject(btlCollisionObject* collisionObject,short int collisionFilterGroup=btlBroadphaseProxy::StaticFilter,short int collisionFilterMask=btlBroadphaseProxy::AllFilter ^ btlBroadphaseProxy::StaticFilter);

	virtual void	addRigidBody(btlRigidBody* body);

	virtual void	addRigidBody(btlRigidBody* body, short group, short mask);

	virtual void	removeRigidBody(btlRigidBody* body);

	///removeCollisionObject will first check if it is a rigid body, if so call removeRigidBody otherwise call btCollisionWorld::removeCollisionObject
	virtual void	removeCollisionObject(btlCollisionObject* collisionObject);


	virtual void	debugDrawConstraint(btlTypedConstraint* constraint);

	virtual void	debugDrawWorld();

	virtual void	setConstraintSolver(btlConstraintSolver* solver);

	virtual btlConstraintSolver* getConstraintSolver();
	
	virtual	int		getNumConstraints() const;

	virtual btlTypedConstraint* getConstraint(int index)	;

	virtual const btlTypedConstraint* getConstraint(int index) const;

	
	virtual btlDynamicsWorldType	getWorldType() const
	{
		return BT_DISCRETE_DYNAMICS_WORLD;
	}
	
	///the forces on each rigidbody is accumulating together with gravity. clear this after each timestep.
	virtual void	clearForces();

	///apply gravity, call this once per timestep
	virtual void	applyGravity();

	virtual void	setNumTasks(int numTasks)
	{
        (void) numTasks;
	}

	///obsolete, use updateActions instead
	virtual void updateVehicles(btlScalar timeStep)
	{
		updateActions(timeStep);
	}

	///obsolete, use addAction instead
	virtual void	addVehicle(btlActionInterface* vehicle);
	///obsolete, use removeAction instead
	virtual void	removeVehicle(btlActionInterface* vehicle);
	///obsolete, use addAction instead
	virtual void	addCharacter(btlActionInterface* character);
	///obsolete, use removeAction instead
	virtual void	removeCharacter(btlActionInterface* character);

	void	setSynchronizeAllMotionStates(bool synchronizeAll)
	{
		m_synchronizeAllMotionStates = synchronizeAll;
	}
	bool getSynchronizeAllMotionStates() const
	{
		return m_synchronizeAllMotionStates;
	}

	void setApplySpeculativeContactRestitution(bool enable)
	{
		m_applySpeculativeContactRestitution = enable;
	}
	
	bool getApplySpeculativeContactRestitution() const
	{
		return m_applySpeculativeContactRestitution;
	}

	///Preliminary serialization test for Bullet 2.76. Loading those files requires a separate parser (see Bullet/Demos/SerializeDemo)
	virtual	void	serialize(btlSerializer* serializer);

	///Interpolate motion state between previous and current transform, instead of current and next transform.
	///This can relieve discontinuities in the rendering, due to penetrations
	void setLatencyMotionStateInterpolation(bool latencyInterpolation )
	{
		m_latencyMotionStateInterpolation = latencyInterpolation;
	}
	bool getLatencyMotionStateInterpolation() const
	{
		return m_latencyMotionStateInterpolation;
	}
};

#endif //BT_DISCRETE_DYNAMICS_WORLD_H
