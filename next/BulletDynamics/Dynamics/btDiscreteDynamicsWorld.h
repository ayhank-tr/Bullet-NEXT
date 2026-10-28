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
class btDispatcher;
class btOverlappingPairCache;
class btConstraintSolver;
class btSimulationIslandManager;
class btTypedConstraint;
class btActionInterface;
class btPersistentManifold;
class btIDebugDraw;

struct InplaceSolverIslandCallback;

#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btThreads.h"

///btDiscreteDynamicsWorld provides discrete rigid body simulation
///those classes replace the obsolete CcdPhysicsEnvironment/CcdPhysicsController
ATTRIBUTE_ALIGNED16(class)
btDiscreteDynamicsWorld : public btDynamicsWorld
{
protected:
	btAlignedObjectArray<btTypedConstraint*> m_sortedConstraints;
	InplaceSolverIslandCallback* m_solverIslandCallback;

	btConstraintSolver* m_constraintSolver;

	btSimulationIslandManager* m_islandManager;

	btAlignedObjectArray<btTypedConstraint*> m_constraints;

	btAlignedObjectArray<btRigidBody*> m_nonStaticRigidBodies;

	btVector3 m_gravity;

	//for variable timesteps
	btScalar m_localTime;
	btScalar m_fixedTimeStep;
	//for variable timesteps

	bool m_ownsIslandManager;
	bool m_ownsConstraintSolver;
	bool m_synchronizeAllMotionStates;
	bool m_applySpeculativeContactRestitution;

	btAlignedObjectArray<btActionInterface*> m_actions;

	int m_profileTimings;

	bool m_latencyMotionStateInterpolation;

	btAlignedObjectArray<btPersistentManifold*> m_predictiveManifolds;
	btSpinMutex m_predictiveManifoldsMutex;  // used to synchronize threads creating predictive contacts

	virtual void predictUnconstraintMotion(btScalar timeStep);

	void integrateTransformsInternal(btRigidBody * *bodies, int numBodies, btScalar timeStep);  // can be called in parallel
	virtual void integrateTransforms(btScalar timeStep);

	virtual void calculateSimulationIslands();

	

	virtual void updateActivationState(btScalar timeStep);

	void updateActions(btScalar timeStep);

	void startProfiling(btScalar timeStep);

	virtual void internalSingleStepSimulation(btScalar timeStep);

public:
	/// Bullet NEXT (F1): sort contact manifolds into a canonical order (body index pairs).
	void bnSortManifolds();

	/// Bullet NEXT (F1, C1.9): two-pass integration (see .cpp). fraction: -1 skip, <1 clamped;
	/// deltaV: velocity change that ends the approach at the time of impact.
	struct BnCcdResult
	{
		btVector3 deltaV;
		btScalar fraction;
		const btCollisionObject* hitObject;  // Stage 1: for the crash report
		btVector3 hitPoint, hitNormal;
	};
	/// Bullet NEXT (Stage 1, crash report): one hit whose approach speed continuous collision
	/// removed at the time of impact. impulse = the body's mass times its change of velocity, in
	/// world space; the other object receives nothing from this step (it is static, kinematic, or
	/// gets its share from the contact on the next step). Contact impulses are in the manifolds.
	struct BnCcdHit
	{
		const btCollisionObject* body;
		const btCollisionObject* other;
		btVector3 point, normal, impulse;
	};
	void bnCollectCcdHits(btRigidBody** bodies, int numBodies, const BnCcdResult* results);
	/// The hits of the last internal step (see btNextCcdHits).
	const btAlignedObjectArray<BnCcdHit>& bnCcdHits() const { return m_bnCcdHits; }
	btAlignedObjectArray<BnCcdHit> m_bnCcdHits;
	void bnSweepTransforms(btRigidBody** bodies, int numBodies, btScalar timeStep, BnCcdResult* results);
	void bnApplyTransforms(btRigidBody** bodies, int numBodies, btScalar timeStep, const BnCcdResult* results);
	btAlignedObjectArray<BnCcdResult> m_bnHitFractions;
	/// Bullet NEXT (F3c): where a body's step takes it, or the given fraction of the way. Soft Step
	/// (BT_SOFT_STEP_SOLVER) has already moved the body and left the end transform in its
	/// interpolation transform; the other solvers leave velocities, integrated here.
	void bnPredictTransform(btRigidBody* body, btScalar timeStep, btScalar fraction, btTransform& out) const;

	/// Bullet NEXT (F1, C1.9): two-pass predictive contacts (see .cpp).
	struct BnPredictiveHit
	{
		const btCollisionObject* hitObject;
		btVector3 distVec, normal, hitPoint;
	};
	void bnSweepPredictive(btRigidBody** bodies, int numBodies, btScalar timeStep, BnPredictiveHit* hits);
	void bnCreatePredictive(btRigidBody** bodies, int numBodies, const BnPredictiveHit* hits);
	btAlignedObjectArray<BnPredictiveHit> m_bnPredictiveHits;

protected:

	void releasePredictiveContacts();
	void createPredictiveContactsInternal(btRigidBody * *bodies, int numBodies, btScalar timeStep);  // can be called in parallel
	virtual void createPredictiveContacts(btScalar timeStep);

	virtual void saveKinematicState(btScalar timeStep);

	void serializeRigidBodies(btSerializer * serializer);

	void serializeDynamicsWorldInfo(btSerializer * serializer);
    
public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	///this btDiscreteDynamicsWorld constructor gets created objects from the user, and will not delete those
	btDiscreteDynamicsWorld(btDispatcher * dispatcher, btBroadphaseInterface * pairCache, btConstraintSolver * constraintSolver, btCollisionConfiguration * collisionConfiguration);

	virtual ~btDiscreteDynamicsWorld();

	///if maxSubSteps > 0, it will interpolate motion between fixedTimeStep's
	virtual int stepSimulation(btScalar timeStep, int maxSubSteps = 1, btScalar fixedTimeStep = btScalar(1.) / btScalar(60.));

    virtual void solveConstraints(btContactSolverInfo & solverInfo);
    
	virtual void synchronizeMotionStates();

	///this can be useful to synchronize a single rigid body -> graphics object
	void synchronizeSingleMotionState(btRigidBody * body);

	virtual void addConstraint(btTypedConstraint * constraint, bool disableCollisionsBetweenLinkedBodies = false);

	virtual void removeConstraint(btTypedConstraint * constraint);

	virtual void addAction(btActionInterface*);

	virtual void removeAction(btActionInterface*);

	btSimulationIslandManager* getSimulationIslandManager()
	{
		return m_islandManager;
	}

	const btSimulationIslandManager* getSimulationIslandManager() const
	{
		return m_islandManager;
	}

	btCollisionWorld* getCollisionWorld()
	{
		return this;
	}

	virtual void setGravity(const btVector3& gravity);

	virtual btVector3 getGravity() const;

	virtual void addCollisionObject(btCollisionObject * collisionObject, int collisionFilterGroup = btBroadphaseProxy::StaticFilter, int collisionFilterMask = btBroadphaseProxy::AllFilter ^ btBroadphaseProxy::StaticFilter);

	virtual void addRigidBody(btRigidBody * body);

	virtual void addRigidBody(btRigidBody * body, int group, int mask);

	virtual void removeRigidBody(btRigidBody * body);

	///removeCollisionObject will first check if it is a rigid body, if so call removeRigidBody otherwise call btCollisionWorld::removeCollisionObject
	virtual void removeCollisionObject(btCollisionObject * collisionObject);

	virtual void debugDrawConstraint(btTypedConstraint * constraint);

	virtual void debugDrawWorld();

	virtual void setConstraintSolver(btConstraintSolver * solver);

	virtual btConstraintSolver* getConstraintSolver();

	virtual int getNumConstraints() const;

	virtual btTypedConstraint* getConstraint(int index);

	virtual const btTypedConstraint* getConstraint(int index) const;

	virtual btDynamicsWorldType getWorldType() const
	{
		return BT_DISCRETE_DYNAMICS_WORLD;
	}

	///the forces on each rigidbody is accumulating together with gravity. clear this after each timestep.
	virtual void clearForces();

	///apply gravity, call this once per timestep
	virtual void applyGravity();

	virtual void setNumTasks(int numTasks)
	{
		(void)numTasks;
	}

	///obsolete, use updateActions instead
	virtual void updateVehicles(btScalar timeStep)
	{
		updateActions(timeStep);
	}

	///obsolete, use addAction instead
	virtual void addVehicle(btActionInterface * vehicle);
	///obsolete, use removeAction instead
	virtual void removeVehicle(btActionInterface * vehicle);
	///obsolete, use addAction instead
	virtual void addCharacter(btActionInterface * character);
	///obsolete, use removeAction instead
	virtual void removeCharacter(btActionInterface * character);

	void setSynchronizeAllMotionStates(bool synchronizeAll)
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
	virtual void serialize(btSerializer * serializer);

	///Interpolate motion state between previous and current transform, instead of current and next transform.
	///This can relieve discontinuities in the rendering, due to penetrations
	void setLatencyMotionStateInterpolation(bool latencyInterpolation)
	{
		m_latencyMotionStateInterpolation = latencyInterpolation;
	}
	bool getLatencyMotionStateInterpolation() const
	{
		return m_latencyMotionStateInterpolation;
	}
    
    btAlignedObjectArray<btRigidBody*>& getNonStaticRigidBodies()
    {
        return m_nonStaticRigidBodies;
    }
    
    const btAlignedObjectArray<btRigidBody*>& getNonStaticRigidBodies() const
    {
        return m_nonStaticRigidBodies;
    }
};

#endif  //BT_DISCRETE_DYNAMICS_WORLD_H
