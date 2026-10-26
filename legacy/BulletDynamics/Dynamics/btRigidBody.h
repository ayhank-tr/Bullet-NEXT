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

#ifndef BT_RIGIDBODY_H
#define BT_RIGIDBODY_H

#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btTransform.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"

class btlCollisionShape;
class btlMotionState;
class btlTypedConstraint;


extern btlScalar btl_gDeactivationTime;
extern bool btl_gDisableDeactivation;

#ifdef BT_USE_DOUBLE_PRECISION
#define btlRigidBodyData	btlRigidBodyDoubleData
#define btlRigidBodyDataName	"btRigidBodyDoubleData"
#else
#define btlRigidBodyData	btlRigidBodyFloatData
#define btlRigidBodyDataName	"btRigidBodyFloatData"
#endif //BT_USE_DOUBLE_PRECISION


enum	btlRigidBodyFlags
{
	BT_DISABLE_WORLD_GRAVITY = 1,
	///BT_ENABLE_GYROPSCOPIC_FORCE flags is enabled by default in Bullet 2.83 and onwards.
	///and it BT_ENABLE_GYROPSCOPIC_FORCE becomes equivalent to BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_BODY
	///See Demos/GyroscopicDemo and computeGyroscopicImpulseImplicit
	BT_ENABLE_GYROSCOPIC_FORCE_EXPLICIT = 2,
	BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_WORLD=4,
	BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_BODY=8,
	BT_ENABLE_GYROPSCOPIC_FORCE = BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_BODY,
};


///The btRigidBody is the main class for rigid body objects. It is derived from btCollisionObject, so it keeps a pointer to a btCollisionShape.
///It is recommended for performance and memory use to share btCollisionShape objects whenever possible.
///There are 3 types of rigid bodies: 
///- A) Dynamic rigid bodies, with positive mass. Motion is controlled by rigid body dynamics.
///- B) Fixed objects with zero mass. They are not moving (basically collision objects)
///- C) Kinematic objects, which are objects without mass, but the user can move them. There is on-way interaction, and Bullet calculates a velocity based on the timestep and previous and current world transform.
///Bullet automatically deactivates dynamic rigid bodies, when the velocity is below a threshold for a given time.
///Deactivated (sleeping) rigid bodies don't take any processing time, except a minor broadphase collision detection impact (to allow active objects to activate/wake up sleeping objects)
class btlRigidBody  : public btlCollisionObject
{

	btlMatrix3x3	m_invInertiaTensorWorld;
	btlVector3		m_linearVelocity;
	btlVector3		m_angularVelocity;
	btlScalar		m_inverseMass;
	btlVector3		m_linearFactor;

	btlVector3		m_gravity;	
	btlVector3		m_gravity_acceleration;
	btlVector3		m_invInertiaLocal;
	btlVector3		m_totalForce;
	btlVector3		m_totalTorque;
	
	btlScalar		m_linearDamping;
	btlScalar		m_angularDamping;

	bool			m_additionalDamping;
	btlScalar		m_additionalDampingFactor;
	btlScalar		m_additionalLinearDampingThresholdSqr;
	btlScalar		m_additionalAngularDampingThresholdSqr;
	btlScalar		m_additionalAngularDampingFactor;


	btlScalar		m_linearSleepingThreshold;
	btlScalar		m_angularSleepingThreshold;

	//m_optionalMotionState allows to automatic synchronize the world transform for active objects
	btlMotionState*	m_optionalMotionState;

	//keep track of typed constraints referencing this rigid body, to disable collision between linked bodies
	btlAlignedObjectArray<btlTypedConstraint*> m_constraintRefs;

	int				m_rigidbodyFlags;
	
	int				m_debugBodyId;
	

protected:

	ATTRIBUTE_ALIGNED16(btlVector3		m_deltaLinearVelocity);
	btlVector3		m_deltaAngularVelocity;
	btlVector3		m_angularFactor;
	btlVector3		m_invMass;
	btlVector3		m_pushVelocity;
	btlVector3		m_turnVelocity;


public:


	///The btRigidBodyConstructionInfo structure provides information to create a rigid body. Setting mass to zero creates a fixed (non-dynamic) rigid body.
	///For dynamic objects, you can use the collision shape to approximate the local inertia tensor, otherwise use the zero vector (default argument)
	///You can use the motion state to synchronize the world transform between physics and graphics objects. 
	///And if the motion state is provided, the rigid body will initialize its initial world transform from the motion state,
	///m_startWorldTransform is only used when you don't provide a motion state.
	struct	btlRigidBodyConstructionInfo
	{
		btlScalar			m_mass;

		///When a motionState is provided, the rigid body will initialize its world transform from the motion state
		///In this case, m_startWorldTransform is ignored.
		btlMotionState*		m_motionState;
		btlTransform	m_startWorldTransform;

		btlCollisionShape*	m_collisionShape;
		btlVector3			m_localInertia;
		btlScalar			m_linearDamping;
		btlScalar			m_angularDamping;

		///best simulation results when friction is non-zero
		btlScalar			m_friction;
		///the m_rollingFriction prevents rounded shapes, such as spheres, cylinders and capsules from rolling forever.
		///See Bullet/Demos/RollingFrictionDemo for usage
		btlScalar			m_rollingFriction;
		///best simulation results using zero restitution.
		btlScalar			m_restitution;

		btlScalar			m_linearSleepingThreshold;
		btlScalar			m_angularSleepingThreshold;

		//Additional damping can help avoiding lowpass jitter motion, help stability for ragdolls etc.
		//Such damping is undesirable, so once the overall simulation quality of the rigid body dynamics system has improved, this should become obsolete
		bool				m_additionalDamping;
		btlScalar			m_additionalDampingFactor;
		btlScalar			m_additionalLinearDampingThresholdSqr;
		btlScalar			m_additionalAngularDampingThresholdSqr;
		btlScalar			m_additionalAngularDampingFactor;

		btlRigidBodyConstructionInfo(	btlScalar mass, btlMotionState* motionState, btlCollisionShape* collisionShape, const btlVector3& localInertia=btlVector3(0,0,0)):
		m_mass(mass),
			m_motionState(motionState),
			m_collisionShape(collisionShape),
			m_localInertia(localInertia),
			m_linearDamping(btlScalar(0.)),
			m_angularDamping(btlScalar(0.)),
			m_friction(btlScalar(0.5)),
			m_rollingFriction(btlScalar(0)),
			m_restitution(btlScalar(0.)),
			m_linearSleepingThreshold(btlScalar(0.8)),
			m_angularSleepingThreshold(btlScalar(1.f)),
			m_additionalDamping(false),
			m_additionalDampingFactor(btlScalar(0.005)),
			m_additionalLinearDampingThresholdSqr(btlScalar(0.01)),
			m_additionalAngularDampingThresholdSqr(btlScalar(0.01)),
			m_additionalAngularDampingFactor(btlScalar(0.01))
		{
			m_startWorldTransform.setIdentity();
		}
	};

	///btRigidBody constructor using construction info
	btlRigidBody(	const btlRigidBodyConstructionInfo& constructionInfo);

	///btRigidBody constructor for backwards compatibility. 
	///To specify friction (etc) during rigid body construction, please use the other constructor (using btRigidBodyConstructionInfo)
	btlRigidBody(	btlScalar mass, btlMotionState* motionState, btlCollisionShape* collisionShape, const btlVector3& localInertia=btlVector3(0,0,0));


	virtual ~btlRigidBody()
        { 
                //No constraints should point to this rigidbody
		//Remove constraints from the dynamics world before you delete the related rigidbodies. 
                btlAssert(m_constraintRefs.size()==0); 
        }

protected:

	///setupRigidBody is only used internally by the constructor
	void	setupRigidBody(const btlRigidBodyConstructionInfo& constructionInfo);

public:

	void			proceedToTransform(const btlTransform& newTrans); 
	
	///to keep collision detection and dynamics separate we don't store a rigidbody pointer
	///but a rigidbody is derived from btCollisionObject, so we can safely perform an upcast
	static const btlRigidBody*	upcast(const btlCollisionObject* colObj)
	{
		if (colObj->getInternalType()&btlCollisionObject::CO_RIGID_BODY)
			return (const btlRigidBody*)colObj;
		return 0;
	}
	static btlRigidBody*	upcast(btlCollisionObject* colObj)
	{
		if (colObj->getInternalType()&btlCollisionObject::CO_RIGID_BODY)
			return (btlRigidBody*)colObj;
		return 0;
	}
	
	/// continuous collision detection needs prediction
	void			predictIntegratedTransform(btlScalar step, btlTransform& predictedTransform) ;
	
	void			saveKinematicState(btlScalar step);
	
	void			applyGravity();
	
	void			setGravity(const btlVector3& acceleration);  

	const btlVector3&	getGravity() const
	{
		return m_gravity_acceleration;
	}

	void			setDamping(btlScalar lin_damping, btlScalar ang_damping);

	btlScalar getLinearDamping() const
	{
		return m_linearDamping;
	}

	btlScalar getAngularDamping() const
	{
		return m_angularDamping;
	}

	btlScalar getLinearSleepingThreshold() const
	{
		return m_linearSleepingThreshold;
	}

	btlScalar getAngularSleepingThreshold() const
	{
		return m_angularSleepingThreshold;
	}

	void			applyDamping(btlScalar timeStep);

	SIMD_FORCE_INLINE const btlCollisionShape*	getCollisionShape() const {
		return m_collisionShape;
	}

	SIMD_FORCE_INLINE btlCollisionShape*	getCollisionShape() {
			return m_collisionShape;
	}
	
	void			setMassProps(btlScalar mass, const btlVector3& inertia);
	
	const btlVector3& getLinearFactor() const
	{
		return m_linearFactor;
	}
	void setLinearFactor(const btlVector3& linearFactor)
	{
		m_linearFactor = linearFactor;
		m_invMass = m_linearFactor*m_inverseMass;
	}
	btlScalar		getInvMass() const { return m_inverseMass; }
	const btlMatrix3x3& getInvInertiaTensorWorld() const { 
		return m_invInertiaTensorWorld; 
	}
		
	void			integrateVelocities(btlScalar step);

	void			setCenterOfMassTransform(const btlTransform& xform);

	void			applyCentralForce(const btlVector3& force)
	{
		m_totalForce += force*m_linearFactor;
	}

	const btlVector3& getTotalForce() const
	{
		return m_totalForce;
	};

	const btlVector3& getTotalTorque() const
	{
		return m_totalTorque;
	};
    
	const btlVector3& getInvInertiaDiagLocal() const
	{
		return m_invInertiaLocal;
	};

	void	setInvInertiaDiagLocal(const btlVector3& diagInvInertia)
	{
		m_invInertiaLocal = diagInvInertia;
	}

	void	setSleepingThresholds(btlScalar linear,btlScalar angular)
	{
		m_linearSleepingThreshold = linear;
		m_angularSleepingThreshold = angular;
	}

	void	applyTorque(const btlVector3& torque)
	{
		m_totalTorque += torque*m_angularFactor;
	}
	
	void	applyForce(const btlVector3& force, const btlVector3& rel_pos) 
	{
		applyCentralForce(force);
		applyTorque(rel_pos.cross(force*m_linearFactor));
	}
	
	void applyCentralImpulse(const btlVector3& impulse)
	{
		m_linearVelocity += impulse *m_linearFactor * m_inverseMass;
	}
	
  	void applyTorqueImpulse(const btlVector3& torque)
	{
			m_angularVelocity += m_invInertiaTensorWorld * torque * m_angularFactor;
	}
	
	void applyImpulse(const btlVector3& impulse, const btlVector3& rel_pos) 
	{
		if (m_inverseMass != btlScalar(0.))
		{
			applyCentralImpulse(impulse);
			if (m_angularFactor)
			{
				applyTorqueImpulse(rel_pos.cross(impulse*m_linearFactor));
			}
		}
	}

	void clearForces() 
	{
		m_totalForce.setValue(btlScalar(0.0), btlScalar(0.0), btlScalar(0.0));
		m_totalTorque.setValue(btlScalar(0.0), btlScalar(0.0), btlScalar(0.0));
	}
	
	void updateInertiaTensor();    
	
	const btlVector3&     getCenterOfMassPosition() const { 
		return m_worldTransform.getOrigin(); 
	}
	btlQuaternion getOrientation() const;
	
	const btlTransform&  getCenterOfMassTransform() const { 
		return m_worldTransform; 
	}
	const btlVector3&   getLinearVelocity() const { 
		return m_linearVelocity; 
	}
	const btlVector3&    getAngularVelocity() const { 
		return m_angularVelocity; 
	}
	

	inline void setLinearVelocity(const btlVector3& lin_vel)
	{ 
		m_updateRevision++;
		m_linearVelocity = lin_vel; 
	}

	inline void setAngularVelocity(const btlVector3& ang_vel) 
	{ 
		m_updateRevision++;
		m_angularVelocity = ang_vel; 
	}

	btlVector3 getVelocityInLocalPoint(const btlVector3& rel_pos) const
	{
		//we also calculate lin/ang velocity for kinematic objects
		return m_linearVelocity + m_angularVelocity.cross(rel_pos);

		//for kinematic objects, we could also use use:
		//		return 	(m_worldTransform(rel_pos) - m_interpolationWorldTransform(rel_pos)) / m_kinematicTimeStep;
	}

	void translate(const btlVector3& v) 
	{
		m_worldTransform.getOrigin() += v; 
	}

	
	void	getAabb(btlVector3& aabbMin,btlVector3& aabbMax) const;




	
	SIMD_FORCE_INLINE btlScalar computeImpulseDenominator(const btlVector3& pos, const btlVector3& normal) const
	{
		btlVector3 r0 = pos - getCenterOfMassPosition();

		btlVector3 c0 = (r0).cross(normal);

		btlVector3 vec = (c0 * getInvInertiaTensorWorld()).cross(r0);

		return m_inverseMass + normal.dot(vec);

	}

	SIMD_FORCE_INLINE btlScalar computeAngularImpulseDenominator(const btlVector3& axis) const
	{
		btlVector3 vec = axis * getInvInertiaTensorWorld();
		return axis.dot(vec);
	}

	SIMD_FORCE_INLINE void	updateDeactivation(btlScalar timeStep)
	{
		if ( (getActivationState() == ISLAND_SLEEPING) || (getActivationState() == DISABLE_DEACTIVATION))
			return;

		if ((getLinearVelocity().length2() < m_linearSleepingThreshold*m_linearSleepingThreshold) &&
			(getAngularVelocity().length2() < m_angularSleepingThreshold*m_angularSleepingThreshold))
		{
			m_deactivationTime += timeStep;
		} else
		{
			m_deactivationTime=btlScalar(0.);
			setActivationState(0);
		}

	}

	SIMD_FORCE_INLINE bool	wantsSleeping()
	{

		if (getActivationState() == DISABLE_DEACTIVATION)
			return false;

		//disable deactivation
		if (btl_gDisableDeactivation || (btl_gDeactivationTime == btlScalar(0.)))
			return false;

		if ( (getActivationState() == ISLAND_SLEEPING) || (getActivationState() == WANTS_DEACTIVATION))
			return true;

		if (m_deactivationTime> btl_gDeactivationTime)
		{
			return true;
		}
		return false;
	}


	
	const btlBroadphaseProxy*	getBroadphaseProxy() const
	{
		return m_broadphaseHandle;
	}
	btlBroadphaseProxy*	getBroadphaseProxy() 
	{
		return m_broadphaseHandle;
	}
	void	setNewBroadphaseProxy(btlBroadphaseProxy* broadphaseProxy)
	{
		m_broadphaseHandle = broadphaseProxy;
	}

	//btMotionState allows to automatic synchronize the world transform for active objects
	btlMotionState*	getMotionState()
	{
		return m_optionalMotionState;
	}
	const btlMotionState*	getMotionState() const
	{
		return m_optionalMotionState;
	}
	void	setMotionState(btlMotionState* motionState)
	{
		m_optionalMotionState = motionState;
		if (m_optionalMotionState)
			motionState->getWorldTransform(m_worldTransform);
	}

	//for experimental overriding of friction/contact solver func
	int	m_contactSolverType;
	int	m_frictionSolverType;

	void	setAngularFactor(const btlVector3& angFac)
	{
		m_updateRevision++;
		m_angularFactor = angFac;
	}

	void	setAngularFactor(btlScalar angFac)
	{
		m_updateRevision++;
		m_angularFactor.setValue(angFac,angFac,angFac);
	}
	const btlVector3&	getAngularFactor() const
	{
		return m_angularFactor;
	}

	//is this rigidbody added to a btCollisionWorld/btDynamicsWorld/btBroadphase?
	bool	isInWorld() const
	{
		return (getBroadphaseProxy() != 0);
	}

	virtual bool checkCollideWithOverride(const  btlCollisionObject* co) const;

	void addConstraintRef(btlTypedConstraint* c);
	void removeConstraintRef(btlTypedConstraint* c);

	btlTypedConstraint* getConstraintRef(int index)
	{
		return m_constraintRefs[index];
	}

	int getNumConstraintRefs() const
	{
		return m_constraintRefs.size();
	}

	void	setFlags(int flags)
	{
		m_rigidbodyFlags = flags;
	}

	int getFlags() const
	{
		return m_rigidbodyFlags;
	}


	

	///perform implicit force computation in world space
	btlVector3 computeGyroscopicImpulseImplicit_World(btlScalar dt) const;
	
	///perform implicit force computation in body space (inertial frame)
	btlVector3 computeGyroscopicImpulseImplicit_Body(btlScalar step) const;

	///explicit version is best avoided, it gains energy
	btlVector3 computeGyroscopicForceExplicit(btlScalar maxGyroscopicForce) const;
	btlVector3 getLocalInertia() const;

	///////////////////////////////////////////////

	virtual	int	calculateSerializeBufferSize()	const;

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual	const char*	serialize(void* dataBuffer,  class btlSerializer* serializer) const;

	virtual void serializeSingleObject(class btlSerializer* serializer) const;

};

//@todo add m_optionalMotionState and m_constraintRefs to btRigidBodyData
///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct	btlRigidBodyFloatData
{
	btlCollisionObjectFloatData	m_collisionObjectData;
	btlMatrix3x3FloatData		m_invInertiaTensorWorld;
	btlVector3FloatData		m_linearVelocity;
	btlVector3FloatData		m_angularVelocity;
	btlVector3FloatData		m_angularFactor;
	btlVector3FloatData		m_linearFactor;
	btlVector3FloatData		m_gravity;	
	btlVector3FloatData		m_gravity_acceleration;
	btlVector3FloatData		m_invInertiaLocal;
	btlVector3FloatData		m_totalForce;
	btlVector3FloatData		m_totalTorque;
	float					m_inverseMass;
	float					m_linearDamping;
	float					m_angularDamping;
	float					m_additionalDampingFactor;
	float					m_additionalLinearDampingThresholdSqr;
	float					m_additionalAngularDampingThresholdSqr;
	float					m_additionalAngularDampingFactor;
	float					m_linearSleepingThreshold;
	float					m_angularSleepingThreshold;
	int						m_additionalDamping;
};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct	btlRigidBodyDoubleData
{
	btlCollisionObjectDoubleData	m_collisionObjectData;
	btlMatrix3x3DoubleData		m_invInertiaTensorWorld;
	btlVector3DoubleData		m_linearVelocity;
	btlVector3DoubleData		m_angularVelocity;
	btlVector3DoubleData		m_angularFactor;
	btlVector3DoubleData		m_linearFactor;
	btlVector3DoubleData		m_gravity;	
	btlVector3DoubleData		m_gravity_acceleration;
	btlVector3DoubleData		m_invInertiaLocal;
	btlVector3DoubleData		m_totalForce;
	btlVector3DoubleData		m_totalTorque;
	double					m_inverseMass;
	double					m_linearDamping;
	double					m_angularDamping;
	double					m_additionalDampingFactor;
	double					m_additionalLinearDampingThresholdSqr;
	double					m_additionalAngularDampingThresholdSqr;
	double					m_additionalAngularDampingFactor;
	double					m_linearSleepingThreshold;
	double					m_angularSleepingThreshold;
	int						m_additionalDamping;
	char	m_padding[4];
};



#endif //BT_RIGIDBODY_H

