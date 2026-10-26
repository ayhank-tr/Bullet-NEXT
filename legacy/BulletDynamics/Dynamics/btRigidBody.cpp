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

#include "btRigidBody.h"
#include "BulletCollision/CollisionShapes/btConvexShape.h"
#include "LinearMath/btMinMax.h"
#include "LinearMath/btTransformUtil.h"
#include "LinearMath/btMotionState.h"
#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"
#include "LinearMath/btSerializer.h"

//'temporarily' global variables
btlScalar	btl_gDeactivationTime = btlScalar(2.);
bool	btl_gDisableDeactivation = false;
static int uniqueId = 0;


btlRigidBody::btlRigidBody(const btlRigidBody::btlRigidBodyConstructionInfo& constructionInfo)
{
	setupRigidBody(constructionInfo);
}

btlRigidBody::btlRigidBody(btlScalar mass, btlMotionState *motionState, btlCollisionShape *collisionShape, const btlVector3 &localInertia)
{
	btlRigidBodyConstructionInfo cinfo(mass,motionState,collisionShape,localInertia);
	setupRigidBody(cinfo);
}

void	btlRigidBody::setupRigidBody(const btlRigidBody::btlRigidBodyConstructionInfo& constructionInfo)
{

	m_internalType=CO_RIGID_BODY;

	m_linearVelocity.setValue(btlScalar(0.0), btlScalar(0.0), btlScalar(0.0));
	m_angularVelocity.setValue(btlScalar(0.),btlScalar(0.),btlScalar(0.));
	m_angularFactor.setValue(1,1,1);
	m_linearFactor.setValue(1,1,1);
	m_gravity.setValue(btlScalar(0.0), btlScalar(0.0), btlScalar(0.0));
	m_gravity_acceleration.setValue(btlScalar(0.0), btlScalar(0.0), btlScalar(0.0));
	m_totalForce.setValue(btlScalar(0.0), btlScalar(0.0), btlScalar(0.0));
	m_totalTorque.setValue(btlScalar(0.0), btlScalar(0.0), btlScalar(0.0)),
    setDamping(constructionInfo.m_linearDamping, constructionInfo.m_angularDamping);

	m_linearSleepingThreshold = constructionInfo.m_linearSleepingThreshold;
	m_angularSleepingThreshold = constructionInfo.m_angularSleepingThreshold;
	m_optionalMotionState = constructionInfo.m_motionState;
	m_contactSolverType = 0;
	m_frictionSolverType = 0;
	m_additionalDamping = constructionInfo.m_additionalDamping;
	m_additionalDampingFactor = constructionInfo.m_additionalDampingFactor;
	m_additionalLinearDampingThresholdSqr = constructionInfo.m_additionalLinearDampingThresholdSqr;
	m_additionalAngularDampingThresholdSqr = constructionInfo.m_additionalAngularDampingThresholdSqr;
	m_additionalAngularDampingFactor = constructionInfo.m_additionalAngularDampingFactor;

	if (m_optionalMotionState)
	{
		m_optionalMotionState->getWorldTransform(m_worldTransform);
	} else
	{
		m_worldTransform = constructionInfo.m_startWorldTransform;
	}

	m_interpolationWorldTransform = m_worldTransform;
	m_interpolationLinearVelocity.setValue(0,0,0);
	m_interpolationAngularVelocity.setValue(0,0,0);
	
	//moved to btCollisionObject
	m_friction = constructionInfo.m_friction;
	m_rollingFriction = constructionInfo.m_rollingFriction;
	m_restitution = constructionInfo.m_restitution;

	setCollisionShape( constructionInfo.m_collisionShape );
	m_debugBodyId = uniqueId++;
	
	setMassProps(constructionInfo.m_mass, constructionInfo.m_localInertia);
	updateInertiaTensor();

	m_rigidbodyFlags = BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_BODY;


	m_deltaLinearVelocity.setZero();
	m_deltaAngularVelocity.setZero();
	m_invMass = m_inverseMass*m_linearFactor;
	m_pushVelocity.setZero();
	m_turnVelocity.setZero();

	

}


void btlRigidBody::predictIntegratedTransform(btlScalar timeStep,btlTransform& predictedTransform) 
{
	btlTransformUtil::integrateTransform(m_worldTransform,m_linearVelocity,m_angularVelocity,timeStep,predictedTransform);
}

void			btlRigidBody::saveKinematicState(btlScalar timeStep)
{
	//todo: clamp to some (user definable) safe minimum timestep, to limit maximum angular/linear velocities
	if (timeStep != btlScalar(0.))
	{
		//if we use motionstate to synchronize world transforms, get the new kinematic/animated world transform
		if (getMotionState())
			getMotionState()->getWorldTransform(m_worldTransform);
		btlVector3 linVel,angVel;
		
		btlTransformUtil::calculateVelocity(m_interpolationWorldTransform,m_worldTransform,timeStep,m_linearVelocity,m_angularVelocity);
		m_interpolationLinearVelocity = m_linearVelocity;
		m_interpolationAngularVelocity = m_angularVelocity;
		m_interpolationWorldTransform = m_worldTransform;
		//printf("angular = %f %f %f\n",m_angularVelocity.getX(),m_angularVelocity.getY(),m_angularVelocity.getZ());
	}
}
	
void	btlRigidBody::getAabb(btlVector3& aabbMin,btlVector3& aabbMax) const
{
	getCollisionShape()->getAabb(m_worldTransform,aabbMin,aabbMax);
}




void btlRigidBody::setGravity(const btlVector3& acceleration) 
{
	if (m_inverseMass != btlScalar(0.0))
	{
		m_gravity = acceleration * (btlScalar(1.0) / m_inverseMass);
	}
	m_gravity_acceleration = acceleration;
}






void btlRigidBody::setDamping(btlScalar lin_damping, btlScalar ang_damping)
{
	m_linearDamping = btlClamped(lin_damping, (btlScalar)btlScalar(0.0), (btlScalar)btlScalar(1.0));
	m_angularDamping = btlClamped(ang_damping, (btlScalar)btlScalar(0.0), (btlScalar)btlScalar(1.0));
}




///applyDamping damps the velocity, using the given m_linearDamping and m_angularDamping
void			btlRigidBody::applyDamping(btlScalar timeStep)
{
	//On new damping: see discussion/issue report here: http://code.google.com/p/bullet/issues/detail?id=74
	//todo: do some performance comparisons (but other parts of the engine are probably bottleneck anyway

//#define USE_OLD_DAMPING_METHOD 1
#ifdef USE_OLD_DAMPING_METHOD
	m_linearVelocity *= GEN_clamped((btlScalar(1.) - timeStep * m_linearDamping), (btlScalar)btlScalar(0.0), (btlScalar)btlScalar(1.0));
	m_angularVelocity *= GEN_clamped((btlScalar(1.) - timeStep * m_angularDamping), (btlScalar)btlScalar(0.0), (btlScalar)btlScalar(1.0));
#else
	m_linearVelocity *= btlPow(btlScalar(1)-m_linearDamping, timeStep);
	m_angularVelocity *= btlPow(btlScalar(1)-m_angularDamping, timeStep);
#endif

	if (m_additionalDamping)
	{
		//Additional damping can help avoiding lowpass jitter motion, help stability for ragdolls etc.
		//Such damping is undesirable, so once the overall simulation quality of the rigid body dynamics system has improved, this should become obsolete
		if ((m_angularVelocity.length2() < m_additionalAngularDampingThresholdSqr) &&
			(m_linearVelocity.length2() < m_additionalLinearDampingThresholdSqr))
		{
			m_angularVelocity *= m_additionalDampingFactor;
			m_linearVelocity *= m_additionalDampingFactor;
		}
	

		btlScalar speed = m_linearVelocity.length();
		if (speed < m_linearDamping)
		{
			btlScalar dampVel = btlScalar(0.005);
			if (speed > dampVel)
			{
				btlVector3 dir = m_linearVelocity.normalized();
				m_linearVelocity -=  dir * dampVel;
			} else
			{
				m_linearVelocity.setValue(btlScalar(0.),btlScalar(0.),btlScalar(0.));
			}
		}

		btlScalar angSpeed = m_angularVelocity.length();
		if (angSpeed < m_angularDamping)
		{
			btlScalar angDampVel = btlScalar(0.005);
			if (angSpeed > angDampVel)
			{
				btlVector3 dir = m_angularVelocity.normalized();
				m_angularVelocity -=  dir * angDampVel;
			} else
			{
				m_angularVelocity.setValue(btlScalar(0.),btlScalar(0.),btlScalar(0.));
			}
		}
	}
}


void btlRigidBody::applyGravity()
{
	if (isStaticOrKinematicObject())
		return;
	
	applyCentralForce(m_gravity);	

}

void btlRigidBody::proceedToTransform(const btlTransform& newTrans)
{
	setCenterOfMassTransform( newTrans );
}
	

void btlRigidBody::setMassProps(btlScalar mass, const btlVector3& inertia)
{
	if (mass == btlScalar(0.))
	{
		m_collisionFlags |= btlCollisionObject::CF_STATIC_OBJECT;
		m_inverseMass = btlScalar(0.);
	} else
	{
		m_collisionFlags &= (~btlCollisionObject::CF_STATIC_OBJECT);
		m_inverseMass = btlScalar(1.0) / mass;
	}

	//Fg = m * a
	m_gravity = mass * m_gravity_acceleration;
	
	m_invInertiaLocal.setValue(inertia.x() != btlScalar(0.0) ? btlScalar(1.0) / inertia.x(): btlScalar(0.0),
				   inertia.y() != btlScalar(0.0) ? btlScalar(1.0) / inertia.y(): btlScalar(0.0),
				   inertia.z() != btlScalar(0.0) ? btlScalar(1.0) / inertia.z(): btlScalar(0.0));

	m_invMass = m_linearFactor*m_inverseMass;
}

	
void btlRigidBody::updateInertiaTensor() 
{
	m_invInertiaTensorWorld = m_worldTransform.getBasis().scaled(m_invInertiaLocal) * m_worldTransform.getBasis().transpose();
}



btlVector3 btlRigidBody::getLocalInertia() const
{

	btlVector3 inertiaLocal;
	const btlVector3 inertia = m_invInertiaLocal;
	inertiaLocal.setValue(inertia.x() != btlScalar(0.0) ? btlScalar(1.0) / inertia.x() : btlScalar(0.0),
		inertia.y() != btlScalar(0.0) ? btlScalar(1.0) / inertia.y() : btlScalar(0.0),
		inertia.z() != btlScalar(0.0) ? btlScalar(1.0) / inertia.z() : btlScalar(0.0));
	return inertiaLocal;
}

inline btlVector3 evalEulerEqn(const btlVector3& w1, const btlVector3& w0, const btlVector3& T, const btlScalar dt,
	const btlMatrix3x3 &I)
{
	const btlVector3 w2 = I*w1 + w1.cross(I*w1)*dt - (T*dt + I*w0);
	return w2;
}

inline btlMatrix3x3 evalEulerEqnDeriv(const btlVector3& w1, const btlVector3& w0, const btlScalar dt,
	const btlMatrix3x3 &I)
{

	btlMatrix3x3 w1x, Iw1x;
	const btlVector3 Iwi = (I*w1);
	w1.getSkewSymmetricMatrix(&w1x[0], &w1x[1], &w1x[2]);
	Iwi.getSkewSymmetricMatrix(&Iw1x[0], &Iw1x[1], &Iw1x[2]);

	const btlMatrix3x3 dfw1 = I + (w1x*I - Iw1x)*dt;
	return dfw1;
}

btlVector3 btlRigidBody::computeGyroscopicForceExplicit(btlScalar maxGyroscopicForce) const
{
	btlVector3 inertiaLocal = getLocalInertia();
	btlMatrix3x3 inertiaTensorWorld = getWorldTransform().getBasis().scaled(inertiaLocal) * getWorldTransform().getBasis().transpose();
	btlVector3 tmp = inertiaTensorWorld*getAngularVelocity();
	btlVector3 gf = getAngularVelocity().cross(tmp);
	btlScalar l2 = gf.length2();
	if (l2>maxGyroscopicForce*maxGyroscopicForce)
	{
		gf *= btlScalar(1.)/btlSqrt(l2)*maxGyroscopicForce;
	}
	return gf;
}


btlVector3 btlRigidBody::computeGyroscopicImpulseImplicit_Body(btlScalar step) const
{	
	btlVector3 idl = getLocalInertia();
	btlVector3 omega1 = getAngularVelocity();
	btlQuaternion q = getWorldTransform().getRotation();
	
	// Convert to body coordinates
	btlVector3 omegab = btl_quatRotate(q.inverse(), omega1);
	btlMatrix3x3 Ib;
	Ib.setValue(idl.x(),0,0,
				0,idl.y(),0,
				0,0,idl.z());
	
	btlVector3 ibo = Ib*omegab;

	// Residual vector
	btlVector3 f = step * omegab.cross(ibo);
	
	btlMatrix3x3 skew0;
	omegab.getSkewSymmetricMatrix(&skew0[0], &skew0[1], &skew0[2]);
	btlVector3 om = Ib*omegab;
	btlMatrix3x3 skew1;
	om.getSkewSymmetricMatrix(&skew1[0],&skew1[1],&skew1[2]);
	
	// Jacobian
	btlMatrix3x3 J = Ib +  (skew0*Ib - skew1)*step;
	
//	btMatrix3x3 Jinv = J.inverse();
//	btVector3 omega_div = Jinv*f;
	btlVector3 omega_div = J.solve33(f);
	
	// Single Newton-Raphson update
	omegab = omegab - omega_div;//Solve33(J, f);
	// Back to world coordinates
	btlVector3 omega2 = btl_quatRotate(q,omegab);
	btlVector3 gf = omega2-omega1;
	return gf;
}



btlVector3 btlRigidBody::computeGyroscopicImpulseImplicit_World(btlScalar step) const
{
	// use full newton-euler equations.  common practice to drop the wxIw term. want it for better tumbling behavior.
	// calculate using implicit euler step so it's stable.

	const btlVector3 inertiaLocal = getLocalInertia();
	const btlVector3 w0 = getAngularVelocity();

	btlMatrix3x3 I;

	I = m_worldTransform.getBasis().scaled(inertiaLocal) *
		m_worldTransform.getBasis().transpose();

	// use newtons method to find implicit solution for new angular velocity (w')
	// f(w') = -(T*step + Iw) + Iw' + w' + w'xIw'*step = 0 
	// df/dw' = I + 1xIw'*step + w'xI*step

	btlVector3 w1 = w0;

	// one step of newton's method
	{
		const btlVector3 fw = evalEulerEqn(w1, w0, btlVector3(0, 0, 0), step, I);
		const btlMatrix3x3 dfw = evalEulerEqnDeriv(w1, w0, step, I);

		btlVector3 dw;
		dw = dfw.solve33(fw);
		//const btMatrix3x3 dfw_inv = dfw.inverse();
		//dw = dfw_inv*fw;

		w1 -= dw;
	}

	btlVector3 gf = (w1 - w0);
	return gf;
}


void btlRigidBody::integrateVelocities(btlScalar step) 
{
	if (isStaticOrKinematicObject())
		return;

	m_linearVelocity += m_totalForce * (m_inverseMass * step);
	m_angularVelocity += m_invInertiaTensorWorld * m_totalTorque * step;

#define MAX_ANGVEL SIMD_HALF_PI
	/// clamp angular velocity. collision calculations will fail on higher angular velocities	
	btlScalar angvel = m_angularVelocity.length();
	if (angvel*step > MAX_ANGVEL)
	{
		m_angularVelocity *= (MAX_ANGVEL/step) /angvel;
	}

}

btlQuaternion btlRigidBody::getOrientation() const
{
		btlQuaternion orn;
		m_worldTransform.getBasis().getRotation(orn);
		return orn;
}
	
	
void btlRigidBody::setCenterOfMassTransform(const btlTransform& xform)
{

	if (isKinematicObject())
	{
		m_interpolationWorldTransform = m_worldTransform;
	} else
	{
		m_interpolationWorldTransform = xform;
	}
	m_interpolationLinearVelocity = getLinearVelocity();
	m_interpolationAngularVelocity = getAngularVelocity();
	m_worldTransform = xform;
	updateInertiaTensor();
}


bool btlRigidBody::checkCollideWithOverride(const  btlCollisionObject* co) const
{
	const btlRigidBody* otherRb = btlRigidBody::upcast(co);
	if (!otherRb)
		return true;

	for (int i = 0; i < m_constraintRefs.size(); ++i)
	{
		const btlTypedConstraint* c = m_constraintRefs[i];
		if (c->isEnabled())
			if (&c->getRigidBodyA() == otherRb || &c->getRigidBodyB() == otherRb)
				return false;
	}

	return true;
}



void btlRigidBody::addConstraintRef(btlTypedConstraint* c)
{
	int index = m_constraintRefs.findLinearSearch(c);
	if (index == m_constraintRefs.size())
		m_constraintRefs.push_back(c); 

	m_checkCollideWith = true;
}

void btlRigidBody::removeConstraintRef(btlTypedConstraint* c)
{
	m_constraintRefs.remove(c);
	m_checkCollideWith = m_constraintRefs.size() > 0;
}

int	btlRigidBody::calculateSerializeBufferSize()	const
{
	int sz = sizeof(btlRigidBodyData);
	return sz;
}

	///fills the dataBuffer and returns the struct name (and 0 on failure)
const char*	btlRigidBody::serialize(void* dataBuffer, class btlSerializer* serializer) const
{
	btlRigidBodyData* rbd = (btlRigidBodyData*) dataBuffer;

	btlCollisionObject::serialize(&rbd->m_collisionObjectData, serializer);

	m_invInertiaTensorWorld.serialize(rbd->m_invInertiaTensorWorld);
	m_linearVelocity.serialize(rbd->m_linearVelocity);
	m_angularVelocity.serialize(rbd->m_angularVelocity);
	rbd->m_inverseMass = m_inverseMass;
	m_angularFactor.serialize(rbd->m_angularFactor);
	m_linearFactor.serialize(rbd->m_linearFactor);
	m_gravity.serialize(rbd->m_gravity);
	m_gravity_acceleration.serialize(rbd->m_gravity_acceleration);
	m_invInertiaLocal.serialize(rbd->m_invInertiaLocal);
	m_totalForce.serialize(rbd->m_totalForce);
	m_totalTorque.serialize(rbd->m_totalTorque);
	rbd->m_linearDamping = m_linearDamping;
	rbd->m_angularDamping = m_angularDamping;
	rbd->m_additionalDamping = m_additionalDamping;
	rbd->m_additionalDampingFactor = m_additionalDampingFactor;
	rbd->m_additionalLinearDampingThresholdSqr = m_additionalLinearDampingThresholdSqr;
	rbd->m_additionalAngularDampingThresholdSqr = m_additionalAngularDampingThresholdSqr;
	rbd->m_additionalAngularDampingFactor = m_additionalAngularDampingFactor;
	rbd->m_linearSleepingThreshold=m_linearSleepingThreshold;
	rbd->m_angularSleepingThreshold = m_angularSleepingThreshold;

	return btlRigidBodyDataName;
}



void btlRigidBody::serializeSingleObject(class btlSerializer* serializer) const
{
	btlChunk* chunk = serializer->allocate(calculateSerializeBufferSize(),1);
	const char* structType = serialize(chunk->m_oldPtr, serializer);
	serializer->finalizeChunk(chunk,structType,BT_RIGIDBODY_CODE,(void*)this);
}


