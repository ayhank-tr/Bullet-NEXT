/*
 * Copyright (c) 2005 Erwin Coumans http://continuousphysics.com/Bullet/
 *
 * Permission to use, copy, modify, distribute and sell this software
 * and its documentation for any purpose is hereby granted without fee,
 * provided that the above copyright notice appear in all copies.
 * Erwin Coumans makes no representations about the suitability 
 * of this software for any purpose.  
 * It is provided "as is" without express or implied warranty.
*/

#include "LinearMath/btVector3.h"
#include "btRaycastVehicle.h"

#include "BulletDynamics/ConstraintSolver/btSolve2LinearConstraint.h"
#include "BulletDynamics/ConstraintSolver/btJacobianEntry.h"
#include "LinearMath/btQuaternion.h"
#include "BulletDynamics/Dynamics/btDynamicsWorld.h"
#include "btVehicleRaycaster.h"
#include "btWheelInfo.h"
#include "LinearMath/btMinMax.h"
#include "LinearMath/btIDebugDraw.h"
#include "BulletDynamics/ConstraintSolver/btContactConstraint.h"

#define ROLLING_INFLUENCE_FIX


btlRigidBody& btlActionInterface::getFixedBody()
{
	static btlRigidBody s_fixed(0, 0,0);
	s_fixed.setMassProps(btlScalar(0.),btlVector3(btlScalar(0.),btlScalar(0.),btlScalar(0.)));
	return s_fixed;
}

btlRaycastVehicle::btlRaycastVehicle(const btlVehicleTuning& tuning,btlRigidBody* chassis,	btlVehicleRaycaster* raycaster )
:m_vehicleRaycaster(raycaster),
m_pitchControl(btlScalar(0.))
{
	m_chassisBody = chassis;
	m_indexRightAxis = 0;
	m_indexUpAxis = 2;
	m_indexForwardAxis = 1;
	defaultInit(tuning);
}


void btlRaycastVehicle::defaultInit(const btlVehicleTuning& tuning)
{
	(void)tuning;
	m_currentVehicleSpeedKmHour = btlScalar(0.);
	m_steeringValue = btlScalar(0.);
	
}

	

btlRaycastVehicle::~btlRaycastVehicle()
{
}


//
// basically most of the code is general for 2 or 4 wheel vehicles, but some of it needs to be reviewed
//
btlWheelInfo&	btlRaycastVehicle::addWheel( const btlVector3& connectionPointCS, const btlVector3& wheelDirectionCS0,const btlVector3& wheelAxleCS, btlScalar suspensionRestLength, btlScalar wheelRadius,const btlVehicleTuning& tuning, bool isFrontWheel)
{

	btlWheelInfoConstructionInfo ci;

	ci.m_chassisConnectionCS = connectionPointCS;
	ci.m_wheelDirectionCS = wheelDirectionCS0;
	ci.m_wheelAxleCS = wheelAxleCS;
	ci.m_suspensionRestLength = suspensionRestLength;
	ci.m_wheelRadius = wheelRadius;
	ci.m_suspensionStiffness = tuning.m_suspensionStiffness;
	ci.m_wheelsDampingCompression = tuning.m_suspensionCompression;
	ci.m_wheelsDampingRelaxation = tuning.m_suspensionDamping;
	ci.m_frictionSlip = tuning.m_frictionSlip;
	ci.m_bIsFrontWheel = isFrontWheel;
	ci.m_maxSuspensionTravelCm = tuning.m_maxSuspensionTravelCm;
	ci.m_maxSuspensionForce = tuning.m_maxSuspensionForce;

	m_wheelInfo.push_back( btlWheelInfo(ci));
	
	btlWheelInfo& wheel = m_wheelInfo[getNumWheels()-1];
	
	updateWheelTransformsWS( wheel , false );
	updateWheelTransform(getNumWheels()-1,false);
	return wheel;
}




const btlTransform&	btlRaycastVehicle::getWheelTransformWS( int wheelIndex ) const
{
	btlAssert(wheelIndex < getNumWheels());
	const btlWheelInfo& wheel = m_wheelInfo[wheelIndex];
	return wheel.m_worldTransform;

}

void	btlRaycastVehicle::updateWheelTransform( int wheelIndex , bool interpolatedTransform)
{
	
	btlWheelInfo& wheel = m_wheelInfo[ wheelIndex ];
	updateWheelTransformsWS(wheel,interpolatedTransform);
	btlVector3 up = -wheel.m_raycastInfo.m_wheelDirectionWS;
	const btlVector3& right = wheel.m_raycastInfo.m_wheelAxleWS;
	btlVector3 fwd = up.cross(right);
	fwd = fwd.normalize();
//	up = right.cross(fwd);
//	up.normalize();

	//rotate around steering over de wheelAxleWS
	btlScalar steering = wheel.m_steering;
	
	btlQuaternion steeringOrn(up,steering);//wheel.m_steering);
	btlMatrix3x3 steeringMat(steeringOrn);

	btlQuaternion rotatingOrn(right,-wheel.m_rotation);
	btlMatrix3x3 rotatingMat(rotatingOrn);

	btlMatrix3x3 basis2(
		right[0],fwd[0],up[0],
		right[1],fwd[1],up[1],
		right[2],fwd[2],up[2]
	);
	
	wheel.m_worldTransform.setBasis(steeringMat * rotatingMat * basis2);
	wheel.m_worldTransform.setOrigin(
		wheel.m_raycastInfo.m_hardPointWS + wheel.m_raycastInfo.m_wheelDirectionWS * wheel.m_raycastInfo.m_suspensionLength
	);
}

void btlRaycastVehicle::resetSuspension()
{

	int i;
	for (i=0;i<m_wheelInfo.size();	i++)
	{
			btlWheelInfo& wheel = m_wheelInfo[i];
			wheel.m_raycastInfo.m_suspensionLength = wheel.getSuspensionRestLength();
			wheel.m_suspensionRelativeVelocity = btlScalar(0.0);
			
			wheel.m_raycastInfo.m_contactNormalWS = - wheel.m_raycastInfo.m_wheelDirectionWS;
			//wheel_info.setContactFriction(btScalar(0.0));
			wheel.m_clippedInvContactDotSuspension = btlScalar(1.0);
	}
}

void	btlRaycastVehicle::updateWheelTransformsWS(btlWheelInfo& wheel , bool interpolatedTransform)
{
	wheel.m_raycastInfo.m_isInContact = false;

	btlTransform chassisTrans = getChassisWorldTransform();
	if (interpolatedTransform && (getRigidBody()->getMotionState()))
	{
		getRigidBody()->getMotionState()->getWorldTransform(chassisTrans);
	}

	wheel.m_raycastInfo.m_hardPointWS = chassisTrans( wheel.m_chassisConnectionPointCS );
	wheel.m_raycastInfo.m_wheelDirectionWS = chassisTrans.getBasis() *  wheel.m_wheelDirectionCS ;
	wheel.m_raycastInfo.m_wheelAxleWS = chassisTrans.getBasis() * wheel.m_wheelAxleCS;
}

btlScalar btlRaycastVehicle::rayCast(btlWheelInfo& wheel)
{
	updateWheelTransformsWS( wheel,false);

	
	btlScalar depth = -1;
	
	btlScalar raylen = wheel.getSuspensionRestLength()+wheel.m_wheelsRadius;

	btlVector3 rayvector = wheel.m_raycastInfo.m_wheelDirectionWS * (raylen);
	const btlVector3& source = wheel.m_raycastInfo.m_hardPointWS;
	wheel.m_raycastInfo.m_contactPointWS = source + rayvector;
	const btlVector3& target = wheel.m_raycastInfo.m_contactPointWS;

	btlScalar param = btlScalar(0.);
	
	btlVehicleRaycaster::btlVehicleRaycasterResult	rayResults;

	btlAssert(m_vehicleRaycaster);

	void* object = m_vehicleRaycaster->castRay(source,target,rayResults);

	wheel.m_raycastInfo.m_groundObject = 0;

	if (object)
	{
		param = rayResults.m_distFraction;
		depth = raylen * rayResults.m_distFraction;
		wheel.m_raycastInfo.m_contactNormalWS  = rayResults.m_hitNormalInWorld;
		wheel.m_raycastInfo.m_isInContact = true;
		
		wheel.m_raycastInfo.m_groundObject = &getFixedBody();///@todo for driving on dynamic/movable objects!;
		//wheel.m_raycastInfo.m_groundObject = object;


		btlScalar hitDistance = param*raylen;
		wheel.m_raycastInfo.m_suspensionLength = hitDistance - wheel.m_wheelsRadius;
		//clamp on max suspension travel

		btlScalar  minSuspensionLength = wheel.getSuspensionRestLength() - wheel.m_maxSuspensionTravelCm*btlScalar(0.01);
		btlScalar maxSuspensionLength = wheel.getSuspensionRestLength()+ wheel.m_maxSuspensionTravelCm*btlScalar(0.01);
		if (wheel.m_raycastInfo.m_suspensionLength < minSuspensionLength)
		{
			wheel.m_raycastInfo.m_suspensionLength = minSuspensionLength;
		}
		if (wheel.m_raycastInfo.m_suspensionLength > maxSuspensionLength)
		{
			wheel.m_raycastInfo.m_suspensionLength = maxSuspensionLength;
		}

		wheel.m_raycastInfo.m_contactPointWS = rayResults.m_hitPointInWorld;

		btlScalar denominator= wheel.m_raycastInfo.m_contactNormalWS.dot( wheel.m_raycastInfo.m_wheelDirectionWS );

		btlVector3 chassis_velocity_at_contactPoint;
		btlVector3 relpos = wheel.m_raycastInfo.m_contactPointWS-getRigidBody()->getCenterOfMassPosition();

		chassis_velocity_at_contactPoint = getRigidBody()->getVelocityInLocalPoint(relpos);

		btlScalar projVel = wheel.m_raycastInfo.m_contactNormalWS.dot( chassis_velocity_at_contactPoint );

		if ( denominator >= btlScalar(-0.1))
		{
			wheel.m_suspensionRelativeVelocity = btlScalar(0.0);
			wheel.m_clippedInvContactDotSuspension = btlScalar(1.0) / btlScalar(0.1);
		}
		else
		{
			btlScalar inv = btlScalar(-1.) / denominator;
			wheel.m_suspensionRelativeVelocity = projVel * inv;
			wheel.m_clippedInvContactDotSuspension = inv;
		}
			
	} else
	{
		//put wheel info as in rest position
		wheel.m_raycastInfo.m_suspensionLength = wheel.getSuspensionRestLength();
		wheel.m_suspensionRelativeVelocity = btlScalar(0.0);
		wheel.m_raycastInfo.m_contactNormalWS = - wheel.m_raycastInfo.m_wheelDirectionWS;
		wheel.m_clippedInvContactDotSuspension = btlScalar(1.0);
	}

	return depth;
}


const btlTransform& btlRaycastVehicle::getChassisWorldTransform() const
{
	/*if (getRigidBody()->getMotionState())
	{
		btTransform chassisWorldTrans;
		getRigidBody()->getMotionState()->getWorldTransform(chassisWorldTrans);
		return chassisWorldTrans;
	}
	*/

	
	return getRigidBody()->getCenterOfMassTransform();
}


void btlRaycastVehicle::updateVehicle( btlScalar step )
{
	{
		for (int i=0;i<getNumWheels();i++)
		{
			updateWheelTransform(i,false);
		}
	}


	m_currentVehicleSpeedKmHour = btlScalar(3.6) * getRigidBody()->getLinearVelocity().length();
	
	const btlTransform& chassisTrans = getChassisWorldTransform();

	btlVector3 forwardW (
		chassisTrans.getBasis()[0][m_indexForwardAxis],
		chassisTrans.getBasis()[1][m_indexForwardAxis],
		chassisTrans.getBasis()[2][m_indexForwardAxis]);

	if (forwardW.dot(getRigidBody()->getLinearVelocity()) < btlScalar(0.))
	{
		m_currentVehicleSpeedKmHour *= btlScalar(-1.);
	}

	//
	// simulate suspension
	//
	
	int i=0;
	for (i=0;i<m_wheelInfo.size();i++)
	{
		//btScalar depth; 
		//depth = 
		rayCast( m_wheelInfo[i]);
	}

	updateSuspension(step);

	
	for (i=0;i<m_wheelInfo.size();i++)
	{
		//apply suspension force
		btlWheelInfo& wheel = m_wheelInfo[i];
		
		btlScalar suspensionForce = wheel.m_wheelsSuspensionForce;
		
		if (suspensionForce > wheel.m_maxSuspensionForce)
		{
			suspensionForce = wheel.m_maxSuspensionForce;
		}
		btlVector3 impulse = wheel.m_raycastInfo.m_contactNormalWS * suspensionForce * step;
		btlVector3 relpos = wheel.m_raycastInfo.m_contactPointWS - getRigidBody()->getCenterOfMassPosition();
		
		getRigidBody()->applyImpulse(impulse, relpos);
	
	}
	

	
	updateFriction( step);

	
	for (i=0;i<m_wheelInfo.size();i++)
	{
		btlWheelInfo& wheel = m_wheelInfo[i];
		btlVector3 relpos = wheel.m_raycastInfo.m_hardPointWS - getRigidBody()->getCenterOfMassPosition();
		btlVector3 vel = getRigidBody()->getVelocityInLocalPoint( relpos );

		if (wheel.m_raycastInfo.m_isInContact)
		{
			const btlTransform&	chassisWorldTransform = getChassisWorldTransform();

			btlVector3 fwd (
				chassisWorldTransform.getBasis()[0][m_indexForwardAxis],
				chassisWorldTransform.getBasis()[1][m_indexForwardAxis],
				chassisWorldTransform.getBasis()[2][m_indexForwardAxis]);

			btlScalar proj = fwd.dot(wheel.m_raycastInfo.m_contactNormalWS);
			fwd -= wheel.m_raycastInfo.m_contactNormalWS * proj;

			btlScalar proj2 = fwd.dot(vel);
			
			wheel.m_deltaRotation = (proj2 * step) / (wheel.m_wheelsRadius);
			wheel.m_rotation += wheel.m_deltaRotation;

		} else
		{
			wheel.m_rotation += wheel.m_deltaRotation;
		}
		
		wheel.m_deltaRotation *= btlScalar(0.99);//damping of rotation when not in contact

	}



}


void	btlRaycastVehicle::setSteeringValue(btlScalar steering,int wheel)
{
	btlAssert(wheel>=0 && wheel < getNumWheels());

	btlWheelInfo& wheelInfo = getWheelInfo(wheel);
	wheelInfo.m_steering = steering;
}



btlScalar	btlRaycastVehicle::getSteeringValue(int wheel) const
{
	return getWheelInfo(wheel).m_steering;
}


void	btlRaycastVehicle::applyEngineForce(btlScalar force, int wheel)
{
	btlAssert(wheel>=0 && wheel < getNumWheels());
	btlWheelInfo& wheelInfo = getWheelInfo(wheel);
	wheelInfo.m_engineForce = force;
}


const btlWheelInfo&	btlRaycastVehicle::getWheelInfo(int index) const
{
	btlAssert((index >= 0) && (index < 	getNumWheels()));
	
	return m_wheelInfo[index];
}

btlWheelInfo&	btlRaycastVehicle::getWheelInfo(int index) 
{
	btlAssert((index >= 0) && (index < 	getNumWheels()));
	
	return m_wheelInfo[index];
}

void btlRaycastVehicle::setBrake(btlScalar brake,int wheelIndex)
{
	btlAssert((wheelIndex >= 0) && (wheelIndex < 	getNumWheels()));
	getWheelInfo(wheelIndex).m_brake = brake;
}


void	btlRaycastVehicle::updateSuspension(btlScalar deltaTime)
{
	(void)deltaTime;

	btlScalar chassisMass = btlScalar(1.) / m_chassisBody->getInvMass();
	
	for (int w_it=0; w_it<getNumWheels(); w_it++)
	{
		btlWheelInfo &wheel_info = m_wheelInfo[w_it];
		
		if ( wheel_info.m_raycastInfo.m_isInContact )
		{
			btlScalar force;
			//	Spring
			{
				btlScalar	susp_length			= wheel_info.getSuspensionRestLength();
				btlScalar	current_length = wheel_info.m_raycastInfo.m_suspensionLength;

				btlScalar length_diff = (susp_length - current_length);

				force = wheel_info.m_suspensionStiffness
					* length_diff * wheel_info.m_clippedInvContactDotSuspension;
			}
		
			// Damper
			{
				btlScalar projected_rel_vel = wheel_info.m_suspensionRelativeVelocity;
				{
					btlScalar	susp_damping;
					if ( projected_rel_vel < btlScalar(0.0) )
					{
						susp_damping = wheel_info.m_wheelsDampingCompression;
					}
					else
					{
						susp_damping = wheel_info.m_wheelsDampingRelaxation;
					}
					force -= susp_damping * projected_rel_vel;
				}
			}

			// RESULT
			wheel_info.m_wheelsSuspensionForce = force * chassisMass;
			if (wheel_info.m_wheelsSuspensionForce < btlScalar(0.))
			{
				wheel_info.m_wheelsSuspensionForce = btlScalar(0.);
			}
		}
		else
		{
			wheel_info.m_wheelsSuspensionForce = btlScalar(0.0);
		}
	}

}


struct btlWheelContactPoint
{
	btlRigidBody* m_body0;
	btlRigidBody* m_body1;
	btlVector3	m_frictionPositionWorld;
	btlVector3	m_frictionDirectionWorld;
	btlScalar	m_jacDiagABInv;
	btlScalar	m_maxImpulse;


	btlWheelContactPoint(btlRigidBody* body0,btlRigidBody* body1,const btlVector3& frictionPosWorld,const btlVector3& frictionDirectionWorld, btlScalar maxImpulse)
		:m_body0(body0),
		m_body1(body1),
		m_frictionPositionWorld(frictionPosWorld),
		m_frictionDirectionWorld(frictionDirectionWorld),
		m_maxImpulse(maxImpulse)
	{
		btlScalar denom0 = body0->computeImpulseDenominator(frictionPosWorld,frictionDirectionWorld);
		btlScalar denom1 = body1->computeImpulseDenominator(frictionPosWorld,frictionDirectionWorld);
		btlScalar	relaxation = 1.f;
		m_jacDiagABInv = relaxation/(denom0+denom1);
	}



};

btlScalar btl_calcRollingFriction(btlWheelContactPoint& contactPoint);
btlScalar btl_calcRollingFriction(btlWheelContactPoint& contactPoint)
{

	btlScalar j1=0.f;

	const btlVector3& contactPosWorld = contactPoint.m_frictionPositionWorld;

	btlVector3 rel_pos1 = contactPosWorld - contactPoint.m_body0->getCenterOfMassPosition(); 
	btlVector3 rel_pos2 = contactPosWorld - contactPoint.m_body1->getCenterOfMassPosition();
	
	btlScalar maxImpulse  = contactPoint.m_maxImpulse;
	
	btlVector3 vel1 = contactPoint.m_body0->getVelocityInLocalPoint(rel_pos1);
	btlVector3 vel2 = contactPoint.m_body1->getVelocityInLocalPoint(rel_pos2);
	btlVector3 vel = vel1 - vel2;

	btlScalar vrel = contactPoint.m_frictionDirectionWorld.dot(vel);

	// calculate j that moves us to zero relative velocity
	j1 = -vrel * contactPoint.m_jacDiagABInv;
	btlSetMin(j1, maxImpulse);
	btlSetMax(j1, -maxImpulse);

	return j1;
}




btlScalar btl_sideFrictionStiffness2 = btlScalar(1.0);
void	btlRaycastVehicle::updateFriction(btlScalar	timeStep)
{

		//calculate the impulse, so that the wheels don't move sidewards
		int numWheel = getNumWheels();
		if (!numWheel)
			return;

		m_forwardWS.resize(numWheel);
		m_axle.resize(numWheel);
		m_forwardImpulse.resize(numWheel);
		m_sideImpulse.resize(numWheel);
		
		int numWheelsOnGround = 0;
	

		//collapse all those loops into one!
		for (int i=0;i<getNumWheels();i++)
		{
			btlWheelInfo& wheelInfo = m_wheelInfo[i];
			class btlRigidBody* groundObject = (class btlRigidBody*) wheelInfo.m_raycastInfo.m_groundObject;
			if (groundObject)
				numWheelsOnGround++;
			m_sideImpulse[i] = btlScalar(0.);
			m_forwardImpulse[i] = btlScalar(0.);

		}
	
		{
	
			for (int i=0;i<getNumWheels();i++)
			{

				btlWheelInfo& wheelInfo = m_wheelInfo[i];
					
				class btlRigidBody* groundObject = (class btlRigidBody*) wheelInfo.m_raycastInfo.m_groundObject;

				if (groundObject)
				{

					const btlTransform& wheelTrans = getWheelTransformWS( i );

					btlMatrix3x3 wheelBasis0 = wheelTrans.getBasis();
					m_axle[i] = btlVector3(	
						wheelBasis0[0][m_indexRightAxis],
						wheelBasis0[1][m_indexRightAxis],
						wheelBasis0[2][m_indexRightAxis]);
					
					const btlVector3& surfNormalWS = wheelInfo.m_raycastInfo.m_contactNormalWS;
					btlScalar proj = m_axle[i].dot(surfNormalWS);
					m_axle[i] -= surfNormalWS * proj;
					m_axle[i] = m_axle[i].normalize();
					
					m_forwardWS[i] = surfNormalWS.cross(m_axle[i]);
					m_forwardWS[i].normalize();

				
					btl_resolveSingleBilateral(*m_chassisBody, wheelInfo.m_raycastInfo.m_contactPointWS,
							  *groundObject, wheelInfo.m_raycastInfo.m_contactPointWS,
							  btlScalar(0.), m_axle[i],m_sideImpulse[i],timeStep);

					m_sideImpulse[i] *= btl_sideFrictionStiffness2;
						
				}
				

			}
		}

	btlScalar sideFactor = btlScalar(1.);
	btlScalar fwdFactor = 0.5;

	bool sliding = false;
	{
		for (int wheel =0;wheel <getNumWheels();wheel++)
		{
			btlWheelInfo& wheelInfo = m_wheelInfo[wheel];
			class btlRigidBody* groundObject = (class btlRigidBody*) wheelInfo.m_raycastInfo.m_groundObject;

			btlScalar	rollingFriction = 0.f;

			if (groundObject)
			{
				if (wheelInfo.m_engineForce != 0.f)
				{
					rollingFriction = wheelInfo.m_engineForce* timeStep;
				} else
				{
					btlScalar defaultRollingFrictionImpulse = 0.f;
					btlScalar maxImpulse = wheelInfo.m_brake ? wheelInfo.m_brake : defaultRollingFrictionImpulse;
					btlWheelContactPoint contactPt(m_chassisBody,groundObject,wheelInfo.m_raycastInfo.m_contactPointWS,m_forwardWS[wheel],maxImpulse);
					rollingFriction = btl_calcRollingFriction(contactPt);
				}
			}

			//switch between active rolling (throttle), braking and non-active rolling friction (no throttle/break)
			



			m_forwardImpulse[wheel] = btlScalar(0.);
			m_wheelInfo[wheel].m_skidInfo= btlScalar(1.);

			if (groundObject)
			{
				m_wheelInfo[wheel].m_skidInfo= btlScalar(1.);
				
				btlScalar maximp = wheelInfo.m_wheelsSuspensionForce * timeStep * wheelInfo.m_frictionSlip;
				btlScalar maximpSide = maximp;

				btlScalar maximpSquared = maximp * maximpSide;
			

				m_forwardImpulse[wheel] = rollingFriction;//wheelInfo.m_engineForce* timeStep;

				btlScalar x = (m_forwardImpulse[wheel] ) * fwdFactor;
				btlScalar y = (m_sideImpulse[wheel] ) * sideFactor;
				
				btlScalar impulseSquared = (x*x + y*y);

				if (impulseSquared > maximpSquared)
				{
					sliding = true;
					
					btlScalar factor = maximp / btlSqrt(impulseSquared);
					
					m_wheelInfo[wheel].m_skidInfo *= factor;
				}
			} 

		}
	}

	


		if (sliding)
		{
			for (int wheel = 0;wheel < getNumWheels(); wheel++)
			{
				if (m_sideImpulse[wheel] != btlScalar(0.))
				{
					if (m_wheelInfo[wheel].m_skidInfo< btlScalar(1.))
					{
						m_forwardImpulse[wheel] *=	m_wheelInfo[wheel].m_skidInfo;
						m_sideImpulse[wheel] *= m_wheelInfo[wheel].m_skidInfo;
					}
				}
			}
		}

		// apply the impulses
		{
			for (int wheel = 0;wheel<getNumWheels() ; wheel++)
			{
				btlWheelInfo& wheelInfo = m_wheelInfo[wheel];

				btlVector3 rel_pos = wheelInfo.m_raycastInfo.m_contactPointWS - 
						m_chassisBody->getCenterOfMassPosition();

				if (m_forwardImpulse[wheel] != btlScalar(0.))
				{
					m_chassisBody->applyImpulse(m_forwardWS[wheel]*(m_forwardImpulse[wheel]),rel_pos);
				}
				if (m_sideImpulse[wheel] != btlScalar(0.))
				{
					class btlRigidBody* groundObject = (class btlRigidBody*) m_wheelInfo[wheel].m_raycastInfo.m_groundObject;

					btlVector3 rel_pos2 = wheelInfo.m_raycastInfo.m_contactPointWS - 
						groundObject->getCenterOfMassPosition();

					
					btlVector3 sideImp = m_axle[wheel] * m_sideImpulse[wheel];

#if defined ROLLING_INFLUENCE_FIX // fix. It only worked if car's up was along Y - VT.
					btlVector3 vChassisWorldUp = getRigidBody()->getCenterOfMassTransform().getBasis().getColumn(m_indexUpAxis);
					rel_pos -= vChassisWorldUp * (vChassisWorldUp.dot(rel_pos) * (1.f-wheelInfo.m_rollInfluence));
#else
					rel_pos[m_indexUpAxis] *= wheelInfo.m_rollInfluence;
#endif
					m_chassisBody->applyImpulse(sideImp,rel_pos);

					//apply friction impulse on the ground
					groundObject->applyImpulse(-sideImp,rel_pos2);
				}
			}
		}

	
}



void	btlRaycastVehicle::debugDraw(btlIDebugDraw* debugDrawer)
{

	for (int v=0;v<this->getNumWheels();v++)
	{
		btlVector3 wheelColor(0,1,1);
		if (getWheelInfo(v).m_raycastInfo.m_isInContact)
		{
			wheelColor.setValue(0,0,1);
		} else
		{
			wheelColor.setValue(1,0,1);
		}

		btlVector3 wheelPosWS = getWheelInfo(v).m_worldTransform.getOrigin();

		btlVector3 axle = btlVector3(	
			getWheelInfo(v).m_worldTransform.getBasis()[0][getRightAxis()],
			getWheelInfo(v).m_worldTransform.getBasis()[1][getRightAxis()],
			getWheelInfo(v).m_worldTransform.getBasis()[2][getRightAxis()]);

		//debug wheels (cylinders)
		debugDrawer->drawLine(wheelPosWS,wheelPosWS+axle,wheelColor);
		debugDrawer->drawLine(wheelPosWS,getWheelInfo(v).m_raycastInfo.m_contactPointWS,wheelColor);

	}
}


void* btlDefaultVehicleRaycaster::castRay(const btlVector3& from,const btlVector3& to, btlVehicleRaycasterResult& result)
{
//	RayResultCallback& resultCallback;

	btlCollisionWorld::ClosestRayResultCallback rayCallback(from,to);

	m_dynamicsWorld->rayTest(from, to, rayCallback);

	if (rayCallback.hasHit())
	{
		
		const btlRigidBody* body = btlRigidBody::upcast(rayCallback.m_collisionObject);
        if (body && body->hasContactResponse())
		{
			result.m_hitPointInWorld = rayCallback.m_hitPointWorld;
			result.m_hitNormalInWorld = rayCallback.m_hitNormalWorld;
			result.m_hitNormalInWorld.normalize();
			result.m_distFraction = rayCallback.m_closestHitFraction;
			return (void*)body;
		}
	}
	return 0;
}

