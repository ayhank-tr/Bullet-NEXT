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
#ifndef BT_WHEEL_INFO_H
#define BT_WHEEL_INFO_H

#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"

class btlRigidBody;

struct btlWheelInfoConstructionInfo
{
	btlVector3	m_chassisConnectionCS;
	btlVector3	m_wheelDirectionCS;
	btlVector3	m_wheelAxleCS;
	btlScalar	m_suspensionRestLength;
	btlScalar	m_maxSuspensionTravelCm;
	btlScalar	m_wheelRadius;
	
	btlScalar		m_suspensionStiffness;
	btlScalar		m_wheelsDampingCompression;
	btlScalar		m_wheelsDampingRelaxation;
	btlScalar		m_frictionSlip;
	btlScalar		m_maxSuspensionForce;
	bool m_bIsFrontWheel;
	
};

/// btWheelInfo contains information per wheel about friction and suspension.
struct btlWheelInfo
{
	struct RaycastInfo
	{
		//set by raycaster
		btlVector3	m_contactNormalWS;//contactnormal
		btlVector3	m_contactPointWS;//raycast hitpoint
		btlScalar	m_suspensionLength;
		btlVector3	m_hardPointWS;//raycast starting point
		btlVector3	m_wheelDirectionWS; //direction in worldspace
		btlVector3	m_wheelAxleWS; // axle in worldspace
		bool		m_isInContact;
		void*		m_groundObject; //could be general void* ptr
	};

	RaycastInfo	m_raycastInfo;

	btlTransform	m_worldTransform;
	
	btlVector3	m_chassisConnectionPointCS; //const
	btlVector3	m_wheelDirectionCS;//const
	btlVector3	m_wheelAxleCS; // const or modified by steering
	btlScalar	m_suspensionRestLength1;//const
	btlScalar	m_maxSuspensionTravelCm;
	btlScalar getSuspensionRestLength() const;
	btlScalar	m_wheelsRadius;//const
	btlScalar	m_suspensionStiffness;//const
	btlScalar	m_wheelsDampingCompression;//const
	btlScalar	m_wheelsDampingRelaxation;//const
	btlScalar	m_frictionSlip;
	btlScalar	m_steering;
	btlScalar	m_rotation;
	btlScalar	m_deltaRotation;
	btlScalar	m_rollInfluence;
	btlScalar	m_maxSuspensionForce;

	btlScalar	m_engineForce;

	btlScalar	m_brake;
	
	bool m_bIsFrontWheel;
	
	void*		m_clientInfo;//can be used to store pointer to sync transforms...

	btlWheelInfo(btlWheelInfoConstructionInfo& ci)

	{

		m_suspensionRestLength1 = ci.m_suspensionRestLength;
		m_maxSuspensionTravelCm = ci.m_maxSuspensionTravelCm;

		m_wheelsRadius = ci.m_wheelRadius;
		m_suspensionStiffness = ci.m_suspensionStiffness;
		m_wheelsDampingCompression = ci.m_wheelsDampingCompression;
		m_wheelsDampingRelaxation = ci.m_wheelsDampingRelaxation;
		m_chassisConnectionPointCS = ci.m_chassisConnectionCS;
		m_wheelDirectionCS = ci.m_wheelDirectionCS;
		m_wheelAxleCS = ci.m_wheelAxleCS;
		m_frictionSlip = ci.m_frictionSlip;
		m_steering = btlScalar(0.);
		m_engineForce = btlScalar(0.);
		m_rotation = btlScalar(0.);
		m_deltaRotation = btlScalar(0.);
		m_brake = btlScalar(0.);
		m_rollInfluence = btlScalar(0.1);
		m_bIsFrontWheel = ci.m_bIsFrontWheel;
		m_maxSuspensionForce = ci.m_maxSuspensionForce;

	}

	void	updateWheel(const btlRigidBody& chassis,RaycastInfo& raycastInfo);

	btlScalar	m_clippedInvContactDotSuspension;
	btlScalar	m_suspensionRelativeVelocity;
	//calculated by suspension
	btlScalar	m_wheelsSuspensionForce;
	btlScalar	m_skidInfo;

};

#endif //BT_WHEEL_INFO_H

