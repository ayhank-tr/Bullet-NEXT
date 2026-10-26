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
#ifndef BT_RAYCASTVEHICLE_H
#define BT_RAYCASTVEHICLE_H

#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"
#include "btVehicleRaycaster.h"
class btlDynamicsWorld;
#include "LinearMath/btAlignedObjectArray.h"
#include "btWheelInfo.h"
#include "BulletDynamics/Dynamics/btActionInterface.h"

class btlVehicleTuning;

///rayCast vehicle, very special constraint that turn a rigidbody into a vehicle.
class btlRaycastVehicle : public btlActionInterface
{

		btlAlignedObjectArray<btlVector3>	m_forwardWS;
		btlAlignedObjectArray<btlVector3>	m_axle;
		btlAlignedObjectArray<btlScalar>	m_forwardImpulse;
		btlAlignedObjectArray<btlScalar>	m_sideImpulse;
	
		///backwards compatibility
		int	m_userConstraintType;
		int	m_userConstraintId;

public:
	class btlVehicleTuning
		{
			public:

			btlVehicleTuning()
				:m_suspensionStiffness(btlScalar(5.88)),
				m_suspensionCompression(btlScalar(0.83)),
				m_suspensionDamping(btlScalar(0.88)),
				m_maxSuspensionTravelCm(btlScalar(500.)),
				m_frictionSlip(btlScalar(10.5)),
				m_maxSuspensionForce(btlScalar(6000.))
			{
			}
			btlScalar	m_suspensionStiffness;
			btlScalar	m_suspensionCompression;
			btlScalar	m_suspensionDamping;
			btlScalar	m_maxSuspensionTravelCm;
			btlScalar	m_frictionSlip;
			btlScalar	m_maxSuspensionForce;

		};
private:

	btlVehicleRaycaster*	m_vehicleRaycaster;
	btlScalar		m_pitchControl;
	btlScalar	m_steeringValue; 
	btlScalar m_currentVehicleSpeedKmHour;

	btlRigidBody* m_chassisBody;

	int m_indexRightAxis;
	int m_indexUpAxis;
	int	m_indexForwardAxis;

	void defaultInit(const btlVehicleTuning& tuning);

public:

	//constructor to create a car from an existing rigidbody
	btlRaycastVehicle(const btlVehicleTuning& tuning,btlRigidBody* chassis,	btlVehicleRaycaster* raycaster );

	virtual ~btlRaycastVehicle() ;


	///btActionInterface interface
	virtual void updateAction( btlCollisionWorld* collisionWorld, btlScalar step)
	{
        (void) collisionWorld;
		updateVehicle(step);
	}
	

	///btActionInterface interface
	void	debugDraw(btlIDebugDraw* debugDrawer);
			
	const btlTransform& getChassisWorldTransform() const;
	
	btlScalar rayCast(btlWheelInfo& wheel);

	virtual void updateVehicle(btlScalar step);
	
	
	void resetSuspension();

	btlScalar	getSteeringValue(int wheel) const;

	void	setSteeringValue(btlScalar steering,int wheel);


	void	applyEngineForce(btlScalar force, int wheel);

	const btlTransform&	getWheelTransformWS( int wheelIndex ) const;

	void	updateWheelTransform( int wheelIndex, bool interpolatedTransform = true );
	
//	void	setRaycastWheelInfo( int wheelIndex , bool isInContact, const btVector3& hitPoint, const btVector3& hitNormal,btScalar depth);

	btlWheelInfo&	addWheel( const btlVector3& connectionPointCS0, const btlVector3& wheelDirectionCS0,const btlVector3& wheelAxleCS,btlScalar suspensionRestLength,btlScalar wheelRadius,const btlVehicleTuning& tuning, bool isFrontWheel);

	inline int		getNumWheels() const {
		return int (m_wheelInfo.size());
	}
	
	btlAlignedObjectArray<btlWheelInfo>	m_wheelInfo;


	const btlWheelInfo&	getWheelInfo(int index) const;

	btlWheelInfo&	getWheelInfo(int index);

	void	updateWheelTransformsWS(btlWheelInfo& wheel , bool interpolatedTransform = true);

	
	void setBrake(btlScalar brake,int wheelIndex);

	void	setPitchControl(btlScalar pitch)
	{
		m_pitchControl = pitch;
	}
	
	void	updateSuspension(btlScalar deltaTime);

	virtual void	updateFriction(btlScalar	timeStep);



	inline btlRigidBody* getRigidBody()
	{
		return m_chassisBody;
	}

	const btlRigidBody* getRigidBody() const
	{
		return m_chassisBody;
	}

	inline int	getRightAxis() const
	{
		return m_indexRightAxis;
	}
	inline int getUpAxis() const
	{
		return m_indexUpAxis;
	}

	inline int getForwardAxis() const
	{
		return m_indexForwardAxis;
	}

	
	///Worldspace forward vector
	btlVector3 getForwardVector() const
	{
		const btlTransform& chassisTrans = getChassisWorldTransform(); 

		btlVector3 forwardW ( 
			  chassisTrans.getBasis()[0][m_indexForwardAxis], 
			  chassisTrans.getBasis()[1][m_indexForwardAxis], 
			  chassisTrans.getBasis()[2][m_indexForwardAxis]); 

		return forwardW;
	}

	///Velocity of vehicle (positive if velocity vector has same direction as foward vector)
	btlScalar	getCurrentSpeedKmHour() const
	{
		return m_currentVehicleSpeedKmHour;
	}

	virtual void	setCoordinateSystem(int rightIndex,int upIndex,int forwardIndex)
	{
		m_indexRightAxis = rightIndex;
		m_indexUpAxis = upIndex;
		m_indexForwardAxis = forwardIndex;
	}


	///backwards compatibility
	int getUserConstraintType() const
	{
		return m_userConstraintType ;
	}

	void	setUserConstraintType(int userConstraintType)
	{
		m_userConstraintType = userConstraintType;
	};

	void	setUserConstraintId(int uid)
	{
		m_userConstraintId = uid;
	}

	int getUserConstraintId() const
	{
		return m_userConstraintId;
	}

};

class btlDefaultVehicleRaycaster : public btlVehicleRaycaster
{
	btlDynamicsWorld*	m_dynamicsWorld;
public:
	btlDefaultVehicleRaycaster(btlDynamicsWorld* world)
		:m_dynamicsWorld(world)
	{
	}

	virtual void* castRay(const btlVector3& from,const btlVector3& to, btlVehicleRaycasterResult& result);

};


#endif //BT_RAYCASTVEHICLE_H

