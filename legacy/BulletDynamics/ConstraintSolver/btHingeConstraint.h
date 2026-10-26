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

/* Hinge Constraint by Dirk Gregorius. Limits added by Marcus Hennix at Starbreeze Studios */

#ifndef BT_HINGECONSTRAINT_H
#define BT_HINGECONSTRAINT_H

#define _BT_USE_CENTER_LIMIT_ 1


#include "LinearMath/btVector3.h"
#include "btJacobianEntry.h"
#include "btTypedConstraint.h"

class btlRigidBody;

#ifdef BT_USE_DOUBLE_PRECISION
#define btlHingeConstraintData	btlHingeConstraintDoubleData2 //rename to 2 for backwards compatibility, so we can still load the 'btHingeConstraintDoubleData' version
#define btlHingeConstraintDataName	"btHingeConstraintDoubleData2" 
#else
#define btlHingeConstraintData	btlHingeConstraintFloatData
#define btlHingeConstraintDataName	"btHingeConstraintFloatData"
#endif //BT_USE_DOUBLE_PRECISION



enum btlHingeFlags
{
	BT_HINGE_FLAGS_CFM_STOP = 1,
	BT_HINGE_FLAGS_ERP_STOP = 2,
	BT_HINGE_FLAGS_CFM_NORM = 4,
	BT_HINGE_FLAGS_ERP_NORM = 8
};


/// hinge constraint between two rigidbodies each with a pivotpoint that descibes the axis location in local space
/// axis defines the orientation of the hinge axis
ATTRIBUTE_ALIGNED16(class) btlHingeConstraint : public btlTypedConstraint
{
#ifdef IN_PARALLELL_SOLVER
public:
#endif
	btlJacobianEntry	m_jac[3]; //3 orthogonal linear constraints
	btlJacobianEntry	m_jacAng[3]; //2 orthogonal angular constraints+ 1 for limit/motor

	btlTransform m_rbAFrame; // constraint axii. Assumes z is hinge axis.
	btlTransform m_rbBFrame;

	btlScalar	m_motorTargetVelocity;
	btlScalar	m_maxMotorImpulse;


#ifdef	_BT_USE_CENTER_LIMIT_
	btlAngularLimit	m_limit;
#else
	btlScalar	m_lowerLimit;	
	btlScalar	m_upperLimit;	
	btlScalar	m_limitSign;
	btlScalar	m_correction;

	btlScalar	m_limitSoftness; 
	btlScalar	m_biasFactor; 
	btlScalar	m_relaxationFactor; 

	bool		m_solveLimit;
#endif

	btlScalar	m_kHinge;


	btlScalar	m_accLimitImpulse;
	btlScalar	m_hingeAngle;
	btlScalar	m_referenceSign;

	bool		m_angularOnly;
	bool		m_enableAngularMotor;
	bool		m_useSolveConstraintObsolete;
	bool		m_useOffsetForConstraintFrame;
	bool		m_useReferenceFrameA;

	btlScalar	m_accMotorImpulse;

	int			m_flags;
	btlScalar	m_normalCFM;
	btlScalar	m_normalERP;
	btlScalar	m_stopCFM;
	btlScalar	m_stopERP;

	
public:

	BT_DECLARE_ALIGNED_ALLOCATOR();
	
	btlHingeConstraint(btlRigidBody& rbA,btlRigidBody& rbB, const btlVector3& pivotInA,const btlVector3& pivotInB, const btlVector3& axisInA,const btlVector3& axisInB, bool useReferenceFrameA = false);

	btlHingeConstraint(btlRigidBody& rbA,const btlVector3& pivotInA,const btlVector3& axisInA, bool useReferenceFrameA = false);
	
	btlHingeConstraint(btlRigidBody& rbA,btlRigidBody& rbB, const btlTransform& rbAFrame, const btlTransform& rbBFrame, bool useReferenceFrameA = false);

	btlHingeConstraint(btlRigidBody& rbA,const btlTransform& rbAFrame, bool useReferenceFrameA = false);


	virtual void	buildJacobian();

	virtual void getInfo1 (btlConstraintInfo1* info);

	void getInfo1NonVirtual(btlConstraintInfo1* info);

	virtual void getInfo2 (btlConstraintInfo2* info);

	void	getInfo2NonVirtual(btlConstraintInfo2* info,const btlTransform& transA,const btlTransform& transB,const btlVector3& angVelA,const btlVector3& angVelB);

	void	getInfo2Internal(btlConstraintInfo2* info,const btlTransform& transA,const btlTransform& transB,const btlVector3& angVelA,const btlVector3& angVelB);
	void	getInfo2InternalUsingFrameOffset(btlConstraintInfo2* info,const btlTransform& transA,const btlTransform& transB,const btlVector3& angVelA,const btlVector3& angVelB);
		

	void	updateRHS(btlScalar	timeStep);

	const btlRigidBody& getRigidBodyA() const
	{
		return m_rbA;
	}
	const btlRigidBody& getRigidBodyB() const
	{
		return m_rbB;
	}

	btlRigidBody& getRigidBodyA()	
	{		
		return m_rbA;	
	}	

	btlRigidBody& getRigidBodyB()	
	{		
		return m_rbB;	
	}

	btlTransform& getFrameOffsetA()
	{
	return m_rbAFrame;
	}

	btlTransform& getFrameOffsetB()
	{
		return m_rbBFrame;
	}

	void setFrames(const btlTransform& frameA, const btlTransform& frameB);
	
	void	setAngularOnly(bool angularOnly)
	{
		m_angularOnly = angularOnly;
	}

	void	enableAngularMotor(bool enableMotor,btlScalar targetVelocity,btlScalar maxMotorImpulse)
	{
		m_enableAngularMotor  = enableMotor;
		m_motorTargetVelocity = targetVelocity;
		m_maxMotorImpulse = maxMotorImpulse;
	}

	// extra motor API, including ability to set a target rotation (as opposed to angular velocity)
	// note: setMotorTarget sets angular velocity under the hood, so you must call it every tick to
	//       maintain a given angular target.
	void enableMotor(bool enableMotor) 	{ m_enableAngularMotor = enableMotor; }
	void setMaxMotorImpulse(btlScalar maxMotorImpulse) { m_maxMotorImpulse = maxMotorImpulse; }
	void setMotorTargetVelocity(btlScalar motorTargetVelocity) { m_motorTargetVelocity = motorTargetVelocity; }
	void setMotorTarget(const btlQuaternion& qAinB, btlScalar dt); // qAinB is rotation of body A wrt body B.
	void setMotorTarget(btlScalar targetAngle, btlScalar dt);


	void	setLimit(btlScalar low,btlScalar high,btlScalar _softness = 0.9f, btlScalar _biasFactor = 0.3f, btlScalar _relaxationFactor = 1.0f)
	{
#ifdef	_BT_USE_CENTER_LIMIT_
		m_limit.set(low, high, _softness, _biasFactor, _relaxationFactor);
#else
		m_lowerLimit = btlNormalizeAngle(low);
		m_upperLimit = btlNormalizeAngle(high);
		m_limitSoftness =  _softness;
		m_biasFactor = _biasFactor;
		m_relaxationFactor = _relaxationFactor;
#endif
	}
	
	btlScalar getLimitSoftness() const
	{
#ifdef	_BT_USE_CENTER_LIMIT_
		return m_limit.getSoftness();
#else
		return m_limitSoftness;
#endif
	}

	btlScalar getLimitBiasFactor() const
	{
#ifdef	_BT_USE_CENTER_LIMIT_
		return m_limit.getBiasFactor();
#else
		return m_biasFactor;
#endif
	}

	btlScalar getLimitRelaxationFactor() const
	{
#ifdef	_BT_USE_CENTER_LIMIT_
		return m_limit.getRelaxationFactor();
#else
		return m_relaxationFactor;
#endif
	}

	void	setAxis(btlVector3& axisInA)
	{
		btlVector3 rbAxisA1, rbAxisA2;
		btlPlaneSpace1(axisInA, rbAxisA1, rbAxisA2);
		btlVector3 pivotInA = m_rbAFrame.getOrigin();
//		m_rbAFrame.getOrigin() = pivotInA;
		m_rbAFrame.getBasis().setValue( rbAxisA1.getX(),rbAxisA2.getX(),axisInA.getX(),
										rbAxisA1.getY(),rbAxisA2.getY(),axisInA.getY(),
										rbAxisA1.getZ(),rbAxisA2.getZ(),axisInA.getZ() );

		btlVector3 axisInB = m_rbA.getCenterOfMassTransform().getBasis() * axisInA;

		btlQuaternion rotationArc = shortestArcQuat(axisInA,axisInB);
		btlVector3 rbAxisB1 =  btl_quatRotate(rotationArc,rbAxisA1);
		btlVector3 rbAxisB2 = axisInB.cross(rbAxisB1);

		m_rbBFrame.getOrigin() = m_rbB.getCenterOfMassTransform().inverse()(m_rbA.getCenterOfMassTransform()(pivotInA));

		m_rbBFrame.getBasis().setValue( rbAxisB1.getX(),rbAxisB2.getX(),axisInB.getX(),
										rbAxisB1.getY(),rbAxisB2.getY(),axisInB.getY(),
										rbAxisB1.getZ(),rbAxisB2.getZ(),axisInB.getZ() );
		m_rbBFrame.getBasis() = m_rbB.getCenterOfMassTransform().getBasis().inverse() * m_rbBFrame.getBasis();

	}

    bool hasLimit() const {
#ifdef  _BT_USE_CENTER_LIMIT_
        return m_limit.getHalfRange() > 0;
#else
        return m_lowerLimit <= m_upperLimit;
#endif
    }

	btlScalar	getLowerLimit() const
	{
#ifdef	_BT_USE_CENTER_LIMIT_
	return m_limit.getLow();
#else
	return m_lowerLimit;
#endif
	}

	btlScalar	getUpperLimit() const
	{
#ifdef	_BT_USE_CENTER_LIMIT_
	return m_limit.getHigh();
#else		
	return m_upperLimit;
#endif
	}


	///The getHingeAngle gives the hinge angle in range [-PI,PI]
	btlScalar getHingeAngle();

	btlScalar getHingeAngle(const btlTransform& transA,const btlTransform& transB);

	void testLimit(const btlTransform& transA,const btlTransform& transB);


	const btlTransform& getAFrame() const { return m_rbAFrame; };	
	const btlTransform& getBFrame() const { return m_rbBFrame; };

	btlTransform& getAFrame() { return m_rbAFrame; };	
	btlTransform& getBFrame() { return m_rbBFrame; };

	inline int getSolveLimit()
	{
#ifdef	_BT_USE_CENTER_LIMIT_
	return m_limit.isLimit();
#else
	return m_solveLimit;
#endif
	}

	inline btlScalar getLimitSign()
	{
#ifdef	_BT_USE_CENTER_LIMIT_
	return m_limit.getSign();
#else
		return m_limitSign;
#endif
	}

	inline bool getAngularOnly() 
	{ 
		return m_angularOnly; 
	}
	inline bool getEnableAngularMotor() 
	{ 
		return m_enableAngularMotor; 
	}
	inline btlScalar getMotorTargetVelosity() 
	{ 
		return m_motorTargetVelocity; 
	}
	inline btlScalar getMaxMotorImpulse() 
	{ 
		return m_maxMotorImpulse; 
	}
	// access for UseFrameOffset
	bool getUseFrameOffset() { return m_useOffsetForConstraintFrame; }
	void setUseFrameOffset(bool frameOffsetOnOff) { m_useOffsetForConstraintFrame = frameOffsetOnOff; }
	// access for UseReferenceFrameA
	bool getUseReferenceFrameA() const { return m_useReferenceFrameA; }
	void setUseReferenceFrameA(bool useReferenceFrameA) { m_useReferenceFrameA = useReferenceFrameA; }

	///override the default global value of a parameter (such as ERP or CFM), optionally provide the axis (0..5). 
	///If no axis is provided, it uses the default axis for this constraint.
	virtual	void	setParam(int num, btlScalar value, int axis = -1);
	///return the local value of parameter
	virtual	btlScalar getParam(int num, int axis = -1) const;
	
	virtual	int getFlags() const
	{
  	    return m_flags;
	}

	virtual	int	calculateSerializeBufferSize() const;

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual	const char*	serialize(void* dataBuffer, btlSerializer* serializer) const;


};


//only for backward compatibility
#ifdef BT_BACKWARDS_COMPATIBLE_SERIALIZATION
///this structure is not used, except for loading pre-2.82 .bullet files
struct	btlHingeConstraintDoubleData
{
	btlTypedConstraintData	m_typeConstraintData;
	btlTransformDoubleData m_rbAFrame; // constraint axii. Assumes z is hinge axis.
	btlTransformDoubleData m_rbBFrame;
	int			m_useReferenceFrameA;
	int			m_angularOnly;
	int			m_enableAngularMotor;
	float	m_motorTargetVelocity;
	float	m_maxMotorImpulse;

	float	m_lowerLimit;
	float	m_upperLimit;
	float	m_limitSoftness;
	float	m_biasFactor;
	float	m_relaxationFactor;

};
#endif //BT_BACKWARDS_COMPATIBLE_SERIALIZATION

///The getAccumulatedHingeAngle returns the accumulated hinge angle, taking rotation across the -PI/PI boundary into account
ATTRIBUTE_ALIGNED16(class) btlHingeAccumulatedAngleConstraint : public btlHingeConstraint
{
protected:
	btlScalar	m_accumulatedAngle;
public:

	BT_DECLARE_ALIGNED_ALLOCATOR();
	
	btlHingeAccumulatedAngleConstraint(btlRigidBody& rbA,btlRigidBody& rbB, const btlVector3& pivotInA,const btlVector3& pivotInB, const btlVector3& axisInA,const btlVector3& axisInB, bool useReferenceFrameA = false)
	:btlHingeConstraint(rbA,rbB,pivotInA,pivotInB, axisInA,axisInB, useReferenceFrameA )
	{
		m_accumulatedAngle=getHingeAngle();
	}

	btlHingeAccumulatedAngleConstraint(btlRigidBody& rbA,const btlVector3& pivotInA,const btlVector3& axisInA, bool useReferenceFrameA = false)
	:btlHingeConstraint(rbA,pivotInA,axisInA, useReferenceFrameA)
	{
		m_accumulatedAngle=getHingeAngle();
	}
	
	btlHingeAccumulatedAngleConstraint(btlRigidBody& rbA,btlRigidBody& rbB, const btlTransform& rbAFrame, const btlTransform& rbBFrame, bool useReferenceFrameA = false)
	:btlHingeConstraint(rbA,rbB, rbAFrame, rbBFrame, useReferenceFrameA )
	{
		m_accumulatedAngle=getHingeAngle();
	}

	btlHingeAccumulatedAngleConstraint(btlRigidBody& rbA,const btlTransform& rbAFrame, bool useReferenceFrameA = false)
	:btlHingeConstraint(rbA,rbAFrame, useReferenceFrameA )
	{
		m_accumulatedAngle=getHingeAngle();
	}
	btlScalar getAccumulatedHingeAngle();
	void	setAccumulatedHingeAngle(btlScalar accAngle);
	virtual void getInfo1 (btlConstraintInfo1* info);

};

struct	btlHingeConstraintFloatData
{
	btlTypedConstraintData	m_typeConstraintData;
	btlTransformFloatData m_rbAFrame; // constraint axii. Assumes z is hinge axis.
	btlTransformFloatData m_rbBFrame;
	int			m_useReferenceFrameA;
	int			m_angularOnly;
	
	int			m_enableAngularMotor;
	float	m_motorTargetVelocity;
	float	m_maxMotorImpulse;

	float	m_lowerLimit;
	float	m_upperLimit;
	float	m_limitSoftness;
	float	m_biasFactor;
	float	m_relaxationFactor;

};



///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct	btlHingeConstraintDoubleData2
{
	btlTypedConstraintDoubleData	m_typeConstraintData;
	btlTransformDoubleData m_rbAFrame; // constraint axii. Assumes z is hinge axis.
	btlTransformDoubleData m_rbBFrame;
	int			m_useReferenceFrameA;
	int			m_angularOnly;
	int			m_enableAngularMotor;
	double		m_motorTargetVelocity;
	double		m_maxMotorImpulse;

	double		m_lowerLimit;
	double		m_upperLimit;
	double		m_limitSoftness;
	double		m_biasFactor;
	double		m_relaxationFactor;
	char	m_padding1[4];

};




SIMD_FORCE_INLINE	int	btlHingeConstraint::calculateSerializeBufferSize() const
{
	return sizeof(btlHingeConstraintData);
}

	///fills the dataBuffer and returns the struct name (and 0 on failure)
SIMD_FORCE_INLINE	const char*	btlHingeConstraint::serialize(void* dataBuffer, btlSerializer* serializer) const
{
	btlHingeConstraintData* hingeData = (btlHingeConstraintData*)dataBuffer;
	btlTypedConstraint::serialize(&hingeData->m_typeConstraintData,serializer);

	m_rbAFrame.serialize(hingeData->m_rbAFrame);
	m_rbBFrame.serialize(hingeData->m_rbBFrame);

	hingeData->m_angularOnly = m_angularOnly;
	hingeData->m_enableAngularMotor = m_enableAngularMotor;
	hingeData->m_maxMotorImpulse = float(m_maxMotorImpulse);
	hingeData->m_motorTargetVelocity = float(m_motorTargetVelocity);
	hingeData->m_useReferenceFrameA = m_useReferenceFrameA;
#ifdef	_BT_USE_CENTER_LIMIT_
	hingeData->m_lowerLimit = float(m_limit.getLow());
	hingeData->m_upperLimit = float(m_limit.getHigh());
	hingeData->m_limitSoftness = float(m_limit.getSoftness());
	hingeData->m_biasFactor = float(m_limit.getBiasFactor());
	hingeData->m_relaxationFactor = float(m_limit.getRelaxationFactor());
#else
	hingeData->m_lowerLimit = float(m_lowerLimit);
	hingeData->m_upperLimit = float(m_upperLimit);
	hingeData->m_limitSoftness = float(m_limitSoftness);
	hingeData->m_biasFactor = float(m_biasFactor);
	hingeData->m_relaxationFactor = float(m_relaxationFactor);
#endif

	return btlHingeConstraintDataName;
}

#endif //BT_HINGECONSTRAINT_H
