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

/*
2014 May: btGeneric6DofSpring2Constraint is created from the original (2.82.2712) btGeneric6DofConstraint by Gabor Puhr and Tamas Umenhoffer
Pros:
- Much more accurate and stable in a lot of situation. (Especially when a sleeping chain of RBs connected with 6dof2 is pulled)
- Stable and accurate spring with minimal energy loss that works with all of the solvers. (latter is not true for the original 6dof spring)
- Servo motor functionality
- Much more accurate bouncing. 0 really means zero bouncing (not true for the original 6odf) and there is only a minimal energy loss when the value is 1 (because of the solvers' precision)
- Rotation order for the Euler system can be set. (One axis' freedom is still limited to pi/2)

Cons:
- It is slower than the original 6dof. There is no exact ratio, but half speed is a good estimation.
- At bouncing the correct velocity is calculated, but not the correct position. (it is because of the solver can correct position or velocity, but not both.)
*/

/// 2009 March: btGeneric6DofConstraint refactored by Roman Ponomarev
/// Added support for generic constraint solver through getInfo1/getInfo2 methods

/*
2007-09-09
btGeneric6DofConstraint Refactored by Francisco Le?n
email: projectileman@yahoo.com
http://gimpact.sf.net
*/


#ifndef BT_GENERIC_6DOF_CONSTRAINT2_H
#define BT_GENERIC_6DOF_CONSTRAINT2_H

#include "LinearMath/btVector3.h"
#include "btJacobianEntry.h"
#include "btTypedConstraint.h"

class btlRigidBody;


#ifdef BT_USE_DOUBLE_PRECISION
#define btlGeneric6DofSpring2ConstraintData2		btlGeneric6DofSpring2ConstraintDoubleData2
#define btlGeneric6DofSpring2ConstraintDataName	"btGeneric6DofSpring2ConstraintDoubleData2"
#else
#define btlGeneric6DofSpring2ConstraintData2		btlGeneric6DofSpring2ConstraintData
#define btlGeneric6DofSpring2ConstraintDataName	"btGeneric6DofSpring2ConstraintData"
#endif //BT_USE_DOUBLE_PRECISION

enum RotateOrder
{
	RO_XYZ=0,
	RO_XZY,
	RO_YXZ,
	RO_YZX,
	RO_ZXY,
	RO_ZYX
};

class btlRotationalLimitMotor2
{
public:
// upper < lower means free
// upper == lower means locked
// upper > lower means limited
	btlScalar m_loLimit;
	btlScalar m_hiLimit;
	btlScalar m_bounce;
	btlScalar m_stopERP;
	btlScalar m_stopCFM;
	btlScalar m_motorERP;
	btlScalar m_motorCFM;
	bool     m_enableMotor;
	btlScalar m_targetVelocity;
	btlScalar m_maxMotorForce;
	bool     m_servoMotor;
	btlScalar m_servoTarget;
	bool     m_enableSpring;
	btlScalar m_springStiffness;
	bool     m_springStiffnessLimited;
	btlScalar m_springDamping;
	bool     m_springDampingLimited;
	btlScalar m_equilibriumPoint;

	btlScalar m_currentLimitError;
	btlScalar m_currentLimitErrorHi;
	btlScalar m_currentPosition;
	int      m_currentLimit;

	btlRotationalLimitMotor2()
	{
		m_loLimit                = 1.0f;
		m_hiLimit                = -1.0f;
		m_bounce                 = 0.0f;
		m_stopERP                = 0.2f;
		m_stopCFM                = 0.f;
		m_motorERP               = 0.9f;
		m_motorCFM               = 0.f;
		m_enableMotor            = false;
		m_targetVelocity         = 0;
		m_maxMotorForce          = 0.1f;
		m_servoMotor             = false;
		m_servoTarget            = 0;
		m_enableSpring           = false;
		m_springStiffness        = 0;
		m_springStiffnessLimited = false;
		m_springDamping          = 0;
		m_springDampingLimited   = false;
		m_equilibriumPoint       = 0;

		m_currentLimitError   = 0;
		m_currentLimitErrorHi = 0;
		m_currentPosition     = 0;
		m_currentLimit        = 0;
	}

	btlRotationalLimitMotor2(const btlRotationalLimitMotor2 & limot)
	{
		m_loLimit                = limot.m_loLimit;
		m_hiLimit                = limot.m_hiLimit;
		m_bounce                 = limot.m_bounce;
		m_stopERP                = limot.m_stopERP;
		m_stopCFM                = limot.m_stopCFM;
		m_motorERP               = limot.m_motorERP;
		m_motorCFM               = limot.m_motorCFM;
		m_enableMotor            = limot.m_enableMotor;
		m_targetVelocity         = limot.m_targetVelocity;
		m_maxMotorForce          = limot.m_maxMotorForce;
		m_servoMotor             = limot.m_servoMotor;
		m_servoTarget            = limot.m_servoTarget;
		m_enableSpring           = limot.m_enableSpring;
		m_springStiffness        = limot.m_springStiffness;
		m_springStiffnessLimited = limot.m_springStiffnessLimited;
		m_springDamping          = limot.m_springDamping;
		m_springDampingLimited   = limot.m_springDampingLimited;
		m_equilibriumPoint       = limot.m_equilibriumPoint;

		m_currentLimitError   = limot.m_currentLimitError;
		m_currentLimitErrorHi = limot.m_currentLimitErrorHi;
		m_currentPosition     = limot.m_currentPosition;
		m_currentLimit        = limot.m_currentLimit;
	}


	bool isLimited()
	{
		if(m_loLimit > m_hiLimit) return false;
		return true;
	}

	void testLimitValue(btlScalar test_value);
};



class btlTranslationalLimitMotor2
{
public:
// upper < lower means free
// upper == lower means locked
// upper > lower means limited
	btlVector3 m_lowerLimit;
	btlVector3 m_upperLimit;
	btlVector3 m_bounce;
	btlVector3 m_stopERP;
	btlVector3 m_stopCFM;
	btlVector3 m_motorERP;
	btlVector3 m_motorCFM;
	bool      m_enableMotor[3];
	bool      m_servoMotor[3];
	bool      m_enableSpring[3];
	btlVector3 m_servoTarget;
	btlVector3 m_springStiffness;
	bool      m_springStiffnessLimited[3];
	btlVector3 m_springDamping;
	bool      m_springDampingLimited[3];
	btlVector3 m_equilibriumPoint;
	btlVector3 m_targetVelocity;
	btlVector3 m_maxMotorForce;

	btlVector3 m_currentLimitError;
	btlVector3 m_currentLimitErrorHi;
	btlVector3 m_currentLinearDiff;
	int       m_currentLimit[3];

	btlTranslationalLimitMotor2()
	{
		m_lowerLimit         .setValue(0.f , 0.f , 0.f );
		m_upperLimit         .setValue(0.f , 0.f , 0.f );
		m_bounce             .setValue(0.f , 0.f , 0.f );
		m_stopERP            .setValue(0.2f, 0.2f, 0.2f);
		m_stopCFM            .setValue(0.f , 0.f , 0.f );
		m_motorERP           .setValue(0.9f, 0.9f, 0.9f);
		m_motorCFM           .setValue(0.f , 0.f , 0.f );

		m_currentLimitError  .setValue(0.f , 0.f , 0.f );
		m_currentLimitErrorHi.setValue(0.f , 0.f , 0.f );
		m_currentLinearDiff  .setValue(0.f , 0.f , 0.f );

		for(int i=0; i < 3; i++) 
		{
			m_enableMotor[i]            = false;
			m_servoMotor[i]             = false;
			m_enableSpring[i]           = false;
			m_servoTarget[i]            = btlScalar(0.f);
			m_springStiffness[i]        = btlScalar(0.f);
			m_springStiffnessLimited[i] = false;
			m_springDamping[i]          = btlScalar(0.f);
			m_springDampingLimited[i]   = false;
			m_equilibriumPoint[i]       = btlScalar(0.f);
			m_targetVelocity[i]         = btlScalar(0.f);
			m_maxMotorForce[i]          = btlScalar(0.f);
			
			m_currentLimit[i]     = 0;
		}
	}

	btlTranslationalLimitMotor2(const btlTranslationalLimitMotor2 & other )
	{
		m_lowerLimit          = other.m_lowerLimit;
		m_upperLimit          = other.m_upperLimit;
		m_bounce              = other.m_bounce;
		m_stopERP             = other.m_stopERP;
		m_stopCFM             = other.m_stopCFM;
		m_motorERP            = other.m_motorERP;
		m_motorCFM            = other.m_motorCFM;
		
		m_currentLimitError   = other.m_currentLimitError;
		m_currentLimitErrorHi = other.m_currentLimitErrorHi;
		m_currentLinearDiff   = other.m_currentLinearDiff;

		for(int i=0; i < 3; i++) 
		{
			m_enableMotor[i]            = other.m_enableMotor[i];
			m_servoMotor[i]             = other.m_servoMotor[i];
			m_enableSpring[i]           = other.m_enableSpring[i];
			m_servoTarget[i]            = other.m_servoTarget[i];
			m_springStiffness[i]        = other.m_springStiffness[i];
			m_springStiffnessLimited[i] = other.m_springStiffnessLimited[i];
			m_springDamping[i]          = other.m_springDamping[i];
			m_springDampingLimited[i]   = other.m_springDampingLimited[i];
			m_equilibriumPoint[i]       = other.m_equilibriumPoint[i];
			m_targetVelocity[i]         = other.m_targetVelocity[i];
			m_maxMotorForce[i]          = other.m_maxMotorForce[i];

			m_currentLimit[i]     = other.m_currentLimit[i];
		}
	}

	inline bool isLimited(int limitIndex)
	{
		return (m_upperLimit[limitIndex] >= m_lowerLimit[limitIndex]);
	}

	void testLimitValue(int limitIndex, btlScalar test_value);
};

enum btl6DofFlags2
{
	BT_6DOF_FLAGS_CFM_STOP2 = 1,
	BT_6DOF_FLAGS_ERP_STOP2 = 2,
	BT_6DOF_FLAGS_CFM_MOTO2 = 4,
	BT_6DOF_FLAGS_ERP_MOTO2 = 8
};
#define BT_6DOF_FLAGS_AXIS_SHIFT2 4 // bits per axis


ATTRIBUTE_ALIGNED16(class) btlGeneric6DofSpring2Constraint : public btlTypedConstraint
{
protected:

	btlTransform m_frameInA;
	btlTransform m_frameInB;

	btlJacobianEntry m_jacLinear[3];
	btlJacobianEntry m_jacAng[3];

	btlTranslationalLimitMotor2 m_linearLimits;
	btlRotationalLimitMotor2 m_angularLimits[3];

	RotateOrder m_rotateOrder;

protected:

	btlTransform  m_calculatedTransformA;
	btlTransform  m_calculatedTransformB;
	btlVector3    m_calculatedAxisAngleDiff;
	btlVector3    m_calculatedAxis[3];
	btlVector3    m_calculatedLinearDiff;
	btlScalar     m_factA;
	btlScalar     m_factB;
	bool         m_hasStaticBody;
	int          m_flags;

	btlGeneric6DofSpring2Constraint&	operator=(btlGeneric6DofSpring2Constraint&)
	{
		btlAssert(0);
		return *this;
	}

	int setAngularLimits(btlConstraintInfo2 *info, int row_offset,const btlTransform& transA,const btlTransform& transB,const btlVector3& linVelA,const btlVector3& linVelB,const btlVector3& angVelA,const btlVector3& angVelB);
	int setLinearLimits(btlConstraintInfo2 *info, int row, const btlTransform& transA,const btlTransform& transB,const btlVector3& linVelA,const btlVector3& linVelB,const btlVector3& angVelA,const btlVector3& angVelB);

	void calculateLinearInfo();
	void calculateAngleInfo();
	void testAngularLimitMotor(int axis_index);

	void calculateJacobi(btlRotationalLimitMotor2* limot, const btlTransform& transA,const btlTransform& transB, btlConstraintInfo2* info, int srow, btlVector3& ax1, int rotational, int rotAllowed);
	int get_limit_motor_info2(btlRotationalLimitMotor2* limot,
		const btlTransform& transA,const btlTransform& transB,const btlVector3& linVelA,const btlVector3& linVelB,const btlVector3& angVelA,const btlVector3& angVelB,
		btlConstraintInfo2* info, int row, btlVector3& ax1, int rotational, int rotAllowed = false);

	static btlScalar btlGetMatrixElem(const btlMatrix3x3& mat, int index);
	static bool btl_matrixToEulerXYZ(const btlMatrix3x3& mat,btlVector3& xyz);
	static bool matrixToEulerXZY(const btlMatrix3x3& mat,btlVector3& xyz);
	static bool matrixToEulerYXZ(const btlMatrix3x3& mat,btlVector3& xyz);
	static bool matrixToEulerYZX(const btlMatrix3x3& mat,btlVector3& xyz);
	static bool matrixToEulerZXY(const btlMatrix3x3& mat,btlVector3& xyz);
	static bool matrixToEulerZYX(const btlMatrix3x3& mat,btlVector3& xyz);

public:

	BT_DECLARE_ALIGNED_ALLOCATOR();

    btlGeneric6DofSpring2Constraint(btlRigidBody& rbA, btlRigidBody& rbB, const btlTransform& frameInA, const btlTransform& frameInB, RotateOrder rotOrder = RO_XYZ);
    btlGeneric6DofSpring2Constraint(btlRigidBody& rbB, const btlTransform& frameInB, RotateOrder rotOrder = RO_XYZ);

	virtual void buildJacobian() {}
	virtual void getInfo1 (btlConstraintInfo1* info);
	virtual void getInfo2 (btlConstraintInfo2* info);
	virtual int calculateSerializeBufferSize() const;
	virtual const char* serialize(void* dataBuffer, btlSerializer* serializer) const;

	btlRotationalLimitMotor2* getRotationalLimitMotor(int index) { return &m_angularLimits[index]; }
	btlTranslationalLimitMotor2* getTranslationalLimitMotor() { return &m_linearLimits; }

	// Calculates the global transform for the joint offset for body A an B, and also calculates the angle differences between the bodies.
	void calculateTransforms(const btlTransform& transA,const btlTransform& transB);
	void calculateTransforms();

	// Gets the global transform of the offset for body A
	const btlTransform & getCalculatedTransformA() const { return m_calculatedTransformA; }
	// Gets the global transform of the offset for body B
	const btlTransform & getCalculatedTransformB() const { return m_calculatedTransformB; }

	const btlTransform & getFrameOffsetA() const { return m_frameInA; }
	const btlTransform & getFrameOffsetB() const { return m_frameInB; }

	btlTransform & getFrameOffsetA() { return m_frameInA; }
	btlTransform & getFrameOffsetB() { return m_frameInB; }

	// Get the rotation axis in global coordinates ( btGeneric6DofSpring2Constraint::calculateTransforms() must be called previously )
	btlVector3 getAxis(int axis_index) const { return m_calculatedAxis[axis_index]; }

	// Get the relative Euler angle ( btGeneric6DofSpring2Constraint::calculateTransforms() must be called previously )
	btlScalar getAngle(int axis_index) const { return m_calculatedAxisAngleDiff[axis_index]; }

	// Get the relative position of the constraint pivot ( btGeneric6DofSpring2Constraint::calculateTransforms() must be called previously )
	btlScalar getRelativePivotPosition(int axis_index) const { return m_calculatedLinearDiff[axis_index]; }

	void setFrames(const btlTransform & frameA, const btlTransform & frameB);

	void setLinearLowerLimit(const btlVector3& linearLower) { m_linearLimits.m_lowerLimit = linearLower; }
	void getLinearLowerLimit(btlVector3& linearLower) { linearLower = m_linearLimits.m_lowerLimit; }
	void setLinearUpperLimit(const btlVector3& linearUpper) { m_linearLimits.m_upperLimit = linearUpper; }
	void getLinearUpperLimit(btlVector3& linearUpper) { linearUpper = m_linearLimits.m_upperLimit; }

	void setAngularLowerLimit(const btlVector3& angularLower)
	{
		for(int i = 0; i < 3; i++) 
			m_angularLimits[i].m_loLimit = btlNormalizeAngle(angularLower[i]);
	}

	void setAngularLowerLimitReversed(const btlVector3& angularLower)
	{
		for(int i = 0; i < 3; i++) 
			m_angularLimits[i].m_hiLimit = btlNormalizeAngle(-angularLower[i]);
	}

	void getAngularLowerLimit(btlVector3& angularLower)
	{
		for(int i = 0; i < 3; i++) 
			angularLower[i] = m_angularLimits[i].m_loLimit;
	}

	void getAngularLowerLimitReversed(btlVector3& angularLower)
	{
		for(int i = 0; i < 3; i++)
			angularLower[i] = -m_angularLimits[i].m_hiLimit;
	}

	void setAngularUpperLimit(const btlVector3& angularUpper)
	{
		for(int i = 0; i < 3; i++)
			m_angularLimits[i].m_hiLimit = btlNormalizeAngle(angularUpper[i]);
	}

	void setAngularUpperLimitReversed(const btlVector3& angularUpper)
	{
		for(int i = 0; i < 3; i++)
			m_angularLimits[i].m_loLimit = btlNormalizeAngle(-angularUpper[i]);
	}

	void getAngularUpperLimit(btlVector3& angularUpper)
	{
		for(int i = 0; i < 3; i++)
			angularUpper[i] = m_angularLimits[i].m_hiLimit;
	}

	void getAngularUpperLimitReversed(btlVector3& angularUpper)
	{
		for(int i = 0; i < 3; i++)
			angularUpper[i] = -m_angularLimits[i].m_loLimit;
	}

	//first 3 are linear, next 3 are angular

	void setLimit(int axis, btlScalar lo, btlScalar hi)
	{
		if(axis<3)
		{
			m_linearLimits.m_lowerLimit[axis] = lo;
			m_linearLimits.m_upperLimit[axis] = hi;
		}
		else
		{
			lo = btlNormalizeAngle(lo);
			hi = btlNormalizeAngle(hi);
			m_angularLimits[axis-3].m_loLimit = lo;
			m_angularLimits[axis-3].m_hiLimit = hi;
		}
	}

	void setLimitReversed(int axis, btlScalar lo, btlScalar hi)
	{
		if(axis<3)
		{
			m_linearLimits.m_lowerLimit[axis] = lo;
			m_linearLimits.m_upperLimit[axis] = hi;
		}
		else
		{
			lo = btlNormalizeAngle(lo);
			hi = btlNormalizeAngle(hi);
			m_angularLimits[axis-3].m_hiLimit = -lo;
			m_angularLimits[axis-3].m_loLimit = -hi;
		}
	}

	bool isLimited(int limitIndex)
	{
		if(limitIndex<3)
		{
			return m_linearLimits.isLimited(limitIndex);
		}
		return m_angularLimits[limitIndex-3].isLimited();
	}

	void setRotationOrder(RotateOrder order) { m_rotateOrder = order; }
	RotateOrder getRotationOrder() { return m_rotateOrder; }

	void setAxis( const btlVector3& axis1, const btlVector3& axis2);

	void setBounce(int index, btlScalar bounce);

	void enableMotor(int index, bool onOff);
	void setServo(int index, bool onOff); // set the type of the motor (servo or not) (the motor has to be turned on for servo also)
	void setTargetVelocity(int index, btlScalar velocity);
	void setServoTarget(int index, btlScalar target);
	void setMaxMotorForce(int index, btlScalar force);

	void enableSpring(int index, bool onOff);
	void setStiffness(int index, btlScalar stiffness, bool limitIfNeeded = true); // if limitIfNeeded is true the system will automatically limit the stiffness in necessary situations where otherwise the spring would move unrealistically too widely
	void setDamping(int index, btlScalar damping, bool limitIfNeeded = true); // if limitIfNeeded is true the system will automatically limit the damping in necessary situations where otherwise the spring would blow up
	void setEquilibriumPoint(); // set the current constraint position/orientation as an equilibrium point for all DOF
	void setEquilibriumPoint(int index);  // set the current constraint position/orientation as an equilibrium point for given DOF
	void setEquilibriumPoint(int index, btlScalar val);

	//override the default global value of a parameter (such as ERP or CFM), optionally provide the axis (0..5). 
	//If no axis is provided, it uses the default axis for this constraint.
	virtual void setParam(int num, btlScalar value, int axis = -1);
	virtual btlScalar getParam(int num, int axis = -1) const;
};


struct btlGeneric6DofSpring2ConstraintData
{
	btlTypedConstraintData m_typeConstraintData;
	btlTransformFloatData m_rbAFrame;
	btlTransformFloatData m_rbBFrame;

	btlVector3FloatData m_linearUpperLimit;
	btlVector3FloatData m_linearLowerLimit;
	btlVector3FloatData m_linearBounce;
	btlVector3FloatData m_linearStopERP;
	btlVector3FloatData m_linearStopCFM;
	btlVector3FloatData m_linearMotorERP;
	btlVector3FloatData m_linearMotorCFM;
	btlVector3FloatData m_linearTargetVelocity;
	btlVector3FloatData m_linearMaxMotorForce;
	btlVector3FloatData m_linearServoTarget;
	btlVector3FloatData m_linearSpringStiffness;
	btlVector3FloatData m_linearSpringDamping;
	btlVector3FloatData m_linearEquilibriumPoint;
	char               m_linearEnableMotor[4];
	char               m_linearServoMotor[4];
	char               m_linearEnableSpring[4];
	char               m_linearSpringStiffnessLimited[4];
	char               m_linearSpringDampingLimited[4];
	char               m_padding1[4];

	btlVector3FloatData m_angularUpperLimit;
	btlVector3FloatData m_angularLowerLimit;
	btlVector3FloatData m_angularBounce;
	btlVector3FloatData m_angularStopERP;
	btlVector3FloatData m_angularStopCFM;
	btlVector3FloatData m_angularMotorERP;
	btlVector3FloatData m_angularMotorCFM;
	btlVector3FloatData m_angularTargetVelocity;
	btlVector3FloatData m_angularMaxMotorForce;
	btlVector3FloatData m_angularServoTarget;
	btlVector3FloatData m_angularSpringStiffness;
	btlVector3FloatData m_angularSpringDamping;
	btlVector3FloatData m_angularEquilibriumPoint;
	char               m_angularEnableMotor[4];
	char               m_angularServoMotor[4];
	char               m_angularEnableSpring[4];
	char               m_angularSpringStiffnessLimited[4];
	char               m_angularSpringDampingLimited[4];

	int                m_rotateOrder;
};

struct btlGeneric6DofSpring2ConstraintDoubleData2
{
	btlTypedConstraintDoubleData m_typeConstraintData;
	btlTransformDoubleData m_rbAFrame;
	btlTransformDoubleData m_rbBFrame;

	btlVector3DoubleData m_linearUpperLimit;
	btlVector3DoubleData m_linearLowerLimit;
	btlVector3DoubleData m_linearBounce;
	btlVector3DoubleData m_linearStopERP;
	btlVector3DoubleData m_linearStopCFM;
	btlVector3DoubleData m_linearMotorERP;
	btlVector3DoubleData m_linearMotorCFM;
	btlVector3DoubleData m_linearTargetVelocity;
	btlVector3DoubleData m_linearMaxMotorForce;
	btlVector3DoubleData m_linearServoTarget;
	btlVector3DoubleData m_linearSpringStiffness;
	btlVector3DoubleData m_linearSpringDamping;
	btlVector3DoubleData m_linearEquilibriumPoint;
	char                m_linearEnableMotor[4];
	char                m_linearServoMotor[4];
	char                m_linearEnableSpring[4];
	char                m_linearSpringStiffnessLimited[4];
	char                m_linearSpringDampingLimited[4];
	char                m_padding1[4];

	btlVector3DoubleData m_angularUpperLimit;
	btlVector3DoubleData m_angularLowerLimit;
	btlVector3DoubleData m_angularBounce;
	btlVector3DoubleData m_angularStopERP;
	btlVector3DoubleData m_angularStopCFM;
	btlVector3DoubleData m_angularMotorERP;
	btlVector3DoubleData m_angularMotorCFM;
	btlVector3DoubleData m_angularTargetVelocity;
	btlVector3DoubleData m_angularMaxMotorForce;
	btlVector3DoubleData m_angularServoTarget;
	btlVector3DoubleData m_angularSpringStiffness;
	btlVector3DoubleData m_angularSpringDamping;
	btlVector3DoubleData m_angularEquilibriumPoint;
	char                m_angularEnableMotor[4];
	char                m_angularServoMotor[4];
	char                m_angularEnableSpring[4];
	char                m_angularSpringStiffnessLimited[4];
	char                m_angularSpringDampingLimited[4];

	int                 m_rotateOrder;
};

SIMD_FORCE_INLINE int btlGeneric6DofSpring2Constraint::calculateSerializeBufferSize() const
{
	return sizeof(btlGeneric6DofSpring2ConstraintData2);
}

SIMD_FORCE_INLINE const char* btlGeneric6DofSpring2Constraint::serialize(void* dataBuffer, btlSerializer* serializer) const
{
	btlGeneric6DofSpring2ConstraintData2* dof = (btlGeneric6DofSpring2ConstraintData2*)dataBuffer;
	btlTypedConstraint::serialize(&dof->m_typeConstraintData,serializer);

	m_frameInA.serialize(dof->m_rbAFrame);
	m_frameInB.serialize(dof->m_rbBFrame);

	int i;
	for (i=0;i<3;i++)
	{
		dof->m_angularLowerLimit.m_floats[i]       = m_angularLimits[i].m_loLimit;
		dof->m_angularUpperLimit.m_floats[i]       = m_angularLimits[i].m_hiLimit;
		dof->m_angularBounce.m_floats[i]           = m_angularLimits[i].m_bounce;
		dof->m_angularStopERP.m_floats[i]          = m_angularLimits[i].m_stopERP;
		dof->m_angularStopCFM.m_floats[i]          = m_angularLimits[i].m_stopCFM;
		dof->m_angularMotorERP.m_floats[i]         = m_angularLimits[i].m_motorERP;
		dof->m_angularMotorCFM.m_floats[i]         = m_angularLimits[i].m_motorCFM;
		dof->m_angularTargetVelocity.m_floats[i]   = m_angularLimits[i].m_targetVelocity;
		dof->m_angularMaxMotorForce.m_floats[i]    = m_angularLimits[i].m_maxMotorForce;
		dof->m_angularServoTarget.m_floats[i]      = m_angularLimits[i].m_servoTarget;
		dof->m_angularSpringStiffness.m_floats[i]  = m_angularLimits[i].m_springStiffness;
		dof->m_angularSpringDamping.m_floats[i]    = m_angularLimits[i].m_springDamping;
		dof->m_angularEquilibriumPoint.m_floats[i] = m_angularLimits[i].m_equilibriumPoint;
	}
	dof->m_angularLowerLimit.m_floats[3]       = 0;
	dof->m_angularUpperLimit.m_floats[3]       = 0;
	dof->m_angularBounce.m_floats[3]           = 0;
	dof->m_angularStopERP.m_floats[3]          = 0;
	dof->m_angularStopCFM.m_floats[3]          = 0;
	dof->m_angularMotorERP.m_floats[3]         = 0;
	dof->m_angularMotorCFM.m_floats[3]         = 0;
	dof->m_angularTargetVelocity.m_floats[3]   = 0;
	dof->m_angularMaxMotorForce.m_floats[3]    = 0;
	dof->m_angularServoTarget.m_floats[3]      = 0;
	dof->m_angularSpringStiffness.m_floats[3]  = 0;
	dof->m_angularSpringDamping.m_floats[3]    = 0;
	dof->m_angularEquilibriumPoint.m_floats[3] = 0;
	for (i=0;i<4;i++)
	{
		dof->m_angularEnableMotor[i]            = i < 3 ? ( m_angularLimits[i].m_enableMotor ? 1 : 0 ) : 0;
		dof->m_angularServoMotor[i]             = i < 3 ? ( m_angularLimits[i].m_servoMotor ? 1 : 0 ) : 0;
		dof->m_angularEnableSpring[i]           = i < 3 ? ( m_angularLimits[i].m_enableSpring ? 1 : 0 ) : 0;
		dof->m_angularSpringStiffnessLimited[i] = i < 3 ? ( m_angularLimits[i].m_springStiffnessLimited ? 1 : 0 ) : 0;
		dof->m_angularSpringDampingLimited[i]   = i < 3 ? ( m_angularLimits[i].m_springDampingLimited ? 1 : 0 ) : 0;
	}

	m_linearLimits.m_lowerLimit.serialize( dof->m_linearLowerLimit );
	m_linearLimits.m_upperLimit.serialize( dof->m_linearUpperLimit );
	m_linearLimits.m_bounce.serialize( dof->m_linearBounce );
	m_linearLimits.m_stopERP.serialize( dof->m_linearStopERP );
	m_linearLimits.m_stopCFM.serialize( dof->m_linearStopCFM );
	m_linearLimits.m_motorERP.serialize( dof->m_linearMotorERP );
	m_linearLimits.m_motorCFM.serialize( dof->m_linearMotorCFM );
	m_linearLimits.m_targetVelocity.serialize( dof->m_linearTargetVelocity );
	m_linearLimits.m_maxMotorForce.serialize( dof->m_linearMaxMotorForce );
	m_linearLimits.m_servoTarget.serialize( dof->m_linearServoTarget );
	m_linearLimits.m_springStiffness.serialize( dof->m_linearSpringStiffness );
	m_linearLimits.m_springDamping.serialize( dof->m_linearSpringDamping );
	m_linearLimits.m_equilibriumPoint.serialize( dof->m_linearEquilibriumPoint );
	for (i=0;i<4;i++)
	{
		dof->m_linearEnableMotor[i]            = i < 3 ? ( m_linearLimits.m_enableMotor[i] ? 1 : 0 ) : 0;
		dof->m_linearServoMotor[i]             = i < 3 ? ( m_linearLimits.m_servoMotor[i] ? 1 : 0 ) : 0;
		dof->m_linearEnableSpring[i]           = i < 3 ? ( m_linearLimits.m_enableSpring[i] ? 1 : 0 ) : 0;
		dof->m_linearSpringStiffnessLimited[i] = i < 3 ? ( m_linearLimits.m_springStiffnessLimited[i] ? 1 : 0 ) : 0;
		dof->m_linearSpringDampingLimited[i]   = i < 3 ? ( m_linearLimits.m_springDampingLimited[i] ? 1 : 0 ) : 0;
	}

	dof->m_rotateOrder = m_rotateOrder;

	return btlGeneric6DofSpring2ConstraintDataName;
}





#endif //BT_GENERIC_6DOF_CONSTRAINT_H
