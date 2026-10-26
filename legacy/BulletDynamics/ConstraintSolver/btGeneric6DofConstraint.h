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

/// 2009 March: btGeneric6DofConstraint refactored by Roman Ponomarev
/// Added support for generic constraint solver through getInfo1/getInfo2 methods

/*
2007-09-09
btGeneric6DofConstraint Refactored by Francisco Le?n
email: projectileman@yahoo.com
http://gimpact.sf.net
*/


#ifndef BT_GENERIC_6DOF_CONSTRAINT_H
#define BT_GENERIC_6DOF_CONSTRAINT_H

#include "LinearMath/btVector3.h"
#include "btJacobianEntry.h"
#include "btTypedConstraint.h"

class btlRigidBody;



#ifdef BT_USE_DOUBLE_PRECISION
#define btlGeneric6DofConstraintData2		btlGeneric6DofConstraintDoubleData2
#define btlGeneric6DofConstraintDataName	"btGeneric6DofConstraintDoubleData2"
#else
#define btlGeneric6DofConstraintData2		btlGeneric6DofConstraintData
#define btlGeneric6DofConstraintDataName	"btGeneric6DofConstraintData"
#endif //BT_USE_DOUBLE_PRECISION


//! Rotation Limit structure for generic joints
class btlRotationalLimitMotor
{
public:
    //! limit_parameters
    //!@{
    btlScalar m_loLimit;//!< joint limit
    btlScalar m_hiLimit;//!< joint limit
    btlScalar m_targetVelocity;//!< target motor velocity
    btlScalar m_maxMotorForce;//!< max force on motor
    btlScalar m_maxLimitForce;//!< max force on limit
    btlScalar m_damping;//!< Damping.
    btlScalar m_limitSoftness;//! Relaxation factor
    btlScalar m_normalCFM;//!< Constraint force mixing factor
    btlScalar m_stopERP;//!< Error tolerance factor when joint is at limit
    btlScalar m_stopCFM;//!< Constraint force mixing factor when joint is at limit
    btlScalar m_bounce;//!< restitution factor
    bool m_enableMotor;

    //!@}

    //! temp_variables
    //!@{
    btlScalar m_currentLimitError;//!  How much is violated this limit
    btlScalar m_currentPosition;     //!  current value of angle 
    int m_currentLimit;//!< 0=free, 1=at lo limit, 2=at hi limit
    btlScalar m_accumulatedImpulse;
    //!@}

    btlRotationalLimitMotor()
    {
    	m_accumulatedImpulse = 0.f;
        m_targetVelocity = 0;
        m_maxMotorForce = 0.1f;
        m_maxLimitForce = 300.0f;
        m_loLimit = 1.0f;
        m_hiLimit = -1.0f;
		m_normalCFM = 0.f;
		m_stopERP = 0.2f;
		m_stopCFM = 0.f;
        m_bounce = 0.0f;
        m_damping = 1.0f;
        m_limitSoftness = 0.5f;
        m_currentLimit = 0;
        m_currentLimitError = 0;
        m_enableMotor = false;
    }

    btlRotationalLimitMotor(const btlRotationalLimitMotor & limot)
    {
        m_targetVelocity = limot.m_targetVelocity;
        m_maxMotorForce = limot.m_maxMotorForce;
        m_limitSoftness = limot.m_limitSoftness;
        m_loLimit = limot.m_loLimit;
        m_hiLimit = limot.m_hiLimit;
		m_normalCFM = limot.m_normalCFM;
		m_stopERP = limot.m_stopERP;
		m_stopCFM =	limot.m_stopCFM;
        m_bounce = limot.m_bounce;
        m_currentLimit = limot.m_currentLimit;
        m_currentLimitError = limot.m_currentLimitError;
        m_enableMotor = limot.m_enableMotor;
    }



	//! Is limited
    bool isLimited() const
    {
    	if(m_loLimit > m_hiLimit) return false;
    	return true;
    }

	//! Need apply correction
    bool needApplyTorques() const
    {
    	if(m_currentLimit == 0 && m_enableMotor == false) return false;
    	return true;
    }

	//! calculates  error
	/*!
	calculates m_currentLimit and m_currentLimitError.
	*/
	int testLimitValue(btlScalar test_value);

	//! apply the correction impulses for two bodies
    btlScalar solveAngularLimits(btlScalar timeStep,btlVector3& axis, btlScalar jacDiagABInv,btlRigidBody * body0, btlRigidBody * body1);

};



class btlTranslationalLimitMotor
{
public:
	btlVector3 m_lowerLimit;//!< the constraint lower limits
    btlVector3 m_upperLimit;//!< the constraint upper limits
    btlVector3 m_accumulatedImpulse;
    //! Linear_Limit_parameters
    //!@{
    btlScalar	m_limitSoftness;//!< Softness for linear limit
    btlScalar	m_damping;//!< Damping for linear limit
    btlScalar	m_restitution;//! Bounce parameter for linear limit
	btlVector3	m_normalCFM;//!< Constraint force mixing factor
    btlVector3	m_stopERP;//!< Error tolerance factor when joint is at limit
	btlVector3	m_stopCFM;//!< Constraint force mixing factor when joint is at limit
    //!@}
	bool		m_enableMotor[3];
    btlVector3	m_targetVelocity;//!< target motor velocity
    btlVector3	m_maxMotorForce;//!< max force on motor
    btlVector3	m_currentLimitError;//!  How much is violated this limit
    btlVector3	m_currentLinearDiff;//!  Current relative offset of constraint frames
    int			m_currentLimit[3];//!< 0=free, 1=at lower limit, 2=at upper limit

    btlTranslationalLimitMotor()
    {
    	m_lowerLimit.setValue(0.f,0.f,0.f);
    	m_upperLimit.setValue(0.f,0.f,0.f);
    	m_accumulatedImpulse.setValue(0.f,0.f,0.f);
		m_normalCFM.setValue(0.f, 0.f, 0.f);
		m_stopERP.setValue(0.2f, 0.2f, 0.2f);
		m_stopCFM.setValue(0.f, 0.f, 0.f);

    	m_limitSoftness = 0.7f;
    	m_damping = btlScalar(1.0f);
    	m_restitution = btlScalar(0.5f);
		for(int i=0; i < 3; i++) 
		{
			m_enableMotor[i] = false;
			m_targetVelocity[i] = btlScalar(0.f);
			m_maxMotorForce[i] = btlScalar(0.f);
		}
    }

    btlTranslationalLimitMotor(const btlTranslationalLimitMotor & other )
    {
    	m_lowerLimit = other.m_lowerLimit;
    	m_upperLimit = other.m_upperLimit;
    	m_accumulatedImpulse = other.m_accumulatedImpulse;

    	m_limitSoftness = other.m_limitSoftness ;
    	m_damping = other.m_damping;
    	m_restitution = other.m_restitution;
		m_normalCFM = other.m_normalCFM;
		m_stopERP = other.m_stopERP;
		m_stopCFM = other.m_stopCFM;

		for(int i=0; i < 3; i++) 
		{
			m_enableMotor[i] = other.m_enableMotor[i];
			m_targetVelocity[i] = other.m_targetVelocity[i];
			m_maxMotorForce[i] = other.m_maxMotorForce[i];
		}
    }

    //! Test limit
	/*!
    - free means upper < lower,
    - locked means upper == lower
    - limited means upper > lower
    - limitIndex: first 3 are linear, next 3 are angular
    */
    inline bool	isLimited(int limitIndex) const
    {
       return (m_upperLimit[limitIndex] >= m_lowerLimit[limitIndex]);
    }
    inline bool needApplyForce(int limitIndex) const
    {
    	if(m_currentLimit[limitIndex] == 0 && m_enableMotor[limitIndex] == false) return false;
    	return true;
    }
	int testLimitValue(int limitIndex, btlScalar test_value);


    btlScalar solveLinearAxis(
    	btlScalar timeStep,
        btlScalar jacDiagABInv,
        btlRigidBody& body1,const btlVector3 &pointInA,
        btlRigidBody& body2,const btlVector3 &pointInB,
        int limit_index,
        const btlVector3 & axis_normal_on_a,
		const btlVector3 & anchorPos);


};

enum btl6DofFlags
{
	BT_6DOF_FLAGS_CFM_NORM = 1,
	BT_6DOF_FLAGS_CFM_STOP = 2,
	BT_6DOF_FLAGS_ERP_STOP = 4
};
#define BT_6DOF_FLAGS_AXIS_SHIFT 3 // bits per axis


/// btGeneric6DofConstraint between two rigidbodies each with a pivotpoint that descibes the axis location in local space
/*!
btGeneric6DofConstraint can leave any of the 6 degree of freedom 'free' or 'locked'.
currently this limit supports rotational motors<br>
<ul>
<li> For Linear limits, use btGeneric6DofConstraint.setLinearUpperLimit, btGeneric6DofConstraint.setLinearLowerLimit. You can set the parameters with the btTranslationalLimitMotor structure accsesible through the btGeneric6DofConstraint.getTranslationalLimitMotor method.
At this moment translational motors are not supported. May be in the future. </li>

<li> For Angular limits, use the btRotationalLimitMotor structure for configuring the limit.
This is accessible through btGeneric6DofConstraint.getLimitMotor method,
This brings support for limit parameters and motors. </li>

<li> Angulars limits have these possible ranges:
<table border=1 >
<tr>
	<td><b>AXIS</b></td>
	<td><b>MIN ANGLE</b></td>
	<td><b>MAX ANGLE</b></td>
</tr><tr>
	<td>X</td>
	<td>-PI</td>
	<td>PI</td>
</tr><tr>
	<td>Y</td>
	<td>-PI/2</td>
	<td>PI/2</td>
</tr><tr>
	<td>Z</td>
	<td>-PI</td>
	<td>PI</td>
</tr>
</table>
</li>
</ul>

*/
ATTRIBUTE_ALIGNED16(class) btlGeneric6DofConstraint : public btlTypedConstraint
{
protected:

	//! relative_frames
    //!@{
	btlTransform	m_frameInA;//!< the constraint space w.r.t body A
    btlTransform	m_frameInB;//!< the constraint space w.r.t body B
    //!@}

    //! Jacobians
    //!@{
    btlJacobianEntry	m_jacLinear[3];//!< 3 orthogonal linear constraints
    btlJacobianEntry	m_jacAng[3];//!< 3 orthogonal angular constraints
    //!@}

	//! Linear_Limit_parameters
    //!@{
    btlTranslationalLimitMotor m_linearLimits;
    //!@}


    //! hinge_parameters
    //!@{
    btlRotationalLimitMotor m_angularLimits[3];
	//!@}


protected:
    //! temporal variables
    //!@{
    btlScalar m_timeStep;
    btlTransform m_calculatedTransformA;
    btlTransform m_calculatedTransformB;
    btlVector3 m_calculatedAxisAngleDiff;
    btlVector3 m_calculatedAxis[3];
    btlVector3 m_calculatedLinearDiff;
	btlScalar	m_factA;
	btlScalar	m_factB;
	bool		m_hasStaticBody;
    
	btlVector3 m_AnchorPos; // point betwen pivots of bodies A and B to solve linear axes

    bool	m_useLinearReferenceFrameA;
	bool	m_useOffsetForConstraintFrame;
    
	int		m_flags;

    //!@}

    btlGeneric6DofConstraint&	operator=(btlGeneric6DofConstraint&	other)
    {
        btlAssert(0);
        (void) other;
        return *this;
    }


	int setAngularLimits(btlConstraintInfo2 *info, int row_offset,const btlTransform& transA,const btlTransform& transB,const btlVector3& linVelA,const btlVector3& linVelB,const btlVector3& angVelA,const btlVector3& angVelB);

	int setLinearLimits(btlConstraintInfo2 *info, int row, const btlTransform& transA,const btlTransform& transB,const btlVector3& linVelA,const btlVector3& linVelB,const btlVector3& angVelA,const btlVector3& angVelB);

    void buildLinearJacobian(
        btlJacobianEntry & jacLinear,const btlVector3 & normalWorld,
        const btlVector3 & pivotAInW,const btlVector3 & pivotBInW);

    void buildAngularJacobian(btlJacobianEntry & jacAngular,const btlVector3 & jointAxisW);

	// tests linear limits
	void calculateLinearInfo();

	//! calcs the euler angles between the two bodies.
    void calculateAngleInfo();



public:

	BT_DECLARE_ALIGNED_ALLOCATOR();
	
	///for backwards compatibility during the transition to 'getInfo/getInfo2'
	bool		m_useSolveConstraintObsolete;

    btlGeneric6DofConstraint(btlRigidBody& rbA, btlRigidBody& rbB, const btlTransform& frameInA, const btlTransform& frameInB ,bool useLinearReferenceFrameA);
    btlGeneric6DofConstraint(btlRigidBody& rbB, const btlTransform& frameInB, bool useLinearReferenceFrameB);
    
	//! Calcs global transform of the offsets
	/*!
	Calcs the global transform for the joint offset for body A an B, and also calcs the agle differences between the bodies.
	\sa btGeneric6DofConstraint.getCalculatedTransformA , btGeneric6DofConstraint.getCalculatedTransformB, btGeneric6DofConstraint.calculateAngleInfo
	*/
    void calculateTransforms(const btlTransform& transA,const btlTransform& transB);

	void calculateTransforms();

	//! Gets the global transform of the offset for body A
    /*!
    \sa btGeneric6DofConstraint.getFrameOffsetA, btGeneric6DofConstraint.getFrameOffsetB, btGeneric6DofConstraint.calculateAngleInfo.
    */
    const btlTransform & getCalculatedTransformA() const
    {
    	return m_calculatedTransformA;
    }

    //! Gets the global transform of the offset for body B
    /*!
    \sa btGeneric6DofConstraint.getFrameOffsetA, btGeneric6DofConstraint.getFrameOffsetB, btGeneric6DofConstraint.calculateAngleInfo.
    */
    const btlTransform & getCalculatedTransformB() const
    {
    	return m_calculatedTransformB;
    }

    const btlTransform & getFrameOffsetA() const
    {
    	return m_frameInA;
    }

    const btlTransform & getFrameOffsetB() const
    {
    	return m_frameInB;
    }


    btlTransform & getFrameOffsetA()
    {
    	return m_frameInA;
    }

    btlTransform & getFrameOffsetB()
    {
    	return m_frameInB;
    }


	//! performs Jacobian calculation, and also calculates angle differences and axis
    virtual void	buildJacobian();

	virtual void getInfo1 (btlConstraintInfo1* info);

	void getInfo1NonVirtual (btlConstraintInfo1* info);

	virtual void getInfo2 (btlConstraintInfo2* info);

	void getInfo2NonVirtual (btlConstraintInfo2* info,const btlTransform& transA,const btlTransform& transB,const btlVector3& linVelA,const btlVector3& linVelB,const btlVector3& angVelA,const btlVector3& angVelB);


    void	updateRHS(btlScalar	timeStep);

	//! Get the rotation axis in global coordinates
	/*!
	\pre btGeneric6DofConstraint.buildJacobian must be called previously.
	*/
    btlVector3 getAxis(int axis_index) const;

    //! Get the relative Euler angle
    /*!
	\pre btGeneric6DofConstraint::calculateTransforms() must be called previously.
	*/
    btlScalar getAngle(int axis_index) const;

	//! Get the relative position of the constraint pivot
    /*!
	\pre btGeneric6DofConstraint::calculateTransforms() must be called previously.
	*/
	btlScalar getRelativePivotPosition(int axis_index) const;

	void setFrames(const btlTransform & frameA, const btlTransform & frameB);

	//! Test angular limit.
	/*!
	Calculates angular correction and returns true if limit needs to be corrected.
	\pre btGeneric6DofConstraint::calculateTransforms() must be called previously.
	*/
    bool testAngularLimitMotor(int axis_index);

    void	setLinearLowerLimit(const btlVector3& linearLower)
    {
    	m_linearLimits.m_lowerLimit = linearLower;
    }

	void	getLinearLowerLimit(btlVector3& linearLower) const
	{
		linearLower = m_linearLimits.m_lowerLimit;
	}

	void	setLinearUpperLimit(const btlVector3& linearUpper)
	{
		m_linearLimits.m_upperLimit = linearUpper;
	}

	void	getLinearUpperLimit(btlVector3& linearUpper) const
	{
		linearUpper = m_linearLimits.m_upperLimit;
	}

    void	setAngularLowerLimit(const btlVector3& angularLower)
    {
		for(int i = 0; i < 3; i++) 
			m_angularLimits[i].m_loLimit = btlNormalizeAngle(angularLower[i]);
    }

	void	getAngularLowerLimit(btlVector3& angularLower) const
	{
		for(int i = 0; i < 3; i++) 
			angularLower[i] = m_angularLimits[i].m_loLimit;
	}

    void	setAngularUpperLimit(const btlVector3& angularUpper)
    {
		for(int i = 0; i < 3; i++)
			m_angularLimits[i].m_hiLimit = btlNormalizeAngle(angularUpper[i]);
    }

	void	getAngularUpperLimit(btlVector3& angularUpper) const
	{
		for(int i = 0; i < 3; i++)
			angularUpper[i] = m_angularLimits[i].m_hiLimit;
	}

	//! Retrieves the angular limit informacion
    btlRotationalLimitMotor * getRotationalLimitMotor(int index)
    {
    	return &m_angularLimits[index];
    }

    //! Retrieves the  limit informacion
    btlTranslationalLimitMotor * getTranslationalLimitMotor()
    {
    	return &m_linearLimits;
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

	//! Test limit
	/*!
    - free means upper < lower,
    - locked means upper == lower
    - limited means upper > lower
    - limitIndex: first 3 are linear, next 3 are angular
    */
    bool	isLimited(int limitIndex) const
    {
    	if(limitIndex<3)
    	{
			return m_linearLimits.isLimited(limitIndex);

    	}
        return m_angularLimits[limitIndex-3].isLimited();
    }

	virtual void calcAnchorPos(void); // overridable

	int get_limit_motor_info2(	btlRotationalLimitMotor * limot,
								const btlTransform& transA,const btlTransform& transB,const btlVector3& linVelA,const btlVector3& linVelB,const btlVector3& angVelA,const btlVector3& angVelB,
								btlConstraintInfo2 *info, int row, btlVector3& ax1, int rotational, int rotAllowed = false);

	// access for UseFrameOffset
	bool getUseFrameOffset() const { return m_useOffsetForConstraintFrame; }
	void setUseFrameOffset(bool frameOffsetOnOff) { m_useOffsetForConstraintFrame = frameOffsetOnOff; }
	
	bool getUseLinearReferenceFrameA() const { return m_useLinearReferenceFrameA; }
	void setUseLinearReferenceFrameA(bool linearReferenceFrameA) { m_useLinearReferenceFrameA = linearReferenceFrameA; }

	///override the default global value of a parameter (such as ERP or CFM), optionally provide the axis (0..5). 
	///If no axis is provided, it uses the default axis for this constraint.
	virtual	void setParam(int num, btlScalar value, int axis = -1);
	///return the local value of parameter
	virtual	btlScalar getParam(int num, int axis = -1) const;

	void setAxis( const btlVector3& axis1, const btlVector3& axis2);

    	virtual	int getFlags() const
    	{
        	return m_flags;
	}

	virtual	int	calculateSerializeBufferSize() const;

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual	const char*	serialize(void* dataBuffer, btlSerializer* serializer) const;

	
};


struct btlGeneric6DofConstraintData
{
	btlTypedConstraintData	m_typeConstraintData;
	btlTransformFloatData m_rbAFrame; // constraint axii. Assumes z is hinge axis.
	btlTransformFloatData m_rbBFrame;
	
	btlVector3FloatData	m_linearUpperLimit;
	btlVector3FloatData	m_linearLowerLimit;

	btlVector3FloatData	m_angularUpperLimit;
	btlVector3FloatData	m_angularLowerLimit;
	
	int	m_useLinearReferenceFrameA;
	int m_useOffsetForConstraintFrame;
};

struct btlGeneric6DofConstraintDoubleData2
{
	btlTypedConstraintDoubleData	m_typeConstraintData;
	btlTransformDoubleData m_rbAFrame; // constraint axii. Assumes z is hinge axis.
	btlTransformDoubleData m_rbBFrame;
	
	btlVector3DoubleData	m_linearUpperLimit;
	btlVector3DoubleData	m_linearLowerLimit;

	btlVector3DoubleData	m_angularUpperLimit;
	btlVector3DoubleData	m_angularLowerLimit;
	
	int	m_useLinearReferenceFrameA;
	int m_useOffsetForConstraintFrame;
};

SIMD_FORCE_INLINE	int	btlGeneric6DofConstraint::calculateSerializeBufferSize() const
{
	return sizeof(btlGeneric6DofConstraintData2);
}

	///fills the dataBuffer and returns the struct name (and 0 on failure)
SIMD_FORCE_INLINE	const char*	btlGeneric6DofConstraint::serialize(void* dataBuffer, btlSerializer* serializer) const
{

	btlGeneric6DofConstraintData2* dof = (btlGeneric6DofConstraintData2*)dataBuffer;
	btlTypedConstraint::serialize(&dof->m_typeConstraintData,serializer);

	m_frameInA.serialize(dof->m_rbAFrame);
	m_frameInB.serialize(dof->m_rbBFrame);

		
	int i;
	for (i=0;i<3;i++)
	{
		dof->m_angularLowerLimit.m_floats[i] =  m_angularLimits[i].m_loLimit;
		dof->m_angularUpperLimit.m_floats[i] =  m_angularLimits[i].m_hiLimit;
		dof->m_linearLowerLimit.m_floats[i] = m_linearLimits.m_lowerLimit[i];
		dof->m_linearUpperLimit.m_floats[i] = m_linearLimits.m_upperLimit[i];
	}
	
	dof->m_useLinearReferenceFrameA = m_useLinearReferenceFrameA? 1 : 0;
	dof->m_useOffsetForConstraintFrame = m_useOffsetForConstraintFrame ? 1 : 0;

	return btlGeneric6DofConstraintDataName;
}





#endif //BT_GENERIC_6DOF_CONSTRAINT_H
