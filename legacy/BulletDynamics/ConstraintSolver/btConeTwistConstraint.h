/*
Bullet Continuous Collision Detection and Physics Library
btConeTwistConstraint is Copyright (c) 2007 Starbreeze Studios

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

Written by: Marcus Hennix
*/



/*
Overview:

btConeTwistConstraint can be used to simulate ragdoll joints (upper arm, leg etc).
It is a fixed translation, 3 degree-of-freedom (DOF) rotational "joint".
It divides the 3 rotational DOFs into swing (movement within a cone) and twist.
Swing is divided into swing1 and swing2 which can have different limits, giving an elliptical shape.
(Note: the cone's base isn't flat, so this ellipse is "embedded" on the surface of a sphere.)

In the contraint's frame of reference:
twist is along the x-axis,
and swing 1 and 2 are along the z and y axes respectively.
*/



#ifndef BT_CONETWISTCONSTRAINT_H
#define BT_CONETWISTCONSTRAINT_H

#include "LinearMath/btVector3.h"
#include "btJacobianEntry.h"
#include "btTypedConstraint.h"

#ifdef BT_USE_DOUBLE_PRECISION
#define btlConeTwistConstraintData2	btlConeTwistConstraintDoubleData
#define btlConeTwistConstraintDataName	"btConeTwistConstraintDoubleData"
#else
#define btlConeTwistConstraintData2	btlConeTwistConstraintData 
#define btlConeTwistConstraintDataName	"btConeTwistConstraintData" 
#endif //BT_USE_DOUBLE_PRECISION


class btlRigidBody;

enum btlConeTwistFlags
{
	BT_CONETWIST_FLAGS_LIN_CFM = 1,
	BT_CONETWIST_FLAGS_LIN_ERP = 2,
	BT_CONETWIST_FLAGS_ANG_CFM = 4
};

///btConeTwistConstraint can be used to simulate ragdoll joints (upper arm, leg etc)
ATTRIBUTE_ALIGNED16(class) btlConeTwistConstraint : public btlTypedConstraint
{
#ifdef IN_PARALLELL_SOLVER
public:
#endif
	btlJacobianEntry	m_jac[3]; //3 orthogonal linear constraints

	btlTransform m_rbAFrame; 
	btlTransform m_rbBFrame;

	btlScalar	m_limitSoftness;
	btlScalar	m_biasFactor;
	btlScalar	m_relaxationFactor;

	btlScalar	m_damping;

	btlScalar	m_swingSpan1;
	btlScalar	m_swingSpan2;
	btlScalar	m_twistSpan;

	btlScalar	m_fixThresh;

	btlVector3   m_swingAxis;
	btlVector3	m_twistAxis;

	btlScalar	m_kSwing;
	btlScalar	m_kTwist;

	btlScalar	m_twistLimitSign;
	btlScalar	m_swingCorrection;
	btlScalar	m_twistCorrection;

	btlScalar	m_twistAngle;

	btlScalar	m_accSwingLimitImpulse;
	btlScalar	m_accTwistLimitImpulse;

	bool		m_angularOnly;
	bool		m_solveTwistLimit;
	bool		m_solveSwingLimit;

	bool	m_useSolveConstraintObsolete;

	// not yet used...
	btlScalar	m_swingLimitRatio;
	btlScalar	m_twistLimitRatio;
	btlVector3   m_twistAxisA;

	// motor
	bool		 m_bMotorEnabled;
	bool		 m_bNormalizedMotorStrength;
	btlQuaternion m_qTarget;
	btlScalar	 m_maxMotorImpulse;
	btlVector3	 m_accMotorImpulse;
	
	// parameters
	int			m_flags;
	btlScalar	m_linCFM;
	btlScalar	m_linERP;
	btlScalar	m_angCFM;
	
protected:

	void init();

	void computeConeLimitInfo(const btlQuaternion& qCone, // in
		btlScalar& swingAngle, btlVector3& vSwingAxis, btlScalar& swingLimit); // all outs

	void computeTwistLimitInfo(const btlQuaternion& qTwist, // in
		btlScalar& twistAngle, btlVector3& vTwistAxis); // all outs

	void adjustSwingAxisToUseEllipseNormal(btlVector3& vSwingAxis) const;


public:

	BT_DECLARE_ALIGNED_ALLOCATOR();

	btlConeTwistConstraint(btlRigidBody& rbA,btlRigidBody& rbB,const btlTransform& rbAFrame, const btlTransform& rbBFrame);
	
	btlConeTwistConstraint(btlRigidBody& rbA,const btlTransform& rbAFrame);

	virtual void	buildJacobian();

	virtual void getInfo1 (btlConstraintInfo1* info);

	void	getInfo1NonVirtual(btlConstraintInfo1* info);
	
	virtual void getInfo2 (btlConstraintInfo2* info);
	
	void	getInfo2NonVirtual(btlConstraintInfo2* info,const btlTransform& transA,const btlTransform& transB,const btlMatrix3x3& invInertiaWorldA,const btlMatrix3x3& invInertiaWorldB);

	virtual	void	solveConstraintObsolete(btlSolverBody& bodyA,btlSolverBody& bodyB,btlScalar	timeStep);

    
	void	updateRHS(btlScalar	timeStep);


	const btlRigidBody& getRigidBodyA() const
	{
		return m_rbA;
	}
	const btlRigidBody& getRigidBodyB() const
	{
		return m_rbB;
	}

	void	setAngularOnly(bool angularOnly)
	{
		m_angularOnly = angularOnly;
	}
	
	bool    getAngularOnly() const
	{
	    return m_angularOnly;
	}

	void	setLimit(int limitIndex,btlScalar limitValue)
	{
		switch (limitIndex)
		{
		case 3:
			{
				m_twistSpan = limitValue;
				break;
			}
		case 4:
			{
				m_swingSpan2 = limitValue;
				break;
			}
		case 5:
			{
				m_swingSpan1 = limitValue;
				break;
			}
		default:
			{
			}
		};
	}

    btlScalar getLimit(int limitIndex) const
	{
		switch (limitIndex)
		{
		case 3:
			{
				return m_twistSpan;
				break;
			}
		case 4:
			{
				return m_swingSpan2;
				break;
			}
		case 5:
			{
				return m_swingSpan1;
				break;
			}
		default:
			{
			    btlAssert(0 && "Invalid limitIndex specified for btConeTwistConstraint");
			    return 0.0;
			}
		};
	}

	// setLimit(), a few notes:
	// _softness:
	//		0->1, recommend ~0.8->1.
	//		describes % of limits where movement is free.
	//		beyond this softness %, the limit is gradually enforced until the "hard" (1.0) limit is reached.
	// _biasFactor:
	//		0->1?, recommend 0.3 +/-0.3 or so.
	//		strength with which constraint resists zeroth order (angular, not angular velocity) limit violation.
	// __relaxationFactor:
	//		0->1, recommend to stay near 1.
	//		the lower the value, the less the constraint will fight velocities which violate the angular limits.
	void	setLimit(btlScalar _swingSpan1,btlScalar _swingSpan2,btlScalar _twistSpan, btlScalar _softness = 1.f, btlScalar _biasFactor = 0.3f, btlScalar _relaxationFactor = 1.0f)
	{
		m_swingSpan1 = _swingSpan1;
		m_swingSpan2 = _swingSpan2;
		m_twistSpan  = _twistSpan;

		m_limitSoftness =  _softness;
		m_biasFactor = _biasFactor;
		m_relaxationFactor = _relaxationFactor;
	}

	const btlTransform& getAFrame() const { return m_rbAFrame; };	
	const btlTransform& getBFrame() const { return m_rbBFrame; };

	inline int getSolveTwistLimit()
	{
		return m_solveTwistLimit;
	}

	inline int getSolveSwingLimit()
	{
		return m_solveTwistLimit;
	}

	inline btlScalar getTwistLimitSign()
	{
		return m_twistLimitSign;
	}

	void calcAngleInfo();
	void calcAngleInfo2(const btlTransform& transA, const btlTransform& transB,const btlMatrix3x3& invInertiaWorldA,const btlMatrix3x3& invInertiaWorldB);

	inline btlScalar getSwingSpan1() const
	{
		return m_swingSpan1;
	}
	inline btlScalar getSwingSpan2() const
	{
		return m_swingSpan2;
	}
	inline btlScalar getTwistSpan() const
	{
		return m_twistSpan;
	}
	inline btlScalar getLimitSoftness() const
	{
		return m_limitSoftness;
	}
	inline btlScalar getBiasFactor() const
	{
		return m_biasFactor;
	}
	inline btlScalar getRelaxationFactor() const
	{
		return m_relaxationFactor;
	}
	inline btlScalar getTwistAngle() const
	{
		return m_twistAngle;
	}
	bool isPastSwingLimit() { return m_solveSwingLimit; }

	btlScalar getDamping() const { return m_damping; }
	void setDamping(btlScalar damping) { m_damping = damping; }

	void enableMotor(bool b) { m_bMotorEnabled = b; }
	bool isMotorEnabled() const { return m_bMotorEnabled; }
	btlScalar getMaxMotorImpulse() const { return m_maxMotorImpulse; }
	bool isMaxMotorImpulseNormalized() const { return m_bNormalizedMotorStrength; }
	void setMaxMotorImpulse(btlScalar maxMotorImpulse) { m_maxMotorImpulse = maxMotorImpulse; m_bNormalizedMotorStrength = false; }
	void setMaxMotorImpulseNormalized(btlScalar maxMotorImpulse) { m_maxMotorImpulse = maxMotorImpulse; m_bNormalizedMotorStrength = true; }

	btlScalar getFixThresh() { return m_fixThresh; }
	void setFixThresh(btlScalar fixThresh) { m_fixThresh = fixThresh; }

	// setMotorTarget:
	// q: the desired rotation of bodyA wrt bodyB.
	// note: if q violates the joint limits, the internal target is clamped to avoid conflicting impulses (very bad for stability)
	// note: don't forget to enableMotor()
	void setMotorTarget(const btlQuaternion &q);
	const btlQuaternion& getMotorTarget() const { return m_qTarget; }

	// same as above, but q is the desired rotation of frameA wrt frameB in constraint space
	void setMotorTargetInConstraintSpace(const btlQuaternion &q);

	btlVector3 GetPointForAngle(btlScalar fAngleInRadians, btlScalar fLength) const;

	///override the default global value of a parameter (such as ERP or CFM), optionally provide the axis (0..5). 
	///If no axis is provided, it uses the default axis for this constraint.
	virtual	void setParam(int num, btlScalar value, int axis = -1);

	virtual void setFrames(const btlTransform& frameA, const btlTransform& frameB);

	const btlTransform& getFrameOffsetA() const
	{
		return m_rbAFrame;
	}

	const btlTransform& getFrameOffsetB() const
	{
		return m_rbBFrame;
	}


	///return the local value of parameter
	virtual	btlScalar getParam(int num, int axis = -1) const;

	int getFlags() const
	{
		return m_flags;
	}

	virtual	int	calculateSerializeBufferSize() const;

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual	const char*	serialize(void* dataBuffer, btlSerializer* serializer) const;

};


	
struct	btlConeTwistConstraintDoubleData
{
	btlTypedConstraintDoubleData	m_typeConstraintData;
	btlTransformDoubleData m_rbAFrame;
	btlTransformDoubleData m_rbBFrame;

	//limits
	double	m_swingSpan1;
	double	m_swingSpan2;
	double	m_twistSpan;
	double	m_limitSoftness;
	double	m_biasFactor;
	double	m_relaxationFactor;

	double	m_damping;
		
	

};

#ifdef BT_BACKWARDS_COMPATIBLE_SERIALIZATION
///this structure is not used, except for loading pre-2.82 .bullet files
struct	btlConeTwistConstraintData
{
	btlTypedConstraintData	m_typeConstraintData;
	btlTransformFloatData m_rbAFrame;
	btlTransformFloatData m_rbBFrame;

	//limits
	float	m_swingSpan1;
	float	m_swingSpan2;
	float	m_twistSpan;
	float	m_limitSoftness;
	float	m_biasFactor;
	float	m_relaxationFactor;

	float	m_damping;
		
	char m_pad[4];

};
#endif //BT_BACKWARDS_COMPATIBLE_SERIALIZATION
//

SIMD_FORCE_INLINE int	btlConeTwistConstraint::calculateSerializeBufferSize() const
{
	return sizeof(btlConeTwistConstraintData2);

}


	///fills the dataBuffer and returns the struct name (and 0 on failure)
SIMD_FORCE_INLINE const char*	btlConeTwistConstraint::serialize(void* dataBuffer, btlSerializer* serializer) const
{
	btlConeTwistConstraintData2* cone = (btlConeTwistConstraintData2*) dataBuffer;
	btlTypedConstraint::serialize(&cone->m_typeConstraintData,serializer);

	m_rbAFrame.serialize(cone->m_rbAFrame);
	m_rbBFrame.serialize(cone->m_rbBFrame);
	
	cone->m_swingSpan1 = m_swingSpan1;
	cone->m_swingSpan2 = m_swingSpan2;
	cone->m_twistSpan = m_twistSpan;
	cone->m_limitSoftness = m_limitSoftness;
	cone->m_biasFactor = m_biasFactor;
	cone->m_relaxationFactor = m_relaxationFactor;
	cone->m_damping = m_damping;

	return btlConeTwistConstraintDataName;
}


#endif //BT_CONETWISTCONSTRAINT_H
