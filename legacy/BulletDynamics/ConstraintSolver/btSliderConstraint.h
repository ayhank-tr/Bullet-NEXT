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
Added by Roman Ponomarev (rponom@gmail.com)
April 04, 2008

TODO:
 - add clamping od accumulated impulse to improve stability
 - add conversion for ODE constraint solver
*/

#ifndef BT_SLIDER_CONSTRAINT_H
#define BT_SLIDER_CONSTRAINT_H

#ifdef BT_USE_DOUBLE_PRECISION
#define btlSliderConstraintData2		btlSliderConstraintDoubleData
#define btlSliderConstraintDataName  "btSliderConstraintDoubleData"
#else
#define btlSliderConstraintData2		btlSliderConstraintData 
#define btlSliderConstraintDataName	"btSliderConstraintData"
#endif //BT_USE_DOUBLE_PRECISION

#include "LinearMath/btVector3.h"
#include "btJacobianEntry.h"
#include "btTypedConstraint.h"



class btlRigidBody;



#define SLIDER_CONSTRAINT_DEF_SOFTNESS		(btlScalar(1.0))
#define SLIDER_CONSTRAINT_DEF_DAMPING		(btlScalar(1.0))
#define SLIDER_CONSTRAINT_DEF_RESTITUTION	(btlScalar(0.7))
#define SLIDER_CONSTRAINT_DEF_CFM			(btlScalar(0.f))


enum btlSliderFlags
{
	BT_SLIDER_FLAGS_CFM_DIRLIN = (1 << 0),
	BT_SLIDER_FLAGS_ERP_DIRLIN = (1 << 1),
	BT_SLIDER_FLAGS_CFM_DIRANG = (1 << 2),
	BT_SLIDER_FLAGS_ERP_DIRANG = (1 << 3),
	BT_SLIDER_FLAGS_CFM_ORTLIN = (1 << 4),
	BT_SLIDER_FLAGS_ERP_ORTLIN = (1 << 5),
	BT_SLIDER_FLAGS_CFM_ORTANG = (1 << 6),
	BT_SLIDER_FLAGS_ERP_ORTANG = (1 << 7),
	BT_SLIDER_FLAGS_CFM_LIMLIN = (1 << 8),
	BT_SLIDER_FLAGS_ERP_LIMLIN = (1 << 9),
	BT_SLIDER_FLAGS_CFM_LIMANG = (1 << 10),
	BT_SLIDER_FLAGS_ERP_LIMANG = (1 << 11)
};


ATTRIBUTE_ALIGNED16(class) btlSliderConstraint : public btlTypedConstraint
{
protected:
	///for backwards compatibility during the transition to 'getInfo/getInfo2'
	bool		m_useSolveConstraintObsolete;
	bool		m_useOffsetForConstraintFrame;
	btlTransform	m_frameInA;
    btlTransform	m_frameInB;
	// use frameA fo define limits, if true
	bool m_useLinearReferenceFrameA;
	// linear limits
	btlScalar m_lowerLinLimit;
	btlScalar m_upperLinLimit;
	// angular limits
	btlScalar m_lowerAngLimit;
	btlScalar m_upperAngLimit;
	// softness, restitution and damping for different cases
	// DirLin - moving inside linear limits
	// LimLin - hitting linear limit
	// DirAng - moving inside angular limits
	// LimAng - hitting angular limit
	// OrthoLin, OrthoAng - against constraint axis
	btlScalar m_softnessDirLin;
	btlScalar m_restitutionDirLin;
	btlScalar m_dampingDirLin;
	btlScalar m_cfmDirLin;

	btlScalar m_softnessDirAng;
	btlScalar m_restitutionDirAng;
	btlScalar m_dampingDirAng;
	btlScalar m_cfmDirAng;

	btlScalar m_softnessLimLin;
	btlScalar m_restitutionLimLin;
	btlScalar m_dampingLimLin;
	btlScalar m_cfmLimLin;

	btlScalar m_softnessLimAng;
	btlScalar m_restitutionLimAng;
	btlScalar m_dampingLimAng;
	btlScalar m_cfmLimAng;

	btlScalar m_softnessOrthoLin;
	btlScalar m_restitutionOrthoLin;
	btlScalar m_dampingOrthoLin;
	btlScalar m_cfmOrthoLin;

	btlScalar m_softnessOrthoAng;
	btlScalar m_restitutionOrthoAng;
	btlScalar m_dampingOrthoAng;
	btlScalar m_cfmOrthoAng;
	
	// for interlal use
	bool m_solveLinLim;
	bool m_solveAngLim;

	int m_flags;

	btlJacobianEntry	m_jacLin[3];
	btlScalar		m_jacLinDiagABInv[3];

    btlJacobianEntry	m_jacAng[3];

	btlScalar m_timeStep;
    btlTransform m_calculatedTransformA;
    btlTransform m_calculatedTransformB;

	btlVector3 m_sliderAxis;
	btlVector3 m_realPivotAInW;
	btlVector3 m_realPivotBInW;
	btlVector3 m_projPivotInW;
	btlVector3 m_delta;
	btlVector3 m_depth;
	btlVector3 m_relPosA;
	btlVector3 m_relPosB;

	btlScalar m_linPos;
	btlScalar m_angPos;

	btlScalar m_angDepth;
	btlScalar m_kAngle;

	bool	 m_poweredLinMotor;
    btlScalar m_targetLinMotorVelocity;
    btlScalar m_maxLinMotorForce;
    btlScalar m_accumulatedLinMotorImpulse;
	
	bool	 m_poweredAngMotor;
    btlScalar m_targetAngMotorVelocity;
    btlScalar m_maxAngMotorForce;
    btlScalar m_accumulatedAngMotorImpulse;

	//------------------------    
	void initParams();
public:
	BT_DECLARE_ALIGNED_ALLOCATOR();
	
	// constructors
    btlSliderConstraint(btlRigidBody& rbA, btlRigidBody& rbB, const btlTransform& frameInA, const btlTransform& frameInB ,bool useLinearReferenceFrameA);
    btlSliderConstraint(btlRigidBody& rbB, const btlTransform& frameInB, bool useLinearReferenceFrameA);

	// overrides

    virtual void getInfo1 (btlConstraintInfo1* info);

	void getInfo1NonVirtual(btlConstraintInfo1* info);
	
	virtual void getInfo2 (btlConstraintInfo2* info);

	void getInfo2NonVirtual(btlConstraintInfo2* info, const btlTransform& transA, const btlTransform& transB,const btlVector3& linVelA,const btlVector3& linVelB, btlScalar rbAinvMass,btlScalar rbBinvMass);


	// access
    const btlRigidBody& getRigidBodyA() const { return m_rbA; }
    const btlRigidBody& getRigidBodyB() const { return m_rbB; }
    const btlTransform & getCalculatedTransformA() const { return m_calculatedTransformA; }
    const btlTransform & getCalculatedTransformB() const { return m_calculatedTransformB; }
    const btlTransform & getFrameOffsetA() const { return m_frameInA; }
    const btlTransform & getFrameOffsetB() const { return m_frameInB; }
    btlTransform & getFrameOffsetA() { return m_frameInA; }
    btlTransform & getFrameOffsetB() { return m_frameInB; }
    btlScalar getLowerLinLimit() { return m_lowerLinLimit; }
    void setLowerLinLimit(btlScalar lowerLimit) { m_lowerLinLimit = lowerLimit; }
    btlScalar getUpperLinLimit() { return m_upperLinLimit; }
    void setUpperLinLimit(btlScalar upperLimit) { m_upperLinLimit = upperLimit; }
    btlScalar getLowerAngLimit() { return m_lowerAngLimit; }
    void setLowerAngLimit(btlScalar lowerLimit) { m_lowerAngLimit = btlNormalizeAngle(lowerLimit); }
    btlScalar getUpperAngLimit() { return m_upperAngLimit; }
    void setUpperAngLimit(btlScalar upperLimit) { m_upperAngLimit = btlNormalizeAngle(upperLimit); }
	bool getUseLinearReferenceFrameA() { return m_useLinearReferenceFrameA; }
	btlScalar getSoftnessDirLin() { return m_softnessDirLin; }
	btlScalar getRestitutionDirLin() { return m_restitutionDirLin; }
	btlScalar getDampingDirLin() { return m_dampingDirLin ; }
	btlScalar getSoftnessDirAng() { return m_softnessDirAng; }
	btlScalar getRestitutionDirAng() { return m_restitutionDirAng; }
	btlScalar getDampingDirAng() { return m_dampingDirAng; }
	btlScalar getSoftnessLimLin() { return m_softnessLimLin; }
	btlScalar getRestitutionLimLin() { return m_restitutionLimLin; }
	btlScalar getDampingLimLin() { return m_dampingLimLin; }
	btlScalar getSoftnessLimAng() { return m_softnessLimAng; }
	btlScalar getRestitutionLimAng() { return m_restitutionLimAng; }
	btlScalar getDampingLimAng() { return m_dampingLimAng; }
	btlScalar getSoftnessOrthoLin() { return m_softnessOrthoLin; }
	btlScalar getRestitutionOrthoLin() { return m_restitutionOrthoLin; }
	btlScalar getDampingOrthoLin() { return m_dampingOrthoLin; }
	btlScalar getSoftnessOrthoAng() { return m_softnessOrthoAng; }
	btlScalar getRestitutionOrthoAng() { return m_restitutionOrthoAng; }
	btlScalar getDampingOrthoAng() { return m_dampingOrthoAng; }
	void setSoftnessDirLin(btlScalar softnessDirLin) { m_softnessDirLin = softnessDirLin; }
	void setRestitutionDirLin(btlScalar restitutionDirLin) { m_restitutionDirLin = restitutionDirLin; }
	void setDampingDirLin(btlScalar dampingDirLin) { m_dampingDirLin = dampingDirLin; }
	void setSoftnessDirAng(btlScalar softnessDirAng) { m_softnessDirAng = softnessDirAng; }
	void setRestitutionDirAng(btlScalar restitutionDirAng) { m_restitutionDirAng = restitutionDirAng; }
	void setDampingDirAng(btlScalar dampingDirAng) { m_dampingDirAng = dampingDirAng; }
	void setSoftnessLimLin(btlScalar softnessLimLin) { m_softnessLimLin = softnessLimLin; }
	void setRestitutionLimLin(btlScalar restitutionLimLin) { m_restitutionLimLin = restitutionLimLin; }
	void setDampingLimLin(btlScalar dampingLimLin) { m_dampingLimLin = dampingLimLin; }
	void setSoftnessLimAng(btlScalar softnessLimAng) { m_softnessLimAng = softnessLimAng; }
	void setRestitutionLimAng(btlScalar restitutionLimAng) { m_restitutionLimAng = restitutionLimAng; }
	void setDampingLimAng(btlScalar dampingLimAng) { m_dampingLimAng = dampingLimAng; }
	void setSoftnessOrthoLin(btlScalar softnessOrthoLin) { m_softnessOrthoLin = softnessOrthoLin; }
	void setRestitutionOrthoLin(btlScalar restitutionOrthoLin) { m_restitutionOrthoLin = restitutionOrthoLin; }
	void setDampingOrthoLin(btlScalar dampingOrthoLin) { m_dampingOrthoLin = dampingOrthoLin; }
	void setSoftnessOrthoAng(btlScalar softnessOrthoAng) { m_softnessOrthoAng = softnessOrthoAng; }
	void setRestitutionOrthoAng(btlScalar restitutionOrthoAng) { m_restitutionOrthoAng = restitutionOrthoAng; }
	void setDampingOrthoAng(btlScalar dampingOrthoAng) { m_dampingOrthoAng = dampingOrthoAng; }
	void setPoweredLinMotor(bool onOff) { m_poweredLinMotor = onOff; }
	bool getPoweredLinMotor() { return m_poweredLinMotor; }
	void setTargetLinMotorVelocity(btlScalar targetLinMotorVelocity) { m_targetLinMotorVelocity = targetLinMotorVelocity; }
	btlScalar getTargetLinMotorVelocity() { return m_targetLinMotorVelocity; }
	void setMaxLinMotorForce(btlScalar maxLinMotorForce) { m_maxLinMotorForce = maxLinMotorForce; }
	btlScalar getMaxLinMotorForce() { return m_maxLinMotorForce; }
	void setPoweredAngMotor(bool onOff) { m_poweredAngMotor = onOff; }
	bool getPoweredAngMotor() { return m_poweredAngMotor; }
	void setTargetAngMotorVelocity(btlScalar targetAngMotorVelocity) { m_targetAngMotorVelocity = targetAngMotorVelocity; }
	btlScalar getTargetAngMotorVelocity() { return m_targetAngMotorVelocity; }
	void setMaxAngMotorForce(btlScalar maxAngMotorForce) { m_maxAngMotorForce = maxAngMotorForce; }
	btlScalar getMaxAngMotorForce() { return m_maxAngMotorForce; }

	btlScalar getLinearPos() const { return m_linPos; }
	btlScalar getAngularPos() const { return m_angPos; }
	
	

	// access for ODE solver
	bool getSolveLinLimit() { return m_solveLinLim; }
	btlScalar getLinDepth() { return m_depth[0]; }
	bool getSolveAngLimit() { return m_solveAngLim; }
	btlScalar getAngDepth() { return m_angDepth; }
	// shared code used by ODE solver
	void	calculateTransforms(const btlTransform& transA,const btlTransform& transB);
	void	testLinLimits();
	void	testAngLimits();
	// access for PE Solver
	btlVector3 getAncorInA();
	btlVector3 getAncorInB();
	// access for UseFrameOffset
	bool getUseFrameOffset() { return m_useOffsetForConstraintFrame; }
	void setUseFrameOffset(bool frameOffsetOnOff) { m_useOffsetForConstraintFrame = frameOffsetOnOff; }

	void setFrames(const btlTransform& frameA, const btlTransform& frameB) 
	{ 
		m_frameInA=frameA; 
		m_frameInB=frameB;
		calculateTransforms(m_rbA.getCenterOfMassTransform(),m_rbB.getCenterOfMassTransform());
		buildJacobian();
	} 


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


///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64


struct btlSliderConstraintData
{
	btlTypedConstraintData	m_typeConstraintData;
	btlTransformFloatData m_rbAFrame; // constraint axii. Assumes z is hinge axis.
	btlTransformFloatData m_rbBFrame;
	
	float	m_linearUpperLimit;
	float	m_linearLowerLimit;

	float	m_angularUpperLimit;
	float	m_angularLowerLimit;

	int	m_useLinearReferenceFrameA;
	int m_useOffsetForConstraintFrame;

};


struct btlSliderConstraintDoubleData
{
	btlTypedConstraintDoubleData	m_typeConstraintData;
	btlTransformDoubleData m_rbAFrame; // constraint axii. Assumes z is hinge axis.
	btlTransformDoubleData m_rbBFrame;
	
	double	m_linearUpperLimit;
	double	m_linearLowerLimit;

	double	m_angularUpperLimit;
	double	m_angularLowerLimit;

	int	m_useLinearReferenceFrameA;
	int m_useOffsetForConstraintFrame;

};

SIMD_FORCE_INLINE		int	btlSliderConstraint::calculateSerializeBufferSize() const
{
	return sizeof(btlSliderConstraintData2);
}

	///fills the dataBuffer and returns the struct name (and 0 on failure)
SIMD_FORCE_INLINE	const char*	btlSliderConstraint::serialize(void* dataBuffer, btlSerializer* serializer) const
{

	btlSliderConstraintData2* sliderData = (btlSliderConstraintData2*) dataBuffer;
	btlTypedConstraint::serialize(&sliderData->m_typeConstraintData,serializer);

	m_frameInA.serialize(sliderData->m_rbAFrame);
	m_frameInB.serialize(sliderData->m_rbBFrame);

	sliderData->m_linearUpperLimit = m_upperLinLimit;
	sliderData->m_linearLowerLimit = m_lowerLinLimit;

	sliderData->m_angularUpperLimit = m_upperAngLimit;
	sliderData->m_angularLowerLimit = m_lowerAngLimit;

	sliderData->m_useLinearReferenceFrameA = m_useLinearReferenceFrameA;
	sliderData->m_useOffsetForConstraintFrame = m_useOffsetForConstraintFrame;

	return btlSliderConstraintDataName;
}



#endif //BT_SLIDER_CONSTRAINT_H

