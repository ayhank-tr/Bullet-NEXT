/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2012 Advanced Micro Devices, Inc.  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/



#ifndef BT_GEAR_CONSTRAINT_H
#define BT_GEAR_CONSTRAINT_H

#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"


#ifdef BT_USE_DOUBLE_PRECISION
#define btlGearConstraintData	btlGearConstraintDoubleData
#define btlGearConstraintDataName	"btGearConstraintDoubleData"
#else
#define btlGearConstraintData	btlGearConstraintFloatData
#define btlGearConstraintDataName	"btGearConstraintFloatData"
#endif //BT_USE_DOUBLE_PRECISION



///The btGeatConstraint will couple the angular velocity for two bodies around given local axis and ratio.
///See Bullet/Demos/ConstraintDemo for an example use.
class btlGearConstraint : public btlTypedConstraint
{
protected:
	btlVector3	m_axisInA;
	btlVector3	m_axisInB;
	bool		m_useFrameA;
	btlScalar	m_ratio;

public:
	btlGearConstraint(btlRigidBody& rbA, btlRigidBody& rbB, const btlVector3& axisInA,const btlVector3& axisInB, btlScalar ratio=1.f);
	virtual ~btlGearConstraint ();

	///internal method used by the constraint solver, don't use them directly
	virtual void getInfo1 (btlConstraintInfo1* info);

	///internal method used by the constraint solver, don't use them directly
	virtual void getInfo2 (btlConstraintInfo2* info);

	void setAxisA(btlVector3& axisA) 
	{
		m_axisInA = axisA;
	}
	void setAxisB(btlVector3& axisB)
	{
		m_axisInB = axisB;
	}
	void setRatio(btlScalar ratio)
	{
		m_ratio = ratio;
	}
	const btlVector3& getAxisA() const
	{
		return m_axisInA;
	}
	const btlVector3& getAxisB() const
	{
		return m_axisInB;
	}
	btlScalar getRatio() const
	{
		return m_ratio;
	}


	virtual	void	setParam(int num, btlScalar value, int axis = -1) 
	{
		(void) num;
		(void) value;
		(void) axis;
		btlAssert(0);
	}

	///return the local value of parameter
	virtual	btlScalar getParam(int num, int axis = -1) const 
	{ 
		(void) num;
		(void) axis;
		btlAssert(0);
		return 0.f;
	}

	virtual	int	calculateSerializeBufferSize() const;

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual	const char*	serialize(void* dataBuffer, btlSerializer* serializer) const;
};




///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct btlGearConstraintFloatData
{
	btlTypedConstraintFloatData	m_typeConstraintData;

	btlVector3FloatData			m_axisInA;
	btlVector3FloatData			m_axisInB;

	float							m_ratio;
	char							m_padding[4];
};

struct btlGearConstraintDoubleData
{
	btlTypedConstraintDoubleData	m_typeConstraintData;

	btlVector3DoubleData			m_axisInA;
	btlVector3DoubleData			m_axisInB;

	double						m_ratio;
};

SIMD_FORCE_INLINE	int	btlGearConstraint::calculateSerializeBufferSize() const
{
	return sizeof(btlGearConstraintData);
}

	///fills the dataBuffer and returns the struct name (and 0 on failure)
SIMD_FORCE_INLINE	const char*	btlGearConstraint::serialize(void* dataBuffer, btlSerializer* serializer) const
{
	btlGearConstraintData* gear = (btlGearConstraintData*)dataBuffer;
	btlTypedConstraint::serialize(&gear->m_typeConstraintData,serializer);

	m_axisInA.serialize( gear->m_axisInA );
	m_axisInB.serialize( gear->m_axisInB );

	gear->m_ratio = m_ratio;

	return btlGearConstraintDataName;
}






#endif //BT_GEAR_CONSTRAINT_H
