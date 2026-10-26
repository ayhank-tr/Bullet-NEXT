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

#ifndef BT_POINT2POINTCONSTRAINT_H
#define BT_POINT2POINTCONSTRAINT_H

#include "LinearMath/btVector3.h"
#include "btJacobianEntry.h"
#include "btTypedConstraint.h"

class btlRigidBody;


#ifdef BT_USE_DOUBLE_PRECISION
#define btlPoint2PointConstraintData2	btlPoint2PointConstraintDoubleData2
#define btlPoint2PointConstraintDataName	"btPoint2PointConstraintDoubleData2"
#else
#define btlPoint2PointConstraintData2	btlPoint2PointConstraintFloatData
#define btlPoint2PointConstraintDataName	"btPoint2PointConstraintFloatData"
#endif //BT_USE_DOUBLE_PRECISION

struct	btlConstraintSetting
{
	btlConstraintSetting()	:
		m_tau(btlScalar(0.3)),
		m_damping(btlScalar(1.)),
		m_impulseClamp(btlScalar(0.))
	{
	}
	btlScalar		m_tau;
	btlScalar		m_damping;
	btlScalar		m_impulseClamp;
};

enum btlPoint2PointFlags
{
	BT_P2P_FLAGS_ERP = 1,
	BT_P2P_FLAGS_CFM = 2
};

/// point to point constraint between two rigidbodies each with a pivotpoint that descibes the 'ballsocket' location in local space
ATTRIBUTE_ALIGNED16(class) btlPoint2PointConstraint : public btlTypedConstraint
{
#ifdef IN_PARALLELL_SOLVER
public:
#endif
	btlJacobianEntry	m_jac[3]; //3 orthogonal linear constraints
	
	btlVector3	m_pivotInA;
	btlVector3	m_pivotInB;
	
	int			m_flags;
	btlScalar	m_erp;
	btlScalar	m_cfm;
	
public:

	BT_DECLARE_ALIGNED_ALLOCATOR();

	///for backwards compatibility during the transition to 'getInfo/getInfo2'
	bool		m_useSolveConstraintObsolete;

	btlConstraintSetting	m_setting;

	btlPoint2PointConstraint(btlRigidBody& rbA,btlRigidBody& rbB, const btlVector3& pivotInA,const btlVector3& pivotInB);

	btlPoint2PointConstraint(btlRigidBody& rbA,const btlVector3& pivotInA);


	virtual void	buildJacobian();

	virtual void getInfo1 (btlConstraintInfo1* info);

	void getInfo1NonVirtual (btlConstraintInfo1* info);

	virtual void getInfo2 (btlConstraintInfo2* info);

	void getInfo2NonVirtual (btlConstraintInfo2* info, const btlTransform& body0_trans, const btlTransform& body1_trans);

	void	updateRHS(btlScalar	timeStep);

	void	setPivotA(const btlVector3& pivotA)
	{
		m_pivotInA = pivotA;
	}

	void	setPivotB(const btlVector3& pivotB)
	{
		m_pivotInB = pivotB;
	}

	const btlVector3& getPivotInA() const
	{
		return m_pivotInA;
	}

	const btlVector3& getPivotInB() const
	{
		return m_pivotInB;
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
struct	btlPoint2PointConstraintFloatData
{
	btlTypedConstraintData	m_typeConstraintData;
	btlVector3FloatData	m_pivotInA;
	btlVector3FloatData	m_pivotInB;
};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct	btlPoint2PointConstraintDoubleData2
{
	btlTypedConstraintDoubleData	m_typeConstraintData;
	btlVector3DoubleData	m_pivotInA;
	btlVector3DoubleData	m_pivotInB;
};

#ifdef BT_BACKWARDS_COMPATIBLE_SERIALIZATION
///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
///this structure is not used, except for loading pre-2.82 .bullet files
///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct	btlPoint2PointConstraintDoubleData
{
	btlTypedConstraintData	m_typeConstraintData;
	btlVector3DoubleData	m_pivotInA;
	btlVector3DoubleData	m_pivotInB;
};
#endif //BT_BACKWARDS_COMPATIBLE_SERIALIZATION


SIMD_FORCE_INLINE	int	btlPoint2PointConstraint::calculateSerializeBufferSize() const
{
	return sizeof(btlPoint2PointConstraintData2);

}

	///fills the dataBuffer and returns the struct name (and 0 on failure)
SIMD_FORCE_INLINE	const char*	btlPoint2PointConstraint::serialize(void* dataBuffer, btlSerializer* serializer) const
{
	btlPoint2PointConstraintData2* p2pData = (btlPoint2PointConstraintData2*)dataBuffer;

	btlTypedConstraint::serialize(&p2pData->m_typeConstraintData,serializer);
	m_pivotInA.serialize(p2pData->m_pivotInA);
	m_pivotInB.serialize(p2pData->m_pivotInB);

	return btlPoint2PointConstraintDataName;
}

#endif //BT_POINT2POINTCONSTRAINT_H
