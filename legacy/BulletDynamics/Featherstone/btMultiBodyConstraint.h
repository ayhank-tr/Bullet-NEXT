/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2013 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_MULTIBODY_CONSTRAINT_H
#define BT_MULTIBODY_CONSTRAINT_H

#include "LinearMath/btScalar.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "btMultiBody.h"

class btlMultiBody;
struct btlSolverInfo;

#include "btMultiBodySolverConstraint.h"

struct btlMultiBodyJacobianData
{
	btlAlignedObjectArray<btlScalar>		m_jacobians;
	btlAlignedObjectArray<btlScalar>		m_deltaVelocitiesUnitImpulse;	//holds the joint-space response of the corresp. tree to the test impulse in each constraint space dimension
	btlAlignedObjectArray<btlScalar>		m_deltaVelocities;				//holds joint-space vectors of all the constrained trees accumulating the effect of corrective impulses applied in SI
	btlAlignedObjectArray<btlScalar>		scratch_r;
	btlAlignedObjectArray<btlVector3>		scratch_v;
	btlAlignedObjectArray<btlMatrix3x3>	scratch_m;
	btlAlignedObjectArray<btlSolverBody>*	m_solverBodyPool;
	int									m_fixedBodyId;

};


class btlMultiBodyConstraint
{
protected:

	btlMultiBody*	m_bodyA;
    btlMultiBody*	m_bodyB;
    int				m_linkA;
    int				m_linkB;

    int				m_numRows;
    int				m_jacSizeA;
    int				m_jacSizeBoth;
    int				m_posOffset;

	bool			m_isUnilateral;
	int				m_numDofsFinalized;
	btlScalar		m_maxAppliedImpulse;


    // warning: the data block lay out is not consistent for all constraints
    // data block laid out as follows:
    // cached impulses. (one per row.)
    // jacobians. (interleaved, row1 body1 then row1 body2 then row2 body 1 etc)
    // positions. (one per row.)
    btlAlignedObjectArray<btlScalar> m_data;

	void	applyDeltaVee(btlMultiBodyJacobianData& data, btlScalar* delta_vee, btlScalar impulse, int velocityIndex, int ndof);

	btlScalar fillMultiBodyConstraint(btlMultiBodySolverConstraint& solverConstraint,
																btlMultiBodyJacobianData& data,
																btlScalar* jacOrgA, btlScalar* jacOrgB,
																const btlVector3& contactNormalOnB,
																const btlVector3& posAworld, const btlVector3& posBworld,
																btlScalar posError,
																const btlContactSolverInfo& infoGlobal,
																btlScalar lowerLimit, btlScalar upperLimit,
																btlScalar relaxation = 1.f,
																bool isFriction = false, btlScalar desiredVelocity=0, btlScalar cfmSlip=0);

public:

	btlMultiBodyConstraint(btlMultiBody* bodyA,btlMultiBody* bodyB,int linkA, int linkB, int numRows, bool isUnilateral);
	virtual ~btlMultiBodyConstraint();

	void updateJacobianSizes();
	void allocateJacobiansMultiDof();

	virtual void finalizeMultiDof()=0;

	virtual int getIslandIdA() const =0;
	virtual int getIslandIdB() const =0;

	virtual void createConstraintRows(btlMultiBodyConstraintArray& constraintRows,
		btlMultiBodyJacobianData& data,
		const btlContactSolverInfo& infoGlobal)=0;

	int	getNumRows() const
	{
		return m_numRows;
	}

	btlMultiBody*	getMultiBodyA()
	{
		return m_bodyA;
	}
    btlMultiBody*	getMultiBodyB()
	{
		return m_bodyB;
	}

	void	internalSetAppliedImpulse(int dof, btlScalar appliedImpulse)
	{
		btlAssert(dof>=0);
		btlAssert(dof < getNumRows());
		m_data[dof] = appliedImpulse;
	}
	
	btlScalar	getAppliedImpulse(int dof)
	{
		btlAssert(dof>=0);
		btlAssert(dof < getNumRows());
		return m_data[dof];
	}
	// current constraint position
    // constraint is pos >= 0 for unilateral, or pos = 0 for bilateral
    // NOTE: ignored position for friction rows.
    btlScalar getPosition(int row) const
	{
		return m_data[m_posOffset + row];
	}

    void setPosition(int row, btlScalar pos)
	{
		m_data[m_posOffset + row] = pos;
	}


	bool isUnilateral() const
	{
		return m_isUnilateral;
	}

	// jacobian blocks.
    // each of size 6 + num_links. (jacobian2 is null if no body2.)
    // format: 3 'omega' coefficients, 3 'v' coefficients, then the 'qdot' coefficients.
    btlScalar* jacobianA(int row)
	{
		return &m_data[m_numRows + row * m_jacSizeBoth];
	}
    const btlScalar* jacobianA(int row) const
	{
		return &m_data[m_numRows + (row * m_jacSizeBoth)];
	}
    btlScalar* jacobianB(int row)
	{
		return &m_data[m_numRows + (row * m_jacSizeBoth) + m_jacSizeA];
	}
    const btlScalar* jacobianB(int row) const
	{
		return &m_data[m_numRows + (row * m_jacSizeBoth) + m_jacSizeA];
	}

	btlScalar	getMaxAppliedImpulse() const
	{
		return m_maxAppliedImpulse;
	}
	void	setMaxAppliedImpulse(btlScalar maxImp)
	{
		m_maxAppliedImpulse = maxImp;
	}

	virtual void debugDraw(class btlIDebugDraw* drawer)=0;

};

#endif //BT_MULTIBODY_CONSTRAINT_H

