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

///This file was written by Erwin Coumans

#include "btMultiBodyJointMotor.h"
#include "btMultiBody.h"
#include "btMultiBodyLinkCollider.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"


btlMultiBodyJointMotor::btlMultiBodyJointMotor(btlMultiBody* body, int link, btlScalar desiredVelocity, btlScalar maxMotorImpulse)
	:btlMultiBodyConstraint(body,body,link,body->getLink(link).m_parent,1,true),
	m_desiredVelocity(desiredVelocity)
{

	m_maxAppliedImpulse = maxMotorImpulse;
	// the data.m_jacobians never change, so may as well
    // initialize them here


}

void btlMultiBodyJointMotor::finalizeMultiDof()
{
	allocateJacobiansMultiDof();
	// note: we rely on the fact that data.m_jacobians are
	// always initialized to zero by the Constraint ctor
	int linkDoF = 0;
	unsigned int offset = 6 + (m_bodyA->getLink(m_linkA).m_dofOffset + linkDoF);

	// row 0: the lower bound
	// row 0: the lower bound
	jacobianA(0)[offset] = 1;

	m_numDofsFinalized = m_jacSizeBoth;
}

btlMultiBodyJointMotor::btlMultiBodyJointMotor(btlMultiBody* body, int link, int linkDoF, btlScalar desiredVelocity, btlScalar maxMotorImpulse)
	//:btMultiBodyConstraint(body,0,link,-1,1,true),
	:btlMultiBodyConstraint(body,body,link,body->getLink(link).m_parent,1,true),
	m_desiredVelocity(desiredVelocity)
{
	btlAssert(linkDoF < body->getLink(link).m_dofCount);

	m_maxAppliedImpulse = maxMotorImpulse;

}
btlMultiBodyJointMotor::~btlMultiBodyJointMotor()
{
}

int btlMultiBodyJointMotor::getIslandIdA() const
{
	btlMultiBodyLinkCollider* col = m_bodyA->getBaseCollider();
	if (col)
		return col->getIslandTag();
	for (int i=0;i<m_bodyA->getNumLinks();i++)
	{
		if (m_bodyA->getLink(i).m_collider)
			return m_bodyA->getLink(i).m_collider->getIslandTag();
	}
	return -1;
}

int btlMultiBodyJointMotor::getIslandIdB() const
{
	btlMultiBodyLinkCollider* col = m_bodyB->getBaseCollider();
	if (col)
		return col->getIslandTag();

	for (int i=0;i<m_bodyB->getNumLinks();i++)
	{
		col = m_bodyB->getLink(i).m_collider;
		if (col)
			return col->getIslandTag();
	}
	return -1;
}


void btlMultiBodyJointMotor::createConstraintRows(btlMultiBodyConstraintArray& constraintRows,
		btlMultiBodyJacobianData& data,
		const btlContactSolverInfo& infoGlobal)
{
    // only positions need to be updated -- data.m_jacobians and force
    // directions were set in the ctor and never change.
	
	if (m_numDofsFinalized != m_jacSizeBoth)
	{
        finalizeMultiDof();
	}

	//don't crash
	if (m_numDofsFinalized != m_jacSizeBoth)
		return;

	const btlScalar posError = 0;
	const btlVector3 dummy(0, 0, 0);

	for (int row=0;row<getNumRows();row++)
	{
		btlMultiBodySolverConstraint& constraintRow = constraintRows.expandNonInitializing();


		fillMultiBodyConstraint(constraintRow,data,jacobianA(row),jacobianB(row),dummy,dummy,dummy,posError,infoGlobal,-m_maxAppliedImpulse,m_maxAppliedImpulse,1,false,m_desiredVelocity);
		constraintRow.m_orgConstraint = this;
		constraintRow.m_orgDofIndex = row;
		{
			//expect either prismatic or revolute joint type for now
			btlAssert((m_bodyA->getLink(m_linkA).m_jointType == btlMultibodyLink::eRevolute)||(m_bodyA->getLink(m_linkA).m_jointType == btlMultibodyLink::ePrismatic));
			switch (m_bodyA->getLink(m_linkA).m_jointType)
			{
				case btlMultibodyLink::eRevolute:
				{
					constraintRow.m_contactNormal1.setZero();
					constraintRow.m_contactNormal2.setZero();
					btlVector3 revoluteAxisInWorld = btl_quatRotate(m_bodyA->getLink(m_linkA).m_cachedWorldTransform.getRotation(),m_bodyA->getLink(m_linkA).m_axes[0].m_topVec);
					constraintRow.m_relpos1CrossNormal=revoluteAxisInWorld;
					constraintRow.m_relpos2CrossNormal=-revoluteAxisInWorld;
					
					break;
				}
				case btlMultibodyLink::ePrismatic:
				{
					btlVector3 prismaticAxisInWorld = btl_quatRotate(m_bodyA->getLink(m_linkA).m_cachedWorldTransform.getRotation(),m_bodyA->getLink(m_linkA).m_axes[0].m_bottomVec);
					constraintRow.m_contactNormal1=prismaticAxisInWorld;
					constraintRow.m_contactNormal2=-prismaticAxisInWorld;
					constraintRow.m_relpos1CrossNormal.setZero();
					constraintRow.m_relpos2CrossNormal.setZero();
					
					break;
				}
				default:
				{
					btlAssert(0);
				}
			};
			
		}

	}

}

