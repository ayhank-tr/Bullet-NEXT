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

#include "btMultiBodyPoint2Point.h"
#include "btMultiBodyLinkCollider.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "LinearMath/btIDebugDraw.h"

#ifndef BTMBP2PCONSTRAINT_BLOCK_ANGULAR_MOTION_TEST
	#define BTMBP2PCONSTRAINT_DIM 3
#else
	#define BTMBP2PCONSTRAINT_DIM 6
#endif

btlMultiBodyPoint2Point::btlMultiBodyPoint2Point(btlMultiBody* body, int link, btlRigidBody* bodyB, const btlVector3& pivotInA, const btlVector3& pivotInB)
	:btlMultiBodyConstraint(body,0,link,-1,BTMBP2PCONSTRAINT_DIM,false),
	m_rigidBodyA(0),
	m_rigidBodyB(bodyB),
	m_pivotInA(pivotInA),
	m_pivotInB(pivotInB)
{
    m_data.resize(BTMBP2PCONSTRAINT_DIM);//at least store the applied impulses
}

btlMultiBodyPoint2Point::btlMultiBodyPoint2Point(btlMultiBody* bodyA, int linkA, btlMultiBody* bodyB, int linkB, const btlVector3& pivotInA, const btlVector3& pivotInB)
	:btlMultiBodyConstraint(bodyA,bodyB,linkA,linkB,BTMBP2PCONSTRAINT_DIM,false),
	m_rigidBodyA(0),
	m_rigidBodyB(0),
	m_pivotInA(pivotInA),
	m_pivotInB(pivotInB)
{
    m_data.resize(BTMBP2PCONSTRAINT_DIM);//at least store the applied impulses
}

void btlMultiBodyPoint2Point::finalizeMultiDof()
{
	//not implemented yet
	btlAssert(0);
}

btlMultiBodyPoint2Point::~btlMultiBodyPoint2Point()
{
}


int btlMultiBodyPoint2Point::getIslandIdA() const
{
	if (m_rigidBodyA)
		return m_rigidBodyA->getIslandTag();

	if (m_bodyA)
	{
		btlMultiBodyLinkCollider* col = m_bodyA->getBaseCollider();
		if (col)
			return col->getIslandTag();
		for (int i=0;i<m_bodyA->getNumLinks();i++)
		{
			if (m_bodyA->getLink(i).m_collider)
				return m_bodyA->getLink(i).m_collider->getIslandTag();
		}
	}
	return -1;
}

int btlMultiBodyPoint2Point::getIslandIdB() const
{
	if (m_rigidBodyB)
		return m_rigidBodyB->getIslandTag();
	if (m_bodyB)
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
	}
	return -1;
}



void btlMultiBodyPoint2Point::createConstraintRows(btlMultiBodyConstraintArray& constraintRows,
		btlMultiBodyJacobianData& data,
		const btlContactSolverInfo& infoGlobal)
{

//	int i=1;
int numDim = BTMBP2PCONSTRAINT_DIM;
	for (int i=0;i<numDim;i++)
	{

		btlMultiBodySolverConstraint& constraintRow = constraintRows.expandNonInitializing();
        //memset(&constraintRow,0xffffffff,sizeof(btMultiBodySolverConstraint));
	constraintRow.m_orgConstraint = this;
	constraintRow.m_orgDofIndex = i;
        constraintRow.m_relpos1CrossNormal.setValue(0,0,0);
        constraintRow.m_contactNormal1.setValue(0,0,0);
        constraintRow.m_relpos2CrossNormal.setValue(0,0,0);
        constraintRow.m_contactNormal2.setValue(0,0,0);
        constraintRow.m_angularComponentA.setValue(0,0,0);
        constraintRow.m_angularComponentB.setValue(0,0,0);

		constraintRow.m_solverBodyIdA = data.m_fixedBodyId;
		constraintRow.m_solverBodyIdB = data.m_fixedBodyId;

		btlVector3 contactNormalOnB(0,0,0);
#ifndef BTMBP2PCONSTRAINT_BLOCK_ANGULAR_MOTION_TEST
		contactNormalOnB[i] = -1;
#else
		contactNormalOnB[i%3] = -1;
#endif


		 // Convert local points back to world
		btlVector3 pivotAworld = m_pivotInA;
		if (m_rigidBodyA)
		{

			constraintRow.m_solverBodyIdA = m_rigidBodyA->getCompanionId();
			pivotAworld = m_rigidBodyA->getCenterOfMassTransform()*m_pivotInA;
		} else
		{
			if (m_bodyA)
				pivotAworld = m_bodyA->localPosToWorld(m_linkA, m_pivotInA);
		}
		btlVector3 pivotBworld = m_pivotInB;
		if (m_rigidBodyB)
		{
			constraintRow.m_solverBodyIdB = m_rigidBodyB->getCompanionId();
			pivotBworld = m_rigidBodyB->getCenterOfMassTransform()*m_pivotInB;
		} else
		{
			if (m_bodyB)
				pivotBworld = m_bodyB->localPosToWorld(m_linkB, m_pivotInB);

		}

		btlScalar posError = i < 3 ? (pivotAworld-pivotBworld).dot(contactNormalOnB) : 0;

#ifndef BTMBP2PCONSTRAINT_BLOCK_ANGULAR_MOTION_TEST


		fillMultiBodyConstraint(constraintRow, data, 0, 0,
															contactNormalOnB, pivotAworld, pivotBworld,						//sucks but let it be this way "for the time being"
															posError,
															infoGlobal,
															-m_maxAppliedImpulse, m_maxAppliedImpulse
															);
    //@todo: support the case of btMultiBody versus btRigidBody,
    //see btPoint2PointConstraint::getInfo2NonVirtual
#else
		const btlVector3 dummy(0, 0, 0);

		btlAssert(m_bodyA->isMultiDof());

		btlScalar* jac1 = jacobianA(i);
		const btlVector3 &normalAng = i >= 3 ? contactNormalOnB : dummy;
		const btlVector3 &normalLin = i < 3 ? contactNormalOnB : dummy;

		m_bodyA->filConstraintJacobianMultiDof(m_linkA, pivotAworld, normalAng, normalLin, jac1, data.scratch_r, data.scratch_v, data.scratch_m);

		fillMultiBodyConstraint(constraintRow, data, jac1, 0,
													dummy, dummy, dummy,						//sucks but let it be this way "for the time being"
													posError,
													infoGlobal,
													-m_maxAppliedImpulse, m_maxAppliedImpulse
													);
#endif
	}
}

void btlMultiBodyPoint2Point::debugDraw(class btlIDebugDraw* drawer)
{
	btlTransform tr;
	tr.setIdentity();

	if (m_rigidBodyA)
	{
		btlVector3 pivot = m_rigidBodyA->getCenterOfMassTransform() * m_pivotInA;
		tr.setOrigin(pivot);
		drawer->drawTransform(tr, 0.1);
	}
	if (m_bodyA)
	{
		btlVector3 pivotAworld = m_bodyA->localPosToWorld(m_linkA, m_pivotInA);
		tr.setOrigin(pivotAworld);
		drawer->drawTransform(tr, 0.1);
	}
	if (m_rigidBodyB)
	{
		// that ideally should draw the same frame
		btlVector3 pivot = m_rigidBodyB->getCenterOfMassTransform() * m_pivotInB;
		tr.setOrigin(pivot);
		drawer->drawTransform(tr, 0.1);
	}
	if (m_bodyB)
	{
		btlVector3 pivotBworld = m_bodyB->localPosToWorld(m_linkB, m_pivotInB);
		tr.setOrigin(pivotBworld);
		drawer->drawTransform(tr, 0.1);
	}
}
