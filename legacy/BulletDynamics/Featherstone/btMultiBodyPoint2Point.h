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

#ifndef BT_MULTIBODY_POINT2POINT_H
#define BT_MULTIBODY_POINT2POINT_H

#include "btMultiBodyConstraint.h"

//#define BTMBP2PCONSTRAINT_BLOCK_ANGULAR_MOTION_TEST

class btlMultiBodyPoint2Point : public btlMultiBodyConstraint
{
protected:

	btlRigidBody*	m_rigidBodyA;
	btlRigidBody*	m_rigidBodyB;
	btlVector3		m_pivotInA;
	btlVector3		m_pivotInB;


public:

	btlMultiBodyPoint2Point(btlMultiBody* body, int link, btlRigidBody* bodyB, const btlVector3& pivotInA, const btlVector3& pivotInB);
	btlMultiBodyPoint2Point(btlMultiBody* bodyA, int linkA, btlMultiBody* bodyB, int linkB, const btlVector3& pivotInA, const btlVector3& pivotInB);

	virtual ~btlMultiBodyPoint2Point();

	virtual void finalizeMultiDof();

	virtual int getIslandIdA() const;
	virtual int getIslandIdB() const;

	virtual void createConstraintRows(btlMultiBodyConstraintArray& constraintRows,
		btlMultiBodyJacobianData& data,
		const btlContactSolverInfo& infoGlobal);

	const btlVector3& getPivotInB() const
	{
		return m_pivotInB;
	}

	void setPivotInB(const btlVector3& pivotInB)
	{
		m_pivotInB = pivotInB;
	}

	virtual void debugDraw(class btlIDebugDraw* drawer);

};

#endif //BT_MULTIBODY_POINT2POINT_H
