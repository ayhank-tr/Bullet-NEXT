/*
Bullet Continuous Collision Detection and Physics Library, http://bulletphysics.org
Copyright (C) 2006, 2007 Sony Computer Entertainment Inc. 

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_HINGE2_CONSTRAINT_H
#define BT_HINGE2_CONSTRAINT_H



#include "LinearMath/btVector3.h"
#include "btTypedConstraint.h"
#include "btGeneric6DofSpring2Constraint.h"



// Constraint similar to ODE Hinge2 Joint
// has 3 degrees of frredom:
// 2 rotational degrees of freedom, similar to Euler rotations around Z (axis 1) and X (axis 2)
// 1 translational (along axis Z) with suspension spring

ATTRIBUTE_ALIGNED16(class) btlHinge2Constraint : public btlGeneric6DofSpring2Constraint
{
protected:
	btlVector3	m_anchor;
	btlVector3	m_axis1;
	btlVector3	m_axis2;
public:
		BT_DECLARE_ALIGNED_ALLOCATOR();
		
	// constructor
	// anchor, axis1 and axis2 are in world coordinate system
	// axis1 must be orthogonal to axis2
    btlHinge2Constraint(btlRigidBody& rbA, btlRigidBody& rbB, btlVector3& anchor, btlVector3& axis1, btlVector3& axis2);
	// access
	const btlVector3& getAnchor() { return m_calculatedTransformA.getOrigin(); }
	const btlVector3& getAnchor2() { return m_calculatedTransformB.getOrigin(); }
	const btlVector3& getAxis1() { return m_axis1; }
	const btlVector3& getAxis2() { return m_axis2; }
	btlScalar getAngle1() { return getAngle(2); }
	btlScalar getAngle2() { return getAngle(0); }
	// limits
	void setUpperLimit(btlScalar ang1max) { setAngularUpperLimit(btlVector3(-1.f, 0.f, ang1max)); }
	void setLowerLimit(btlScalar ang1min) { setAngularLowerLimit(btlVector3( 1.f, 0.f, ang1min)); }
};



#endif // BT_HINGE2_CONSTRAINT_H

