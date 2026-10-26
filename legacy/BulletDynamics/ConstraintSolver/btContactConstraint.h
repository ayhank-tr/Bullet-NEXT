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

#ifndef BT_CONTACT_CONSTRAINT_H
#define BT_CONTACT_CONSTRAINT_H

#include "LinearMath/btVector3.h"
#include "btJacobianEntry.h"
#include "btTypedConstraint.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"

///btContactConstraint can be automatically created to solve contact constraints using the unified btTypedConstraint interface
ATTRIBUTE_ALIGNED16(class) btlContactConstraint : public btlTypedConstraint
{
protected:

	btlPersistentManifold m_contactManifold;

public:


	btlContactConstraint(btlPersistentManifold* contactManifold,btlRigidBody& rbA,btlRigidBody& rbB);

	void	setContactManifold(btlPersistentManifold* contactManifold);

	btlPersistentManifold* getContactManifold()
	{
		return &m_contactManifold;
	}

	const btlPersistentManifold* getContactManifold() const
	{
		return &m_contactManifold;
	}

	virtual ~btlContactConstraint();

	virtual void getInfo1 (btlConstraintInfo1* info);

	virtual void getInfo2 (btlConstraintInfo2* info);

	///obsolete methods
	virtual void	buildJacobian();


};

///very basic collision resolution without friction
btlScalar btl_resolveSingleCollision(btlRigidBody* body1, class btlCollisionObject* colObj2, const btlVector3& contactPositionWorld,const btlVector3& contactNormalOnB, const struct btlContactSolverInfo& solverInfo,btlScalar distance);


///resolveSingleBilateral is an obsolete methods used for vehicle friction between two dynamic objects
void btl_resolveSingleBilateral(btlRigidBody& body1, const btlVector3& pos1,
                      btlRigidBody& body2, const btlVector3& pos2,
                      btlScalar distance, const btlVector3& normal,btlScalar& impulse ,btlScalar timeStep);



#endif //BT_CONTACT_CONSTRAINT_H
