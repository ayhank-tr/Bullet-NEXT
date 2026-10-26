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


#include "btContactConstraint.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "LinearMath/btVector3.h"
#include "btJacobianEntry.h"
#include "btContactSolverInfo.h"
#include "LinearMath/btMinMax.h"
#include "BulletCollision/NarrowPhaseCollision/btManifoldPoint.h"



btlContactConstraint::btlContactConstraint(btlPersistentManifold* contactManifold,btlRigidBody& rbA,btlRigidBody& rbB)
:btlTypedConstraint(CONTACT_CONSTRAINT_TYPE,rbA,rbB),
	m_contactManifold(*contactManifold)
{

}

btlContactConstraint::~btlContactConstraint()
{

}

void	btlContactConstraint::setContactManifold(btlPersistentManifold* contactManifold)
{
	m_contactManifold = *contactManifold;
}

void btlContactConstraint::getInfo1 (btlConstraintInfo1* info)
{

}

void btlContactConstraint::getInfo2 (btlConstraintInfo2* info)
{

}

void	btlContactConstraint::buildJacobian()
{

}





#include "btContactConstraint.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "LinearMath/btVector3.h"
#include "btJacobianEntry.h"
#include "btContactSolverInfo.h"
#include "LinearMath/btMinMax.h"
#include "BulletCollision/NarrowPhaseCollision/btManifoldPoint.h"



//response  between two dynamic objects without friction and no restitution, assuming 0 penetration depth
btlScalar btl_resolveSingleCollision(
        btlRigidBody* body1,
        btlCollisionObject* colObj2,
		const btlVector3& contactPositionWorld,
		const btlVector3& contactNormalOnB,
        const btlContactSolverInfo& solverInfo,
		btlScalar distance)
{
	btlRigidBody* body2 = btlRigidBody::upcast(colObj2);
    
	
    const btlVector3& normal = contactNormalOnB;

    btlVector3 rel_pos1 = contactPositionWorld - body1->getWorldTransform().getOrigin(); 
    btlVector3 rel_pos2 = contactPositionWorld - colObj2->getWorldTransform().getOrigin();
    
    btlVector3 vel1 = body1->getVelocityInLocalPoint(rel_pos1);
	btlVector3 vel2 = body2? body2->getVelocityInLocalPoint(rel_pos2) : btlVector3(0,0,0);
    btlVector3 vel = vel1 - vel2;
    btlScalar rel_vel;
    rel_vel = normal.dot(vel);
    
    btlScalar combinedRestitution = 0.f;
    btlScalar restitution = combinedRestitution* -rel_vel;

    btlScalar positionalError = solverInfo.m_erp *-distance /solverInfo.m_timeStep ;
    btlScalar velocityError = -(1.0f + restitution) * rel_vel;// * damping;
	btlScalar denom0 = body1->computeImpulseDenominator(contactPositionWorld,normal);
	btlScalar denom1 = body2? body2->computeImpulseDenominator(contactPositionWorld,normal) : 0.f;
	btlScalar relaxation = 1.f;
	btlScalar jacDiagABInv = relaxation/(denom0+denom1);

    btlScalar penetrationImpulse = positionalError * jacDiagABInv;
    btlScalar velocityImpulse = velocityError * jacDiagABInv;

    btlScalar normalImpulse = penetrationImpulse+velocityImpulse;
    normalImpulse = 0.f > normalImpulse ? 0.f: normalImpulse;

	body1->applyImpulse(normal*(normalImpulse), rel_pos1);
    if (body2)
		body2->applyImpulse(-normal*(normalImpulse), rel_pos2);
    
    return normalImpulse;
}


//bilateral constraint between two dynamic objects
void btl_resolveSingleBilateral(btlRigidBody& body1, const btlVector3& pos1,
                      btlRigidBody& body2, const btlVector3& pos2,
                      btlScalar distance, const btlVector3& normal,btlScalar& impulse ,btlScalar timeStep)
{
	(void)timeStep;
	(void)distance;


	btlScalar normalLenSqr = normal.length2();
	btlAssert(btlFabs(normalLenSqr) < btlScalar(1.1));
	if (normalLenSqr > btlScalar(1.1))
	{
		impulse = btlScalar(0.);
		return;
	}
	btlVector3 rel_pos1 = pos1 - body1.getCenterOfMassPosition(); 
	btlVector3 rel_pos2 = pos2 - body2.getCenterOfMassPosition();
	//this jacobian entry could be re-used for all iterations
	
	btlVector3 vel1 = body1.getVelocityInLocalPoint(rel_pos1);
	btlVector3 vel2 = body2.getVelocityInLocalPoint(rel_pos2);
	btlVector3 vel = vel1 - vel2;
	

	   btlJacobianEntry jac(body1.getCenterOfMassTransform().getBasis().transpose(),
		body2.getCenterOfMassTransform().getBasis().transpose(),
		rel_pos1,rel_pos2,normal,body1.getInvInertiaDiagLocal(),body1.getInvMass(),
		body2.getInvInertiaDiagLocal(),body2.getInvMass());

	btlScalar jacDiagAB = jac.getDiagonal();
	btlScalar jacDiagABInv = btlScalar(1.) / jacDiagAB;
	
	  btlScalar rel_vel = jac.getRelativeVelocity(
		body1.getLinearVelocity(),
		body1.getCenterOfMassTransform().getBasis().transpose() * body1.getAngularVelocity(),
		body2.getLinearVelocity(),
		body2.getCenterOfMassTransform().getBasis().transpose() * body2.getAngularVelocity()); 



	rel_vel = normal.dot(vel);
	
	//todo: move this into proper structure
	btlScalar contactDamping = btlScalar(0.2);

#ifdef ONLY_USE_LINEAR_MASS
	btlScalar massTerm = btlScalar(1.) / (body1.getInvMass() + body2.getInvMass());
	impulse = - contactDamping * rel_vel * massTerm;
#else	
	btlScalar velocityImpulse = -contactDamping * rel_vel * jacDiagABInv;
	impulse = velocityImpulse;
#endif
}




