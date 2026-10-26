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

#ifndef BT_SOLVE_2LINEAR_CONSTRAINT_H
#define BT_SOLVE_2LINEAR_CONSTRAINT_H

#include "LinearMath/btMatrix3x3.h"
#include "LinearMath/btVector3.h"


class btlRigidBody;



/// constraint class used for lateral tyre friction.
class	btlSolve2LinearConstraint
{
	btlScalar	m_tau;
	btlScalar	m_damping;

public:

	btlSolve2LinearConstraint(btlScalar tau,btlScalar damping)
	{
		m_tau = tau;
		m_damping = damping;
	}
	//
	// solve unilateral constraint (equality, direct method)
	//
	void resolveUnilateralPairConstraint(		
														   btlRigidBody* body0,
		btlRigidBody* body1,

		const btlMatrix3x3& world2A,
						const btlMatrix3x3& world2B,
						
						const btlVector3& invInertiaADiag,
						const btlScalar invMassA,
						const btlVector3& linvelA,const btlVector3& angvelA,
						const btlVector3& rel_posA1,
						const btlVector3& invInertiaBDiag,
						const btlScalar invMassB,
						const btlVector3& linvelB,const btlVector3& angvelB,
						const btlVector3& rel_posA2,

					  btlScalar depthA, const btlVector3& normalA, 
					  const btlVector3& rel_posB1,const btlVector3& rel_posB2,
					  btlScalar depthB, const btlVector3& normalB, 
					  btlScalar& imp0,btlScalar& imp1);


	//
	// solving 2x2 lcp problem (inequality, direct solution )
	//
	void resolveBilateralPairConstraint(
			btlRigidBody* body0,
						btlRigidBody* body1,
		const btlMatrix3x3& world2A,
						const btlMatrix3x3& world2B,
						
						const btlVector3& invInertiaADiag,
						const btlScalar invMassA,
						const btlVector3& linvelA,const btlVector3& angvelA,
						const btlVector3& rel_posA1,
						const btlVector3& invInertiaBDiag,
						const btlScalar invMassB,
						const btlVector3& linvelB,const btlVector3& angvelB,
						const btlVector3& rel_posA2,

					  btlScalar depthA, const btlVector3& normalA, 
					  const btlVector3& rel_posB1,const btlVector3& rel_posB2,
					  btlScalar depthB, const btlVector3& normalB, 
					  btlScalar& imp0,btlScalar& imp1);

/*
	void resolveAngularConstraint(	const btMatrix3x3& invInertiaAWS,
						const btScalar invMassA,
						const btVector3& linvelA,const btVector3& angvelA,
						const btVector3& rel_posA1,
						const btMatrix3x3& invInertiaBWS,
						const btScalar invMassB,
						const btVector3& linvelB,const btVector3& angvelB,
						const btVector3& rel_posA2,

					  btScalar depthA, const btVector3& normalA, 
					  const btVector3& rel_posB1,const btVector3& rel_posB2,
					  btScalar depthB, const btVector3& normalB, 
					  btScalar& imp0,btScalar& imp1);

*/

};

#endif //BT_SOLVE_2LINEAR_CONSTRAINT_H
