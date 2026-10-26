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
/*
2007-09-09
Refactored by Francisco Le?n
email: projectileman@yahoo.com
http://gimpact.sf.net
*/

#include "btGeneric6DofConstraint.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "LinearMath/btTransformUtil.h"
#include "LinearMath/btTransformUtil.h"
#include <new>



#define D6_USE_OBSOLETE_METHOD false
#define D6_USE_FRAME_OFFSET true






btlGeneric6DofConstraint::btlGeneric6DofConstraint(btlRigidBody& rbA, btlRigidBody& rbB, const btlTransform& frameInA, const btlTransform& frameInB, bool useLinearReferenceFrameA)
: btlTypedConstraint(D6_CONSTRAINT_TYPE, rbA, rbB)
, m_frameInA(frameInA)
, m_frameInB(frameInB),
m_useLinearReferenceFrameA(useLinearReferenceFrameA),
m_useOffsetForConstraintFrame(D6_USE_FRAME_OFFSET),
m_flags(0),
m_useSolveConstraintObsolete(D6_USE_OBSOLETE_METHOD)
{
	calculateTransforms();
}



btlGeneric6DofConstraint::btlGeneric6DofConstraint(btlRigidBody& rbB, const btlTransform& frameInB, bool useLinearReferenceFrameB)
        : btlTypedConstraint(D6_CONSTRAINT_TYPE, getFixedBody(), rbB),
		m_frameInB(frameInB),
		m_useLinearReferenceFrameA(useLinearReferenceFrameB),
		m_useOffsetForConstraintFrame(D6_USE_FRAME_OFFSET),
		m_flags(0),
		m_useSolveConstraintObsolete(false)
{
	///not providing rigidbody A means implicitly using worldspace for body A
	m_frameInA = rbB.getCenterOfMassTransform() * m_frameInB;
	calculateTransforms();
}




#define GENERIC_D6_DISABLE_WARMSTARTING 1



btlScalar btlGetMatrixElem(const btlMatrix3x3& mat, int index);
btlScalar btlGetMatrixElem(const btlMatrix3x3& mat, int index)
{
	int i = index%3;
	int j = index/3;
	return mat[i][j];
}



///MatrixToEulerXYZ from http://www.geometrictools.com/LibFoundation/Mathematics/Wm4Matrix3.inl.html
bool	btl_matrixToEulerXYZ(const btlMatrix3x3& mat,btlVector3& xyz);
bool	btl_matrixToEulerXYZ(const btlMatrix3x3& mat,btlVector3& xyz)
{
	//	// rot =  cy*cz          -cy*sz           sy
	//	//        cz*sx*sy+cx*sz  cx*cz-sx*sy*sz -cy*sx
	//	//       -cx*cz*sy+sx*sz  cz*sx+cx*sy*sz  cx*cy
	//

	btlScalar fi = btlGetMatrixElem(mat,2);
	if (fi < btlScalar(1.0f))
	{
		if (fi > btlScalar(-1.0f))
		{
			xyz[0] = btlAtan2(-btlGetMatrixElem(mat,5),btlGetMatrixElem(mat,8));
			xyz[1] = btlAsin(btlGetMatrixElem(mat,2));
			xyz[2] = btlAtan2(-btlGetMatrixElem(mat,1),btlGetMatrixElem(mat,0));
			return true;
		}
		else
		{
			// WARNING.  Not unique.  XA - ZA = -atan2(r10,r11)
			xyz[0] = -btlAtan2(btlGetMatrixElem(mat,3),btlGetMatrixElem(mat,4));
			xyz[1] = -SIMD_HALF_PI;
			xyz[2] = btlScalar(0.0);
			return false;
		}
	}
	else
	{
		// WARNING.  Not unique.  XAngle + ZAngle = atan2(r10,r11)
		xyz[0] = btlAtan2(btlGetMatrixElem(mat,3),btlGetMatrixElem(mat,4));
		xyz[1] = SIMD_HALF_PI;
		xyz[2] = 0.0;
	}
	return false;
}

//////////////////////////// btRotationalLimitMotor ////////////////////////////////////

int btlRotationalLimitMotor::testLimitValue(btlScalar test_value)
{
	if(m_loLimit>m_hiLimit)
	{
		m_currentLimit = 0;//Free from violation
		return 0;
	}
	if (test_value < m_loLimit)
	{
		m_currentLimit = 1;//low limit violation
		m_currentLimitError =  test_value - m_loLimit;
		if(m_currentLimitError>SIMD_PI) 
			m_currentLimitError-=SIMD_2_PI;
		else if(m_currentLimitError<-SIMD_PI) 
			m_currentLimitError+=SIMD_2_PI;
		return 1;
	}
	else if (test_value> m_hiLimit)
	{
		m_currentLimit = 2;//High limit violation
		m_currentLimitError = test_value - m_hiLimit;
		if(m_currentLimitError>SIMD_PI) 
			m_currentLimitError-=SIMD_2_PI;
		else if(m_currentLimitError<-SIMD_PI) 
			m_currentLimitError+=SIMD_2_PI;
		return 2;
	};

	m_currentLimit = 0;//Free from violation
	return 0;

}



btlScalar btlRotationalLimitMotor::solveAngularLimits(
	btlScalar timeStep,btlVector3& axis,btlScalar jacDiagABInv,
	btlRigidBody * body0, btlRigidBody * body1 )
{
	if (needApplyTorques()==false) return 0.0f;

	btlScalar target_velocity = m_targetVelocity;
	btlScalar maxMotorForce = m_maxMotorForce;

	//current error correction
	if (m_currentLimit!=0)
	{
		target_velocity = -m_stopERP*m_currentLimitError/(timeStep);
		maxMotorForce = m_maxLimitForce;
	}

	maxMotorForce *= timeStep;

	// current velocity difference

	btlVector3 angVelA = body0->getAngularVelocity();
	btlVector3 angVelB = body1->getAngularVelocity();

	btlVector3 vel_diff;
	vel_diff = angVelA-angVelB;



	btlScalar rel_vel = axis.dot(vel_diff);

	// correction velocity
	btlScalar motor_relvel = m_limitSoftness*(target_velocity  - m_damping*rel_vel);


	if ( motor_relvel < SIMD_EPSILON && motor_relvel > -SIMD_EPSILON  )
	{
		return 0.0f;//no need for applying force
	}


	// correction impulse
	btlScalar unclippedMotorImpulse = (1+m_bounce)*motor_relvel*jacDiagABInv;

	// clip correction impulse
	btlScalar clippedMotorImpulse;

	///@todo: should clip against accumulated impulse
	if (unclippedMotorImpulse>0.0f)
	{
		clippedMotorImpulse =  unclippedMotorImpulse > maxMotorForce? maxMotorForce: unclippedMotorImpulse;
	}
	else
	{
		clippedMotorImpulse =  unclippedMotorImpulse < -maxMotorForce ? -maxMotorForce: unclippedMotorImpulse;
	}


	// sort with accumulated impulses
	btlScalar	lo = btlScalar(-BT_LARGE_FLOAT);
	btlScalar	hi = btlScalar(BT_LARGE_FLOAT);

	btlScalar oldaccumImpulse = m_accumulatedImpulse;
	btlScalar sum = oldaccumImpulse + clippedMotorImpulse;
	m_accumulatedImpulse = sum > hi ? btlScalar(0.) : sum < lo ? btlScalar(0.) : sum;

	clippedMotorImpulse = m_accumulatedImpulse - oldaccumImpulse;

	btlVector3 motorImp = clippedMotorImpulse * axis;

	body0->applyTorqueImpulse(motorImp);
	body1->applyTorqueImpulse(-motorImp);

	return clippedMotorImpulse;


}

//////////////////////////// End btRotationalLimitMotor ////////////////////////////////////




//////////////////////////// btTranslationalLimitMotor ////////////////////////////////////


int btlTranslationalLimitMotor::testLimitValue(int limitIndex, btlScalar test_value)
{
	btlScalar loLimit = m_lowerLimit[limitIndex];
	btlScalar hiLimit = m_upperLimit[limitIndex];
	if(loLimit > hiLimit)
	{
		m_currentLimit[limitIndex] = 0;//Free from violation
		m_currentLimitError[limitIndex] = btlScalar(0.f);
		return 0;
	}

	if (test_value < loLimit)
	{
		m_currentLimit[limitIndex] = 2;//low limit violation
		m_currentLimitError[limitIndex] =  test_value - loLimit;
		return 2;
	}
	else if (test_value> hiLimit)
	{
		m_currentLimit[limitIndex] = 1;//High limit violation
		m_currentLimitError[limitIndex] = test_value - hiLimit;
		return 1;
	};

	m_currentLimit[limitIndex] = 0;//Free from violation
	m_currentLimitError[limitIndex] = btlScalar(0.f);
	return 0;
}



btlScalar btlTranslationalLimitMotor::solveLinearAxis(
	btlScalar timeStep,
	btlScalar jacDiagABInv,
	btlRigidBody& body1,const btlVector3 &pointInA,
	btlRigidBody& body2,const btlVector3 &pointInB,
	int limit_index,
	const btlVector3 & axis_normal_on_a,
	const btlVector3 & anchorPos)
{

	///find relative velocity
	//    btVector3 rel_pos1 = pointInA - body1.getCenterOfMassPosition();
	//    btVector3 rel_pos2 = pointInB - body2.getCenterOfMassPosition();
	btlVector3 rel_pos1 = anchorPos - body1.getCenterOfMassPosition();
	btlVector3 rel_pos2 = anchorPos - body2.getCenterOfMassPosition();

	btlVector3 vel1 = body1.getVelocityInLocalPoint(rel_pos1);
	btlVector3 vel2 = body2.getVelocityInLocalPoint(rel_pos2);
	btlVector3 vel = vel1 - vel2;

	btlScalar rel_vel = axis_normal_on_a.dot(vel);



	/// apply displacement correction

	//positional error (zeroth order error)
	btlScalar depth = -(pointInA - pointInB).dot(axis_normal_on_a);
	btlScalar	lo = btlScalar(-BT_LARGE_FLOAT);
	btlScalar	hi = btlScalar(BT_LARGE_FLOAT);

	btlScalar minLimit = m_lowerLimit[limit_index];
	btlScalar maxLimit = m_upperLimit[limit_index];

	//handle the limits
	if (minLimit < maxLimit)
	{
		{
			if (depth > maxLimit)
			{
				depth -= maxLimit;
				lo = btlScalar(0.);

			}
			else
			{
				if (depth < minLimit)
				{
					depth -= minLimit;
					hi = btlScalar(0.);
				}
				else
				{
					return 0.0f;
				}
			}
		}
	}

	btlScalar normalImpulse= m_limitSoftness*(m_restitution*depth/timeStep - m_damping*rel_vel) * jacDiagABInv;




	btlScalar oldNormalImpulse = m_accumulatedImpulse[limit_index];
	btlScalar sum = oldNormalImpulse + normalImpulse;
	m_accumulatedImpulse[limit_index] = sum > hi ? btlScalar(0.) : sum < lo ? btlScalar(0.) : sum;
	normalImpulse = m_accumulatedImpulse[limit_index] - oldNormalImpulse;

	btlVector3 impulse_vector = axis_normal_on_a * normalImpulse;
	body1.applyImpulse( impulse_vector, rel_pos1);
	body2.applyImpulse(-impulse_vector, rel_pos2);

	

	return normalImpulse;
}

//////////////////////////// btTranslationalLimitMotor ////////////////////////////////////

void btlGeneric6DofConstraint::calculateAngleInfo()
{
	btlMatrix3x3 relative_frame = m_calculatedTransformA.getBasis().inverse()*m_calculatedTransformB.getBasis();
	btl_matrixToEulerXYZ(relative_frame,m_calculatedAxisAngleDiff);
	// in euler angle mode we do not actually constrain the angular velocity
	// along the axes axis[0] and axis[2] (although we do use axis[1]) :
	//
	//    to get			constrain w2-w1 along		...not
	//    ------			---------------------		------
	//    d(angle[0])/dt = 0	ax[1] x ax[2]			ax[0]
	//    d(angle[1])/dt = 0	ax[1]
	//    d(angle[2])/dt = 0	ax[0] x ax[1]			ax[2]
	//
	// constraining w2-w1 along an axis 'a' means that a'*(w2-w1)=0.
	// to prove the result for angle[0], write the expression for angle[0] from
	// GetInfo1 then take the derivative. to prove this for angle[2] it is
	// easier to take the euler rate expression for d(angle[2])/dt with respect
	// to the components of w and set that to 0.
	btlVector3 axis0 = m_calculatedTransformB.getBasis().getColumn(0);
	btlVector3 axis2 = m_calculatedTransformA.getBasis().getColumn(2);

	m_calculatedAxis[1] = axis2.cross(axis0);
	m_calculatedAxis[0] = m_calculatedAxis[1].cross(axis2);
	m_calculatedAxis[2] = axis0.cross(m_calculatedAxis[1]);

	m_calculatedAxis[0].normalize();
	m_calculatedAxis[1].normalize();
	m_calculatedAxis[2].normalize();

}

void btlGeneric6DofConstraint::calculateTransforms()
{
	calculateTransforms(m_rbA.getCenterOfMassTransform(),m_rbB.getCenterOfMassTransform());
}

void btlGeneric6DofConstraint::calculateTransforms(const btlTransform& transA,const btlTransform& transB)
{
	m_calculatedTransformA = transA * m_frameInA;
	m_calculatedTransformB = transB * m_frameInB;
	calculateLinearInfo();
	calculateAngleInfo();
	if(m_useOffsetForConstraintFrame)
	{	//  get weight factors depending on masses
		btlScalar miA = getRigidBodyA().getInvMass();
		btlScalar miB = getRigidBodyB().getInvMass();
		m_hasStaticBody = (miA < SIMD_EPSILON) || (miB < SIMD_EPSILON);
		btlScalar miS = miA + miB;
		if(miS > btlScalar(0.f))
		{
			m_factA = miB / miS;
		}
		else 
		{
			m_factA = btlScalar(0.5f);
		}
		m_factB = btlScalar(1.0f) - m_factA;
	}
}



void btlGeneric6DofConstraint::buildLinearJacobian(
	btlJacobianEntry & jacLinear,const btlVector3 & normalWorld,
	const btlVector3 & pivotAInW,const btlVector3 & pivotBInW)
{
	new (&jacLinear) btlJacobianEntry(
        m_rbA.getCenterOfMassTransform().getBasis().transpose(),
        m_rbB.getCenterOfMassTransform().getBasis().transpose(),
        pivotAInW - m_rbA.getCenterOfMassPosition(),
        pivotBInW - m_rbB.getCenterOfMassPosition(),
        normalWorld,
        m_rbA.getInvInertiaDiagLocal(),
        m_rbA.getInvMass(),
        m_rbB.getInvInertiaDiagLocal(),
        m_rbB.getInvMass());
}



void btlGeneric6DofConstraint::buildAngularJacobian(
	btlJacobianEntry & jacAngular,const btlVector3 & jointAxisW)
{
	 new (&jacAngular)	btlJacobianEntry(jointAxisW,
                                      m_rbA.getCenterOfMassTransform().getBasis().transpose(),
                                      m_rbB.getCenterOfMassTransform().getBasis().transpose(),
                                      m_rbA.getInvInertiaDiagLocal(),
                                      m_rbB.getInvInertiaDiagLocal());

}



bool btlGeneric6DofConstraint::testAngularLimitMotor(int axis_index)
{
	btlScalar angle = m_calculatedAxisAngleDiff[axis_index];
	angle = btlAdjustAngleToLimits(angle, m_angularLimits[axis_index].m_loLimit, m_angularLimits[axis_index].m_hiLimit);
	m_angularLimits[axis_index].m_currentPosition = angle;
	//test limits
	m_angularLimits[axis_index].testLimitValue(angle);
	return m_angularLimits[axis_index].needApplyTorques();
}



void btlGeneric6DofConstraint::buildJacobian()
{
#ifndef __SPU__
	if (m_useSolveConstraintObsolete)
	{

		// Clear accumulated impulses for the next simulation step
		m_linearLimits.m_accumulatedImpulse.setValue(btlScalar(0.), btlScalar(0.), btlScalar(0.));
		int i;
		for(i = 0; i < 3; i++)
		{
			m_angularLimits[i].m_accumulatedImpulse = btlScalar(0.);
		}
		//calculates transform
		calculateTransforms(m_rbA.getCenterOfMassTransform(),m_rbB.getCenterOfMassTransform());

		//  const btVector3& pivotAInW = m_calculatedTransformA.getOrigin();
		//  const btVector3& pivotBInW = m_calculatedTransformB.getOrigin();
		calcAnchorPos();
		btlVector3 pivotAInW = m_AnchorPos;
		btlVector3 pivotBInW = m_AnchorPos;

		// not used here
		//    btVector3 rel_pos1 = pivotAInW - m_rbA.getCenterOfMassPosition();
		//    btVector3 rel_pos2 = pivotBInW - m_rbB.getCenterOfMassPosition();

		btlVector3 normalWorld;
		//linear part
		for (i=0;i<3;i++)
		{
			if (m_linearLimits.isLimited(i))
			{
				if (m_useLinearReferenceFrameA)
					normalWorld = m_calculatedTransformA.getBasis().getColumn(i);
				else
					normalWorld = m_calculatedTransformB.getBasis().getColumn(i);

				buildLinearJacobian(
					m_jacLinear[i],normalWorld ,
					pivotAInW,pivotBInW);

			}
		}

		// angular part
		for (i=0;i<3;i++)
		{
			//calculates error angle
			if (testAngularLimitMotor(i))
			{
				normalWorld = this->getAxis(i);
				// Create angular atom
				buildAngularJacobian(m_jacAng[i],normalWorld);
			}
		}

	}
#endif //__SPU__

}


void btlGeneric6DofConstraint::getInfo1 (btlConstraintInfo1* info)
{
	if (m_useSolveConstraintObsolete)
	{
		info->m_numConstraintRows = 0;
		info->nub = 0;
	} else
	{
		//prepare constraint
		calculateTransforms(m_rbA.getCenterOfMassTransform(),m_rbB.getCenterOfMassTransform());
		info->m_numConstraintRows = 0;
		info->nub = 6;
		int i;
		//test linear limits
		for(i = 0; i < 3; i++)
		{
			if(m_linearLimits.needApplyForce(i))
			{
				info->m_numConstraintRows++;
				info->nub--;
			}
		}
		//test angular limits
		for (i=0;i<3 ;i++ )
		{
			if(testAngularLimitMotor(i))
			{
				info->m_numConstraintRows++;
				info->nub--;
			}
		}
	}
}

void btlGeneric6DofConstraint::getInfo1NonVirtual (btlConstraintInfo1* info)
{
	if (m_useSolveConstraintObsolete)
	{
		info->m_numConstraintRows = 0;
		info->nub = 0;
	} else
	{
		//pre-allocate all 6
		info->m_numConstraintRows = 6;
		info->nub = 0;
	}
}


void btlGeneric6DofConstraint::getInfo2 (btlConstraintInfo2* info)
{
	btlAssert(!m_useSolveConstraintObsolete);

	const btlTransform& transA = m_rbA.getCenterOfMassTransform();
	const btlTransform& transB = m_rbB.getCenterOfMassTransform();
	const btlVector3& linVelA = m_rbA.getLinearVelocity();
	const btlVector3& linVelB = m_rbB.getLinearVelocity();
	const btlVector3& angVelA = m_rbA.getAngularVelocity();
	const btlVector3& angVelB = m_rbB.getAngularVelocity();

	if(m_useOffsetForConstraintFrame)
	{ // for stability better to solve angular limits first
		int row = setAngularLimits(info, 0,transA,transB,linVelA,linVelB,angVelA,angVelB);
		setLinearLimits(info, row, transA,transB,linVelA,linVelB,angVelA,angVelB);
	}
	else
	{ // leave old version for compatibility
		int row = setLinearLimits(info, 0, transA,transB,linVelA,linVelB,angVelA,angVelB);
		setAngularLimits(info, row,transA,transB,linVelA,linVelB,angVelA,angVelB);
	}

}


void btlGeneric6DofConstraint::getInfo2NonVirtual (btlConstraintInfo2* info, const btlTransform& transA,const btlTransform& transB,const btlVector3& linVelA,const btlVector3& linVelB,const btlVector3& angVelA,const btlVector3& angVelB)
{
	
	btlAssert(!m_useSolveConstraintObsolete);
	//prepare constraint
	calculateTransforms(transA,transB);

	int i;
	for (i=0;i<3 ;i++ )
	{
		testAngularLimitMotor(i);
	}

	if(m_useOffsetForConstraintFrame)
	{ // for stability better to solve angular limits first
		int row = setAngularLimits(info, 0,transA,transB,linVelA,linVelB,angVelA,angVelB);
		setLinearLimits(info, row, transA,transB,linVelA,linVelB,angVelA,angVelB);
	}
	else
	{ // leave old version for compatibility
		int row = setLinearLimits(info, 0, transA,transB,linVelA,linVelB,angVelA,angVelB);
		setAngularLimits(info, row,transA,transB,linVelA,linVelB,angVelA,angVelB);
	}
}



int btlGeneric6DofConstraint::setLinearLimits(btlConstraintInfo2* info, int row, const btlTransform& transA,const btlTransform& transB,const btlVector3& linVelA,const btlVector3& linVelB,const btlVector3& angVelA,const btlVector3& angVelB)
{
//	int row = 0;
	//solve linear limits
	btlRotationalLimitMotor limot;
	for (int i=0;i<3 ;i++ )
	{
		if(m_linearLimits.needApplyForce(i))
		{ // re-use rotational motor code
			limot.m_bounce = btlScalar(0.f);
			limot.m_currentLimit = m_linearLimits.m_currentLimit[i];
			limot.m_currentPosition = m_linearLimits.m_currentLinearDiff[i];
			limot.m_currentLimitError  = m_linearLimits.m_currentLimitError[i];
			limot.m_damping  = m_linearLimits.m_damping;
			limot.m_enableMotor  = m_linearLimits.m_enableMotor[i];
			limot.m_hiLimit  = m_linearLimits.m_upperLimit[i];
			limot.m_limitSoftness  = m_linearLimits.m_limitSoftness;
			limot.m_loLimit  = m_linearLimits.m_lowerLimit[i];
			limot.m_maxLimitForce  = btlScalar(0.f);
			limot.m_maxMotorForce  = m_linearLimits.m_maxMotorForce[i];
			limot.m_targetVelocity  = m_linearLimits.m_targetVelocity[i];
			btlVector3 axis = m_calculatedTransformA.getBasis().getColumn(i);
			int flags = m_flags >> (i * BT_6DOF_FLAGS_AXIS_SHIFT);
			limot.m_normalCFM	= (flags & BT_6DOF_FLAGS_CFM_NORM) ? m_linearLimits.m_normalCFM[i] : info->cfm[0];
			limot.m_stopCFM		= (flags & BT_6DOF_FLAGS_CFM_STOP) ? m_linearLimits.m_stopCFM[i] : info->cfm[0];
			limot.m_stopERP		= (flags & BT_6DOF_FLAGS_ERP_STOP) ? m_linearLimits.m_stopERP[i] : info->erp;
			if(m_useOffsetForConstraintFrame)
			{
				int indx1 = (i + 1) % 3;
				int indx2 = (i + 2) % 3;
				int rotAllowed = 1; // rotations around orthos to current axis
				if(m_angularLimits[indx1].m_currentLimit && m_angularLimits[indx2].m_currentLimit)
				{
					rotAllowed = 0;
				}
				row += get_limit_motor_info2(&limot, transA,transB,linVelA,linVelB,angVelA,angVelB, info, row, axis, 0, rotAllowed);
			}
			else
			{
				row += get_limit_motor_info2(&limot, transA,transB,linVelA,linVelB,angVelA,angVelB, info, row, axis, 0);
			}
		}
	}
	return row;
}



int btlGeneric6DofConstraint::setAngularLimits(btlConstraintInfo2 *info, int row_offset, const btlTransform& transA,const btlTransform& transB,const btlVector3& linVelA,const btlVector3& linVelB,const btlVector3& angVelA,const btlVector3& angVelB)
{
	btlGeneric6DofConstraint * d6constraint = this;
	int row = row_offset;
	//solve angular limits
	for (int i=0;i<3 ;i++ )
	{
		if(d6constraint->getRotationalLimitMotor(i)->needApplyTorques())
		{
			btlVector3 axis = d6constraint->getAxis(i);
			int flags = m_flags >> ((i + 3) * BT_6DOF_FLAGS_AXIS_SHIFT);
			if(!(flags & BT_6DOF_FLAGS_CFM_NORM))
			{
				m_angularLimits[i].m_normalCFM = info->cfm[0];
			}
			if(!(flags & BT_6DOF_FLAGS_CFM_STOP))
			{
				m_angularLimits[i].m_stopCFM = info->cfm[0];
			}
			if(!(flags & BT_6DOF_FLAGS_ERP_STOP))
			{
				m_angularLimits[i].m_stopERP = info->erp;
			}
			row += get_limit_motor_info2(d6constraint->getRotationalLimitMotor(i),
												transA,transB,linVelA,linVelB,angVelA,angVelB, info,row,axis,1);
		}
	}

	return row;
}




void	btlGeneric6DofConstraint::updateRHS(btlScalar	timeStep)
{
	(void)timeStep;

}


void btlGeneric6DofConstraint::setFrames(const btlTransform& frameA, const btlTransform& frameB)
{
	m_frameInA = frameA;
	m_frameInB = frameB;
	buildJacobian();
	calculateTransforms();
}



btlVector3 btlGeneric6DofConstraint::getAxis(int axis_index) const
{
	return m_calculatedAxis[axis_index];
}


btlScalar	btlGeneric6DofConstraint::getRelativePivotPosition(int axisIndex) const
{
	return m_calculatedLinearDiff[axisIndex];
}


btlScalar btlGeneric6DofConstraint::getAngle(int axisIndex) const
{
	return m_calculatedAxisAngleDiff[axisIndex];
}



void btlGeneric6DofConstraint::calcAnchorPos(void)
{
	btlScalar imA = m_rbA.getInvMass();
	btlScalar imB = m_rbB.getInvMass();
	btlScalar weight;
	if(imB == btlScalar(0.0))
	{
		weight = btlScalar(1.0);
	}
	else
	{
		weight = imA / (imA + imB);
	}
	const btlVector3& pA = m_calculatedTransformA.getOrigin();
	const btlVector3& pB = m_calculatedTransformB.getOrigin();
	m_AnchorPos = pA * weight + pB * (btlScalar(1.0) - weight);
	return;
}



void btlGeneric6DofConstraint::calculateLinearInfo()
{
	m_calculatedLinearDiff = m_calculatedTransformB.getOrigin() - m_calculatedTransformA.getOrigin();
	m_calculatedLinearDiff = m_calculatedTransformA.getBasis().inverse() * m_calculatedLinearDiff;
	for(int i = 0; i < 3; i++)
	{
		m_linearLimits.m_currentLinearDiff[i] = m_calculatedLinearDiff[i];
		m_linearLimits.testLimitValue(i, m_calculatedLinearDiff[i]);
	}
}



int btlGeneric6DofConstraint::get_limit_motor_info2(
	btlRotationalLimitMotor * limot,
	const btlTransform& transA,const btlTransform& transB,const btlVector3& linVelA,const btlVector3& linVelB,const btlVector3& angVelA,const btlVector3& angVelB,
	btlConstraintInfo2 *info, int row, btlVector3& ax1, int rotational,int rotAllowed)
{
    int srow = row * info->rowskip;
    int powered = limot->m_enableMotor;
    int limit = limot->m_currentLimit;
    if (powered || limit)
    {   // if the joint is powered, or has joint limits, add in the extra row
        btlScalar *J1 = rotational ? info->m_J1angularAxis : info->m_J1linearAxis;
        btlScalar *J2 = rotational ? info->m_J2angularAxis : info->m_J2linearAxis;
        J1[srow+0] = ax1[0];
        J1[srow+1] = ax1[1];
        J1[srow+2] = ax1[2];

        J2[srow+0] = -ax1[0];
        J2[srow+1] = -ax1[1];
        J2[srow+2] = -ax1[2];

		if((!rotational))
        {
			if (m_useOffsetForConstraintFrame)
			{
				btlVector3 tmpA, tmpB, relA, relB;
				// get vector from bodyB to frameB in WCS
				relB = m_calculatedTransformB.getOrigin() - transB.getOrigin();
				// get its projection to constraint axis
				btlVector3 projB = ax1 * relB.dot(ax1);
				// get vector directed from bodyB to constraint axis (and orthogonal to it)
				btlVector3 orthoB = relB - projB;
				// same for bodyA
				relA = m_calculatedTransformA.getOrigin() - transA.getOrigin();
				btlVector3 projA = ax1 * relA.dot(ax1);
				btlVector3 orthoA = relA - projA;
				// get desired offset between frames A and B along constraint axis
				btlScalar desiredOffs = limot->m_currentPosition - limot->m_currentLimitError;
				// desired vector from projection of center of bodyA to projection of center of bodyB to constraint axis
				btlVector3 totalDist = projA + ax1 * desiredOffs - projB;
				// get offset vectors relA and relB
				relA = orthoA + totalDist * m_factA;
				relB = orthoB - totalDist * m_factB;
				tmpA = relA.cross(ax1);
				tmpB = relB.cross(ax1);
				if(m_hasStaticBody && (!rotAllowed))
				{
					tmpA *= m_factA;
					tmpB *= m_factB;
				}
				int i;
				for (i=0; i<3; i++) info->m_J1angularAxis[srow+i] = tmpA[i];
				for (i=0; i<3; i++) info->m_J2angularAxis[srow+i] = -tmpB[i];
			} else
			{
				btlVector3 ltd;	// Linear Torque Decoupling vector
				btlVector3 c = m_calculatedTransformB.getOrigin() - transA.getOrigin();
				ltd = c.cross(ax1);
				info->m_J1angularAxis[srow+0] = ltd[0];
				info->m_J1angularAxis[srow+1] = ltd[1];
				info->m_J1angularAxis[srow+2] = ltd[2];

				c = m_calculatedTransformB.getOrigin() - transB.getOrigin();
				ltd = -c.cross(ax1);
				info->m_J2angularAxis[srow+0] = ltd[0];
				info->m_J2angularAxis[srow+1] = ltd[1];
				info->m_J2angularAxis[srow+2] = ltd[2];
			}
        }
        // if we're limited low and high simultaneously, the joint motor is
        // ineffective
        if (limit && (limot->m_loLimit == limot->m_hiLimit)) powered = 0;
        info->m_constraintError[srow] = btlScalar(0.f);
        if (powered)
        {
			info->cfm[srow] = limot->m_normalCFM;
            if(!limit)
            {
				btlScalar tag_vel = rotational ? limot->m_targetVelocity : -limot->m_targetVelocity;

				btlScalar mot_fact = getMotorFactor(	limot->m_currentPosition, 
													limot->m_loLimit,
													limot->m_hiLimit, 
													tag_vel, 
													info->fps * limot->m_stopERP);
				info->m_constraintError[srow] += mot_fact * limot->m_targetVelocity;
                info->m_lowerLimit[srow] = -limot->m_maxMotorForce;
                info->m_upperLimit[srow] = limot->m_maxMotorForce;
            }
        }
        if(limit)
        {
            btlScalar k = info->fps * limot->m_stopERP;
			if(!rotational)
			{
				info->m_constraintError[srow] += k * limot->m_currentLimitError;
			}
			else
			{
				info->m_constraintError[srow] += -k * limot->m_currentLimitError;
			}
			info->cfm[srow] = limot->m_stopCFM;
            if (limot->m_loLimit == limot->m_hiLimit)
            {   // limited low and high simultaneously
                info->m_lowerLimit[srow] = -SIMD_INFINITY;
                info->m_upperLimit[srow] = SIMD_INFINITY;
            }
            else
            {
                if (limit == 1)
                {
                    info->m_lowerLimit[srow] = 0;
                    info->m_upperLimit[srow] = SIMD_INFINITY;
                }
                else
                {
                    info->m_lowerLimit[srow] = -SIMD_INFINITY;
                    info->m_upperLimit[srow] = 0;
                }
                // deal with bounce
                if (limot->m_bounce > 0)
                {
                    // calculate joint velocity
                    btlScalar vel;
                    if (rotational)
                    {
                        vel = angVelA.dot(ax1);
//make sure that if no body -> angVelB == zero vec
//                        if (body1)
                            vel -= angVelB.dot(ax1);
                    }
                    else
                    {
                        vel = linVelA.dot(ax1);
//make sure that if no body -> angVelB == zero vec
//                        if (body1)
                            vel -= linVelB.dot(ax1);
                    }
                    // only apply bounce if the velocity is incoming, and if the
                    // resulting c[] exceeds what we already have.
                    if (limit == 1)
                    {
                        if (vel < 0)
                        {
                            btlScalar newc = -limot->m_bounce* vel;
                            if (newc > info->m_constraintError[srow]) 
								info->m_constraintError[srow] = newc;
                        }
                    }
                    else
                    {
                        if (vel > 0)
                        {
                            btlScalar newc = -limot->m_bounce * vel;
                            if (newc < info->m_constraintError[srow]) 
								info->m_constraintError[srow] = newc;
                        }
                    }
                }
            }
        }
        return 1;
    }
    else return 0;
}






	///override the default global value of a parameter (such as ERP or CFM), optionally provide the axis (0..5). 
	///If no axis is provided, it uses the default axis for this constraint.
void btlGeneric6DofConstraint::setParam(int num, btlScalar value, int axis)
{
	if((axis >= 0) && (axis < 3))
	{
		switch(num)
		{
			case BT_CONSTRAINT_STOP_ERP : 
				m_linearLimits.m_stopERP[axis] = value;
				m_flags |= BT_6DOF_FLAGS_ERP_STOP << (axis * BT_6DOF_FLAGS_AXIS_SHIFT);
				break;
			case BT_CONSTRAINT_STOP_CFM : 
				m_linearLimits.m_stopCFM[axis] = value;
				m_flags |= BT_6DOF_FLAGS_CFM_STOP << (axis * BT_6DOF_FLAGS_AXIS_SHIFT);
				break;
			case BT_CONSTRAINT_CFM : 
				m_linearLimits.m_normalCFM[axis] = value;
				m_flags |= BT_6DOF_FLAGS_CFM_NORM << (axis * BT_6DOF_FLAGS_AXIS_SHIFT);
				break;
			default : 
				btlAssertConstrParams(0);
		}
	}
	else if((axis >=3) && (axis < 6))
	{
		switch(num)
		{
			case BT_CONSTRAINT_STOP_ERP : 
				m_angularLimits[axis - 3].m_stopERP = value;
				m_flags |= BT_6DOF_FLAGS_ERP_STOP << (axis * BT_6DOF_FLAGS_AXIS_SHIFT);
				break;
			case BT_CONSTRAINT_STOP_CFM : 
				m_angularLimits[axis - 3].m_stopCFM = value;
				m_flags |= BT_6DOF_FLAGS_CFM_STOP << (axis * BT_6DOF_FLAGS_AXIS_SHIFT);
				break;
			case BT_CONSTRAINT_CFM : 
				m_angularLimits[axis - 3].m_normalCFM = value;
				m_flags |= BT_6DOF_FLAGS_CFM_NORM << (axis * BT_6DOF_FLAGS_AXIS_SHIFT);
				break;
			default : 
				btlAssertConstrParams(0);
		}
	}
	else
	{
		btlAssertConstrParams(0);
	}
}

	///return the local value of parameter
btlScalar btlGeneric6DofConstraint::getParam(int num, int axis) const 
{
	btlScalar retVal = 0;
	if((axis >= 0) && (axis < 3))
	{
		switch(num)
		{
			case BT_CONSTRAINT_STOP_ERP : 
				btlAssertConstrParams(m_flags & (BT_6DOF_FLAGS_ERP_STOP << (axis * BT_6DOF_FLAGS_AXIS_SHIFT)));
				retVal = m_linearLimits.m_stopERP[axis];
				break;
			case BT_CONSTRAINT_STOP_CFM : 
				btlAssertConstrParams(m_flags & (BT_6DOF_FLAGS_CFM_STOP << (axis * BT_6DOF_FLAGS_AXIS_SHIFT)));
				retVal = m_linearLimits.m_stopCFM[axis];
				break;
			case BT_CONSTRAINT_CFM : 
				btlAssertConstrParams(m_flags & (BT_6DOF_FLAGS_CFM_NORM << (axis * BT_6DOF_FLAGS_AXIS_SHIFT)));
				retVal = m_linearLimits.m_normalCFM[axis];
				break;
			default : 
				btlAssertConstrParams(0);
		}
	}
	else if((axis >=3) && (axis < 6))
	{
		switch(num)
		{
			case BT_CONSTRAINT_STOP_ERP : 
				btlAssertConstrParams(m_flags & (BT_6DOF_FLAGS_ERP_STOP << (axis * BT_6DOF_FLAGS_AXIS_SHIFT)));
				retVal = m_angularLimits[axis - 3].m_stopERP;
				break;
			case BT_CONSTRAINT_STOP_CFM : 
				btlAssertConstrParams(m_flags & (BT_6DOF_FLAGS_CFM_STOP << (axis * BT_6DOF_FLAGS_AXIS_SHIFT)));
				retVal = m_angularLimits[axis - 3].m_stopCFM;
				break;
			case BT_CONSTRAINT_CFM : 
				btlAssertConstrParams(m_flags & (BT_6DOF_FLAGS_CFM_NORM << (axis * BT_6DOF_FLAGS_AXIS_SHIFT)));
				retVal = m_angularLimits[axis - 3].m_normalCFM;
				break;
			default : 
				btlAssertConstrParams(0);
		}
	}
	else
	{
		btlAssertConstrParams(0);
	}
	return retVal;
}

 

void btlGeneric6DofConstraint::setAxis(const btlVector3& axis1,const btlVector3& axis2)
{
	btlVector3 zAxis = axis1.normalized();
	btlVector3 yAxis = axis2.normalized();
	btlVector3 xAxis = yAxis.cross(zAxis); // we want right coordinate system
	
	btlTransform frameInW;
	frameInW.setIdentity();
	frameInW.getBasis().setValue(	xAxis[0], yAxis[0], zAxis[0],	
	                                xAxis[1], yAxis[1], zAxis[1],
	                               xAxis[2], yAxis[2], zAxis[2]);
	
	// now get constraint frame in local coordinate systems
	m_frameInA = m_rbA.getCenterOfMassTransform().inverse() * frameInW;
	m_frameInB = m_rbB.getCenterOfMassTransform().inverse() * frameInW;
	
	calculateTransforms();
}
