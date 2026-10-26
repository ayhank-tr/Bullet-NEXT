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
2014 May: btGeneric6DofSpring2Constraint is created from the original (2.82.2712) btGeneric6DofConstraint by Gabor Puhr and Tamas Umenhoffer
Pros:
- Much more accurate and stable in a lot of situation. (Especially when a sleeping chain of RBs connected with 6dof2 is pulled)
- Stable and accurate spring with minimal energy loss that works with all of the solvers. (latter is not true for the original 6dof spring)
- Servo motor functionality
- Much more accurate bouncing. 0 really means zero bouncing (not true for the original 6odf) and there is only a minimal energy loss when the value is 1 (because of the solvers' precision)
- Rotation order for the Euler system can be set. (One axis' freedom is still limited to pi/2)

Cons:
- It is slower than the original 6dof. There is no exact ratio, but half speed is a good estimation. (with PGS)
- At bouncing the correct velocity is calculated, but not the correct position. (it is because of the solver can correct position or velocity, but not both.)
*/

/// 2009 March: btGeneric6DofConstraint refactored by Roman Ponomarev
/// Added support for generic constraint solver through getInfo1/getInfo2 methods

/*
2007-09-09
btGeneric6DofConstraint Refactored by Francisco Le?n
email: projectileman@yahoo.com
http://gimpact.sf.net
*/



#include "btGeneric6DofSpring2Constraint.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "LinearMath/btTransformUtil.h"
#include <new>



btlGeneric6DofSpring2Constraint::btlGeneric6DofSpring2Constraint(btlRigidBody& rbA, btlRigidBody& rbB, const btlTransform& frameInA, const btlTransform& frameInB, RotateOrder rotOrder)
	: btlTypedConstraint(D6_SPRING_2_CONSTRAINT_TYPE, rbA, rbB)
	, m_frameInA(frameInA)
	, m_frameInB(frameInB)
	, m_rotateOrder(rotOrder)	
	, m_flags(0)
{
	calculateTransforms();
}


btlGeneric6DofSpring2Constraint::btlGeneric6DofSpring2Constraint(btlRigidBody& rbB, const btlTransform& frameInB, RotateOrder rotOrder)
	: btlTypedConstraint(D6_SPRING_2_CONSTRAINT_TYPE, getFixedBody(), rbB)
	, m_frameInB(frameInB)
	, m_rotateOrder(rotOrder)
	, m_flags(0)
{
	///not providing rigidbody A means implicitly using worldspace for body A
	m_frameInA = rbB.getCenterOfMassTransform() * m_frameInB;
	calculateTransforms();
}


btlScalar btlGeneric6DofSpring2Constraint::btlGetMatrixElem(const btlMatrix3x3& mat, int index)
{
	int i = index%3;
	int j = index/3;
	return mat[i][j];
}

// MatrixToEulerXYZ from http://www.geometrictools.com/LibFoundation/Mathematics/Wm4Matrix3.inl.html

bool btlGeneric6DofSpring2Constraint::btl_matrixToEulerXYZ(const btlMatrix3x3& mat,btlVector3& xyz)
{
	// rot =  cy*cz          -cy*sz           sy
	//        cz*sx*sy+cx*sz  cx*cz-sx*sy*sz -cy*sx
	//       -cx*cz*sy+sx*sz  cz*sx+cx*sy*sz  cx*cy

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

bool btlGeneric6DofSpring2Constraint::matrixToEulerXZY(const btlMatrix3x3& mat,btlVector3& xyz)
{
	// rot =  cy*cz          -sz           sy*cz
	//        cy*cx*sz+sx*sy  cx*cz        sy*cx*sz-cy*sx
	//        cy*sx*sz-cx*sy  sx*cz        sy*sx*sz+cx*cy

	btlScalar fi = btlGetMatrixElem(mat,1);
	if (fi < btlScalar(1.0f))
	{
		if (fi > btlScalar(-1.0f))
		{
			xyz[0] = btlAtan2(btlGetMatrixElem(mat,7),btlGetMatrixElem(mat,4));
			xyz[1] = btlAtan2(btlGetMatrixElem(mat,2),btlGetMatrixElem(mat,0));
			xyz[2] = btlAsin(-btlGetMatrixElem(mat,1));
			return true;
		}
		else
		{
			xyz[0] = -btlAtan2(-btlGetMatrixElem(mat,6),btlGetMatrixElem(mat,8));
			xyz[1] = btlScalar(0.0);
			xyz[2] = SIMD_HALF_PI;
			return false;
		}
	}
	else
	{
		xyz[0] = btlAtan2(-btlGetMatrixElem(mat,6),btlGetMatrixElem(mat,8));
		xyz[1] = 0.0;
		xyz[2] = -SIMD_HALF_PI;
	}
	return false;
}

bool btlGeneric6DofSpring2Constraint::matrixToEulerYXZ(const btlMatrix3x3& mat,btlVector3& xyz)
{
	// rot =  cy*cz+sy*sx*sz  cz*sy*sx-cy*sz  cx*sy
	//        cx*sz           cx*cz           -sx
	//        cy*sx*sz-cz*sy  sy*sz+cy*cz*sx  cy*cx

	btlScalar fi = btlGetMatrixElem(mat,5);
	if (fi < btlScalar(1.0f))
	{
		if (fi > btlScalar(-1.0f))
		{
			xyz[0] = btlAsin(-btlGetMatrixElem(mat,5));
			xyz[1] = btlAtan2(btlGetMatrixElem(mat,2),btlGetMatrixElem(mat,8));
			xyz[2] = btlAtan2(btlGetMatrixElem(mat,3),btlGetMatrixElem(mat,4));
			return true;
		}
		else
		{
			xyz[0] = SIMD_HALF_PI;
			xyz[1] = -btlAtan2(-btlGetMatrixElem(mat,1),btlGetMatrixElem(mat,0));
			xyz[2] = btlScalar(0.0);
			return false;
		}
	}
	else
	{
		xyz[0] = -SIMD_HALF_PI;
		xyz[1] = btlAtan2(-btlGetMatrixElem(mat,1),btlGetMatrixElem(mat,0));
		xyz[2] = 0.0;
	}
	return false;
}

bool btlGeneric6DofSpring2Constraint::matrixToEulerYZX(const btlMatrix3x3& mat,btlVector3& xyz)
{
	// rot =  cy*cz   sy*sx-cy*cx*sz   cx*sy+cy*sz*sx
	//        sz           cz*cx           -cz*sx
	//        -cz*sy  cy*sx+cx*sy*sz   cy*cx-sy*sz*sx

	btlScalar fi = btlGetMatrixElem(mat,3);
	if (fi < btlScalar(1.0f))
	{
		if (fi > btlScalar(-1.0f))
		{
			xyz[0] = btlAtan2(-btlGetMatrixElem(mat,5),btlGetMatrixElem(mat,4));
			xyz[1] = btlAtan2(-btlGetMatrixElem(mat,6),btlGetMatrixElem(mat,0));
			xyz[2] = btlAsin(btlGetMatrixElem(mat,3));
			return true;
		}
		else
		{
			xyz[0] = btlScalar(0.0);
			xyz[1] = -btlAtan2(btlGetMatrixElem(mat,7),btlGetMatrixElem(mat,8));
			xyz[2] = -SIMD_HALF_PI;
			return false;
		}
	}
	else
	{
		xyz[0] = btlScalar(0.0);
		xyz[1] = btlAtan2(btlGetMatrixElem(mat,7),btlGetMatrixElem(mat,8));
		xyz[2] = SIMD_HALF_PI;
	}
	return false;
}

bool btlGeneric6DofSpring2Constraint::matrixToEulerZXY(const btlMatrix3x3& mat,btlVector3& xyz)
{
	// rot =  cz*cy-sz*sx*sy    -cx*sz   cz*sy+cy*sz*sx
	//        cy*sz+cz*sx*sy     cz*cx   sz*sy-cz*xy*sx
	//        -cx*sy              sx     cx*cy

	btlScalar fi = btlGetMatrixElem(mat,7);
	if (fi < btlScalar(1.0f))
	{
		if (fi > btlScalar(-1.0f))
		{
			xyz[0] = btlAsin(btlGetMatrixElem(mat,7));
			xyz[1] = btlAtan2(-btlGetMatrixElem(mat,6),btlGetMatrixElem(mat,8));
			xyz[2] = btlAtan2(-btlGetMatrixElem(mat,1),btlGetMatrixElem(mat,4));
			return true;
		}
		else
		{
			xyz[0] = -SIMD_HALF_PI;
			xyz[1] = btlScalar(0.0);
			xyz[2] = -btlAtan2(btlGetMatrixElem(mat,2),btlGetMatrixElem(mat,0));
			return false;
		}
	}
	else
	{
		xyz[0] = SIMD_HALF_PI;
		xyz[1] = btlScalar(0.0);
		xyz[2] = btlAtan2(btlGetMatrixElem(mat,2),btlGetMatrixElem(mat,0));
	}
	return false;
}

bool btlGeneric6DofSpring2Constraint::matrixToEulerZYX(const btlMatrix3x3& mat,btlVector3& xyz)
{
	// rot =  cz*cy   cz*sy*sx-cx*sz   sz*sx+cz*cx*sy
	//        cy*sz   cz*cx+sz*sy*sx   cx*sz*sy-cz*sx
	//        -sy          cy*sx         cy*cx

	btlScalar fi = btlGetMatrixElem(mat,6);
	if (fi < btlScalar(1.0f))
	{
		if (fi > btlScalar(-1.0f))
		{
			xyz[0] = btlAtan2(btlGetMatrixElem(mat,7), btlGetMatrixElem(mat,8));
			xyz[1] = btlAsin(-btlGetMatrixElem(mat,6));
			xyz[2] = btlAtan2(btlGetMatrixElem(mat,3),btlGetMatrixElem(mat,0));
			return true;
		}
		else
		{
			xyz[0] = btlScalar(0.0);
			xyz[1] = SIMD_HALF_PI;
			xyz[2] = -btlAtan2(btlGetMatrixElem(mat,1),btlGetMatrixElem(mat,2));
			return false;
		}
	}
	else
	{
		xyz[0] = btlScalar(0.0);
		xyz[1] = -SIMD_HALF_PI;
		xyz[2] = btlAtan2(-btlGetMatrixElem(mat,1),-btlGetMatrixElem(mat,2));
	}
	return false;
}

void btlGeneric6DofSpring2Constraint::calculateAngleInfo()
{
	btlMatrix3x3 relative_frame = m_calculatedTransformA.getBasis().inverse()*m_calculatedTransformB.getBasis();
	switch (m_rotateOrder)
	{
		case RO_XYZ : btl_matrixToEulerXYZ(relative_frame,m_calculatedAxisAngleDiff); break;
		case RO_XZY : matrixToEulerXZY(relative_frame,m_calculatedAxisAngleDiff); break;
		case RO_YXZ : matrixToEulerYXZ(relative_frame,m_calculatedAxisAngleDiff); break;
		case RO_YZX : matrixToEulerYZX(relative_frame,m_calculatedAxisAngleDiff); break;
		case RO_ZXY : matrixToEulerZXY(relative_frame,m_calculatedAxisAngleDiff); break;
		case RO_ZYX : matrixToEulerZYX(relative_frame,m_calculatedAxisAngleDiff); break;
		default : btlAssert(false);
	}
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
	switch (m_rotateOrder)
	{
	case RO_XYZ :
		{
			//Is this the "line of nodes" calculation choosing planes YZ (B coordinate system) and xy (A coordinate system)? (http://en.wikipedia.org/wiki/Euler_angles)
			//The two planes are non-homologous, so this is a Tait–Bryan angle formalism and not a proper Euler
			//Extrinsic rotations are equal to the reversed order intrinsic rotations so the above xyz extrinsic rotations (axes are fixed) are the same as the zy'x" intrinsic rotations (axes are refreshed after each rotation)
			//that is why xy and YZ planes are chosen (this will describe a zy'x" intrinsic rotation) (see the figure on the left at http://en.wikipedia.org/wiki/Euler_angles under Tait–Bryan angles)
			// x' = Nperp = N.cross(axis2)
			// y' = N = axis2.cross(axis0)	
			// z' = z
			//
			// x" = X
			// y" = y'
			// z" = ??
			//in other words:
			//first rotate around z
			//second rotate around y'= z.cross(X)
			//third rotate around x" = X
			//Original XYZ extrinsic rotation order. 
			//Planes: xy and YZ normals: z, X.  Plane intersection (N) is z.cross(X)
			btlVector3 axis0 = m_calculatedTransformB.getBasis().getColumn(0);
			btlVector3 axis2 = m_calculatedTransformA.getBasis().getColumn(2);
			m_calculatedAxis[1] = axis2.cross(axis0);
			m_calculatedAxis[0] = m_calculatedAxis[1].cross(axis2);
			m_calculatedAxis[2] = axis0.cross(m_calculatedAxis[1]);
			break;
		}
	case RO_XZY :
		{
			//planes: xz,ZY normals: y, X
			//first rotate around y
			//second rotate around z'= y.cross(X)
			//third rotate around x" = X
			btlVector3 axis0 = m_calculatedTransformB.getBasis().getColumn(0);
			btlVector3 axis1 = m_calculatedTransformA.getBasis().getColumn(1);
			m_calculatedAxis[2] = axis0.cross(axis1);
			m_calculatedAxis[0] = axis1.cross(m_calculatedAxis[2]);
			m_calculatedAxis[1] = m_calculatedAxis[2].cross(axis0);
			break;
		}
	case RO_YXZ :
		{
			//planes: yx,XZ normals: z, Y
			//first rotate around z
			//second rotate around x'= z.cross(Y)
			//third rotate around y" = Y
			btlVector3 axis1 = m_calculatedTransformB.getBasis().getColumn(1);
			btlVector3 axis2 = m_calculatedTransformA.getBasis().getColumn(2);
			m_calculatedAxis[0] = axis1.cross(axis2);
			m_calculatedAxis[1] = axis2.cross(m_calculatedAxis[0]);
			m_calculatedAxis[2] = m_calculatedAxis[0].cross(axis1);
			break;
		}
	case RO_YZX :
		{
			//planes: yz,ZX normals: x, Y
			//first rotate around x
			//second rotate around z'= x.cross(Y)
			//third rotate around y" = Y
			btlVector3 axis0 = m_calculatedTransformA.getBasis().getColumn(0);
			btlVector3 axis1 = m_calculatedTransformB.getBasis().getColumn(1);
			m_calculatedAxis[2] = axis0.cross(axis1);
			m_calculatedAxis[0] = axis1.cross(m_calculatedAxis[2]);
			m_calculatedAxis[1] = m_calculatedAxis[2].cross(axis0);
			break;
		}
	case RO_ZXY :
		{
			//planes: zx,XY normals: y, Z
			//first rotate around y
			//second rotate around x'= y.cross(Z)
			//third rotate around z" = Z
			btlVector3 axis1 = m_calculatedTransformA.getBasis().getColumn(1);
			btlVector3 axis2 = m_calculatedTransformB.getBasis().getColumn(2);
			m_calculatedAxis[0] = axis1.cross(axis2);
			m_calculatedAxis[1] = axis2.cross(m_calculatedAxis[0]);
			m_calculatedAxis[2] = m_calculatedAxis[0].cross(axis1);
			break;
		}
	case RO_ZYX :
		{
			//planes: zy,YX normals: x, Z
			//first rotate around x
			//second rotate around y' = x.cross(Z)
			//third rotate around z" = Z
			btlVector3 axis0 = m_calculatedTransformA.getBasis().getColumn(0);
			btlVector3 axis2 = m_calculatedTransformB.getBasis().getColumn(2);
			m_calculatedAxis[1] = axis2.cross(axis0);
			m_calculatedAxis[0] = m_calculatedAxis[1].cross(axis2);
			m_calculatedAxis[2] = axis0.cross(m_calculatedAxis[1]);
			break;
		}
	default:
		btlAssert(false);
	}

	m_calculatedAxis[0].normalize();
	m_calculatedAxis[1].normalize();
	m_calculatedAxis[2].normalize();

}

void btlGeneric6DofSpring2Constraint::calculateTransforms()
{
	calculateTransforms(m_rbA.getCenterOfMassTransform(),m_rbB.getCenterOfMassTransform());
}

void btlGeneric6DofSpring2Constraint::calculateTransforms(const btlTransform& transA,const btlTransform& transB)
{
	m_calculatedTransformA = transA * m_frameInA;
	m_calculatedTransformB = transB * m_frameInB;
	calculateLinearInfo();
	calculateAngleInfo();

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


void btlGeneric6DofSpring2Constraint::testAngularLimitMotor(int axis_index)
{
	btlScalar angle = m_calculatedAxisAngleDiff[axis_index];
	angle = btlAdjustAngleToLimits(angle, m_angularLimits[axis_index].m_loLimit, m_angularLimits[axis_index].m_hiLimit);
	m_angularLimits[axis_index].m_currentPosition = angle;
	m_angularLimits[axis_index].testLimitValue(angle);
}


void btlGeneric6DofSpring2Constraint::getInfo1 (btlConstraintInfo1* info)
{
	//prepare constraint
	calculateTransforms(m_rbA.getCenterOfMassTransform(),m_rbB.getCenterOfMassTransform());
	info->m_numConstraintRows = 0;
	info->nub = 0;
	int i;
	//test linear limits
	for(i = 0; i < 3; i++)
	{
		     if (m_linearLimits.m_currentLimit[i]==4) info->m_numConstraintRows += 2;
		else if (m_linearLimits.m_currentLimit[i]!=0) info->m_numConstraintRows += 1;
		if (m_linearLimits.m_enableMotor[i] ) info->m_numConstraintRows += 1;
		if (m_linearLimits.m_enableSpring[i]) info->m_numConstraintRows += 1;
	}
	//test angular limits
	for (i=0;i<3 ;i++ )
	{
		testAngularLimitMotor(i);
		     if (m_angularLimits[i].m_currentLimit==4) info->m_numConstraintRows += 2;
		else if (m_angularLimits[i].m_currentLimit!=0) info->m_numConstraintRows += 1;
		if (m_angularLimits[i].m_enableMotor ) info->m_numConstraintRows += 1;
		if (m_angularLimits[i].m_enableSpring) info->m_numConstraintRows += 1;
	}
}


void btlGeneric6DofSpring2Constraint::getInfo2 (btlConstraintInfo2* info)
{
	const btlTransform& transA = m_rbA.getCenterOfMassTransform();
	const btlTransform& transB = m_rbB.getCenterOfMassTransform();
	const btlVector3& linVelA = m_rbA.getLinearVelocity();
	const btlVector3& linVelB = m_rbB.getLinearVelocity();
	const btlVector3& angVelA = m_rbA.getAngularVelocity();
	const btlVector3& angVelB = m_rbB.getAngularVelocity();

	// for stability better to solve angular limits first
	int row = setAngularLimits(info, 0,transA,transB,linVelA,linVelB,angVelA,angVelB);
	setLinearLimits(info, row, transA,transB,linVelA,linVelB,angVelA,angVelB);
}


int btlGeneric6DofSpring2Constraint::setLinearLimits(btlConstraintInfo2* info, int row, const btlTransform& transA,const btlTransform& transB,const btlVector3& linVelA,const btlVector3& linVelB,const btlVector3& angVelA,const btlVector3& angVelB)
{
	//solve linear limits
	btlRotationalLimitMotor2 limot;
	for (int i=0;i<3 ;i++ )
	{
		if(m_linearLimits.m_currentLimit[i] || m_linearLimits.m_enableMotor[i] || m_linearLimits.m_enableSpring[i])
		{ // re-use rotational motor code
			limot.m_bounce                 = m_linearLimits.m_bounce[i];
			limot.m_currentLimit           = m_linearLimits.m_currentLimit[i];
			limot.m_currentPosition        = m_linearLimits.m_currentLinearDiff[i];
			limot.m_currentLimitError      = m_linearLimits.m_currentLimitError[i];
			limot.m_currentLimitErrorHi    = m_linearLimits.m_currentLimitErrorHi[i];
			limot.m_enableMotor            = m_linearLimits.m_enableMotor[i];
			limot.m_servoMotor             = m_linearLimits.m_servoMotor[i];
			limot.m_servoTarget            = m_linearLimits.m_servoTarget[i];
			limot.m_enableSpring           = m_linearLimits.m_enableSpring[i];
			limot.m_springStiffness        = m_linearLimits.m_springStiffness[i];
			limot.m_springStiffnessLimited = m_linearLimits.m_springStiffnessLimited[i];
			limot.m_springDamping          = m_linearLimits.m_springDamping[i];
			limot.m_springDampingLimited   = m_linearLimits.m_springDampingLimited[i];
			limot.m_equilibriumPoint       = m_linearLimits.m_equilibriumPoint[i];
			limot.m_hiLimit                = m_linearLimits.m_upperLimit[i];
			limot.m_loLimit                = m_linearLimits.m_lowerLimit[i];
			limot.m_maxMotorForce          = m_linearLimits.m_maxMotorForce[i];
			limot.m_targetVelocity         = m_linearLimits.m_targetVelocity[i];
			btlVector3 axis = m_calculatedTransformA.getBasis().getColumn(i);
			int flags = m_flags >> (i * BT_6DOF_FLAGS_AXIS_SHIFT2);
			limot.m_stopCFM  = (flags & BT_6DOF_FLAGS_CFM_STOP2) ? m_linearLimits.m_stopCFM[i] : info->cfm[0];
			limot.m_stopERP  = (flags & BT_6DOF_FLAGS_ERP_STOP2) ? m_linearLimits.m_stopERP[i] : info->erp;
			limot.m_motorCFM = (flags & BT_6DOF_FLAGS_CFM_MOTO2) ? m_linearLimits.m_motorCFM[i] : info->cfm[0];
			limot.m_motorERP = (flags & BT_6DOF_FLAGS_ERP_MOTO2) ? m_linearLimits.m_motorERP[i] : info->erp;

			//rotAllowed is a bit of a magic from the original 6dof. The calculation of it here is something that imitates the original behavior as much as possible.
			int indx1 = (i + 1) % 3;
			int indx2 = (i + 2) % 3;
			int rotAllowed = 1; // rotations around orthos to current axis (it is used only when one of the body is static)
			#define D6_LIMIT_ERROR_THRESHOLD_FOR_ROTATION 1.0e-3
			bool indx1Violated = m_angularLimits[indx1].m_currentLimit == 1 ||
				m_angularLimits[indx1].m_currentLimit == 2 ||
				( m_angularLimits[indx1].m_currentLimit == 3 && ( m_angularLimits[indx1].m_currentLimitError < -D6_LIMIT_ERROR_THRESHOLD_FOR_ROTATION || m_angularLimits[indx1].m_currentLimitError > D6_LIMIT_ERROR_THRESHOLD_FOR_ROTATION ) ) ||
				( m_angularLimits[indx1].m_currentLimit == 4 && ( m_angularLimits[indx1].m_currentLimitError < -D6_LIMIT_ERROR_THRESHOLD_FOR_ROTATION || m_angularLimits[indx1].m_currentLimitErrorHi > D6_LIMIT_ERROR_THRESHOLD_FOR_ROTATION ) );
			bool indx2Violated = m_angularLimits[indx2].m_currentLimit == 1 ||
				m_angularLimits[indx2].m_currentLimit == 2 ||
				( m_angularLimits[indx2].m_currentLimit == 3 && ( m_angularLimits[indx2].m_currentLimitError < -D6_LIMIT_ERROR_THRESHOLD_FOR_ROTATION || m_angularLimits[indx2].m_currentLimitError > D6_LIMIT_ERROR_THRESHOLD_FOR_ROTATION ) ) ||
				( m_angularLimits[indx2].m_currentLimit == 4 && ( m_angularLimits[indx2].m_currentLimitError < -D6_LIMIT_ERROR_THRESHOLD_FOR_ROTATION || m_angularLimits[indx2].m_currentLimitErrorHi > D6_LIMIT_ERROR_THRESHOLD_FOR_ROTATION ) );
			if( indx1Violated && indx2Violated )
			{
				rotAllowed = 0;
			}
			row += get_limit_motor_info2(&limot, transA,transB,linVelA,linVelB,angVelA,angVelB, info, row, axis, 0, rotAllowed);

		}
	}
	return row;
}



int btlGeneric6DofSpring2Constraint::setAngularLimits(btlConstraintInfo2 *info, int row_offset, const btlTransform& transA,const btlTransform& transB,const btlVector3& linVelA,const btlVector3& linVelB,const btlVector3& angVelA,const btlVector3& angVelB)
{
	int row = row_offset;

	//order of rotational constraint rows
	int cIdx[] = {0, 1, 2};
	switch(m_rotateOrder)
	{
		case RO_XYZ : cIdx[0] = 0; cIdx[1] = 1; cIdx[2] = 2; break;
		case RO_XZY : cIdx[0] = 0; cIdx[1] = 2; cIdx[2] = 1; break;
		case RO_YXZ : cIdx[0] = 1; cIdx[1] = 0; cIdx[2] = 2; break;
		case RO_YZX : cIdx[0] = 1; cIdx[1] = 2; cIdx[2] = 0; break;
		case RO_ZXY : cIdx[0] = 2; cIdx[1] = 0; cIdx[2] = 1; break;
		case RO_ZYX : cIdx[0] = 2; cIdx[1] = 1; cIdx[2] = 0; break;
		default : btlAssert(false);
	}

	for (int ii = 0; ii < 3 ; ii++ )
	{
		int i = cIdx[ii];
		if(m_angularLimits[i].m_currentLimit || m_angularLimits[i].m_enableMotor || m_angularLimits[i].m_enableSpring)
		{
			btlVector3 axis = getAxis(i);
			int flags = m_flags >> ((i + 3) * BT_6DOF_FLAGS_AXIS_SHIFT2);
			if(!(flags & BT_6DOF_FLAGS_CFM_STOP2))
			{
				m_angularLimits[i].m_stopCFM = info->cfm[0];
			}
			if(!(flags & BT_6DOF_FLAGS_ERP_STOP2))
			{
				m_angularLimits[i].m_stopERP = info->erp;
			}
			if(!(flags & BT_6DOF_FLAGS_CFM_MOTO2))
			{
				m_angularLimits[i].m_motorCFM = info->cfm[0];
			}
			if(!(flags & BT_6DOF_FLAGS_ERP_MOTO2))
			{
				m_angularLimits[i].m_motorERP = info->erp;
			}
			row += get_limit_motor_info2(&m_angularLimits[i],transA,transB,linVelA,linVelB,angVelA,angVelB, info,row,axis,1);
		}
	}

	return row;
}


void btlGeneric6DofSpring2Constraint::setFrames(const btlTransform& frameA, const btlTransform& frameB)
{
	m_frameInA = frameA;
	m_frameInB = frameB;
	buildJacobian();
	calculateTransforms();
}


void btlGeneric6DofSpring2Constraint::calculateLinearInfo()
{
	m_calculatedLinearDiff = m_calculatedTransformB.getOrigin() - m_calculatedTransformA.getOrigin();
	m_calculatedLinearDiff = m_calculatedTransformA.getBasis().inverse() * m_calculatedLinearDiff;
	for(int i = 0; i < 3; i++)
	{
		m_linearLimits.m_currentLinearDiff[i] = m_calculatedLinearDiff[i];
		m_linearLimits.testLimitValue(i, m_calculatedLinearDiff[i]);
	}
}

void btlGeneric6DofSpring2Constraint::calculateJacobi(btlRotationalLimitMotor2 * limot, const btlTransform& transA,const btlTransform& transB, btlConstraintInfo2 *info, int srow, btlVector3& ax1, int rotational, int rotAllowed)
{
	btlScalar *J1 = rotational ? info->m_J1angularAxis : info->m_J1linearAxis;
	btlScalar *J2 = rotational ? info->m_J2angularAxis : info->m_J2linearAxis;

	J1[srow+0] = ax1[0];
	J1[srow+1] = ax1[1];
	J1[srow+2] = ax1[2];

	J2[srow+0] = -ax1[0];
	J2[srow+1] = -ax1[1];
	J2[srow+2] = -ax1[2];

	if(!rotational)
	{
		btlVector3 tmpA, tmpB, relA, relB;
		// get vector from bodyB to frameB in WCS
		relB = m_calculatedTransformB.getOrigin() - transB.getOrigin();
		// same for bodyA
		relA = m_calculatedTransformA.getOrigin() - transA.getOrigin();
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
	}
}


int btlGeneric6DofSpring2Constraint::get_limit_motor_info2(
	btlRotationalLimitMotor2 * limot,
	const btlTransform& transA,const btlTransform& transB,const btlVector3& linVelA,const btlVector3& linVelB,const btlVector3& angVelA,const btlVector3& angVelB,
	btlConstraintInfo2 *info, int row, btlVector3& ax1, int rotational,int rotAllowed)
{
	int count = 0;
	int srow = row * info->rowskip;

	if (limot->m_currentLimit==4) 
	{
		btlScalar vel = rotational ? angVelA.dot(ax1) - angVelB.dot(ax1) : linVelA.dot(ax1) - linVelB.dot(ax1);

		calculateJacobi(limot,transA,transB,info,srow,ax1,rotational,rotAllowed);
		info->m_constraintError[srow] = info->fps * limot->m_stopERP * limot->m_currentLimitError * (rotational ? -1 : 1);
		if (rotational) {
			if (info->m_constraintError[srow]-vel*limot->m_stopERP > 0) {
				btlScalar bounceerror = -limot->m_bounce* vel;
				if (bounceerror > info->m_constraintError[srow]) info->m_constraintError[srow] = bounceerror;
			}
		} else {
			if (info->m_constraintError[srow]-vel*limot->m_stopERP < 0) {
				btlScalar bounceerror = -limot->m_bounce* vel;
				if (bounceerror < info->m_constraintError[srow]) info->m_constraintError[srow] = bounceerror;
			}
		}
		info->m_lowerLimit[srow] = rotational ? 0 : -SIMD_INFINITY;
		info->m_upperLimit[srow] = rotational ? SIMD_INFINITY : 0;
		info->cfm[srow] = limot->m_stopCFM;
		srow += info->rowskip;
		++count;

		calculateJacobi(limot,transA,transB,info,srow,ax1,rotational,rotAllowed);
		info->m_constraintError[srow] = info->fps * limot->m_stopERP * limot->m_currentLimitErrorHi * (rotational ? -1 : 1);
		if (rotational) {
			if (info->m_constraintError[srow]-vel*limot->m_stopERP < 0) {
				btlScalar bounceerror = -limot->m_bounce* vel;
				if (bounceerror < info->m_constraintError[srow]) info->m_constraintError[srow] = bounceerror;
			}
		} else {
			if (info->m_constraintError[srow]-vel*limot->m_stopERP > 0) {
				btlScalar bounceerror = -limot->m_bounce* vel;
				if (bounceerror > info->m_constraintError[srow]) info->m_constraintError[srow] = bounceerror;
			}
		}
		info->m_lowerLimit[srow] = rotational ? -SIMD_INFINITY : 0;
		info->m_upperLimit[srow] = rotational ? 0 : SIMD_INFINITY;
		info->cfm[srow] = limot->m_stopCFM;
		srow += info->rowskip;
		++count;
	} else
	if (limot->m_currentLimit==3) 
	{
		calculateJacobi(limot,transA,transB,info,srow,ax1,rotational,rotAllowed);
		info->m_constraintError[srow] = info->fps * limot->m_stopERP * limot->m_currentLimitError * (rotational ? -1 : 1);
		info->m_lowerLimit[srow] = -SIMD_INFINITY;
		info->m_upperLimit[srow] = SIMD_INFINITY;
		info->cfm[srow] = limot->m_stopCFM;
		srow += info->rowskip;
		++count;
	}

	if (limot->m_enableMotor && !limot->m_servoMotor)
	{
		calculateJacobi(limot,transA,transB,info,srow,ax1,rotational,rotAllowed);
		btlScalar tag_vel = rotational ? limot->m_targetVelocity : -limot->m_targetVelocity;
		btlScalar mot_fact = getMotorFactor(limot->m_currentPosition, 
			limot->m_loLimit,
			limot->m_hiLimit,
			tag_vel,
			info->fps * limot->m_motorERP);
		info->m_constraintError[srow] = mot_fact * limot->m_targetVelocity;
		info->m_lowerLimit[srow] = -limot->m_maxMotorForce;
		info->m_upperLimit[srow] = limot->m_maxMotorForce;
		info->cfm[srow] = limot->m_motorCFM;
		srow += info->rowskip;
		++count;
	}

	if (limot->m_enableMotor && limot->m_servoMotor)
	{
		btlScalar error = limot->m_currentPosition - limot->m_servoTarget;
		calculateJacobi(limot,transA,transB,info,srow,ax1,rotational,rotAllowed);
		btlScalar targetvelocity = error<0 ? -limot->m_targetVelocity : limot->m_targetVelocity;
		btlScalar tag_vel = -targetvelocity;
		btlScalar mot_fact;
		if(error != 0)
		{
			btlScalar lowLimit;
			btlScalar hiLimit;
			if(limot->m_loLimit > limot->m_hiLimit)
			{
				lowLimit = error > 0 ? limot->m_servoTarget : -SIMD_INFINITY;
				hiLimit  = error < 0 ? limot->m_servoTarget :  SIMD_INFINITY;
			}
			else
			{
				lowLimit = error > 0 && limot->m_servoTarget>limot->m_loLimit ? limot->m_servoTarget : limot->m_loLimit;
				hiLimit  = error < 0 && limot->m_servoTarget<limot->m_hiLimit ? limot->m_servoTarget : limot->m_hiLimit;
			}
			mot_fact = getMotorFactor(limot->m_currentPosition, lowLimit, hiLimit, tag_vel, info->fps * limot->m_motorERP);
		} 
		else 
		{
			mot_fact = 0;
		}
		info->m_constraintError[srow] = mot_fact * targetvelocity * (rotational ? -1 : 1);
		info->m_lowerLimit[srow] = -limot->m_maxMotorForce;
		info->m_upperLimit[srow] = limot->m_maxMotorForce;
		info->cfm[srow] = limot->m_motorCFM;
		srow += info->rowskip;
		++count;
	}

	if (limot->m_enableSpring)
	{
		btlScalar error = limot->m_currentPosition - limot->m_equilibriumPoint;
		calculateJacobi(limot,transA,transB,info,srow,ax1,rotational,rotAllowed);

		//btScalar cfm = 1.0 / ((1.0/info->fps)*limot->m_springStiffness+ limot->m_springDamping);
		//if(cfm > 0.99999)
		//	cfm = 0.99999;
		//btScalar erp = (1.0/info->fps)*limot->m_springStiffness / ((1.0/info->fps)*limot->m_springStiffness + limot->m_springDamping);
		//info->m_constraintError[srow] = info->fps * erp * error * (rotational ? -1.0 : 1.0);
		//info->m_lowerLimit[srow] = -SIMD_INFINITY;
		//info->m_upperLimit[srow] = SIMD_INFINITY;

		btlScalar dt = BT_ONE / info->fps;
		btlScalar kd = limot->m_springDamping;
		btlScalar ks = limot->m_springStiffness;
		btlScalar vel = rotational ? angVelA.dot(ax1) - angVelB.dot(ax1) : linVelA.dot(ax1) - linVelB.dot(ax1);
//		btScalar erp = 0.1;
		btlScalar cfm = BT_ZERO;
		btlScalar mA = BT_ONE / m_rbA.getInvMass();
		btlScalar mB = BT_ONE / m_rbB.getInvMass();
		btlScalar m = mA > mB ? mB : mA;
		btlScalar angularfreq = sqrt(ks / m);


		//limit stiffness (the spring should not be sampled faster that the quarter of its angular frequency)
		if(limot->m_springStiffnessLimited && 0.25 < angularfreq * dt)
		{
			ks = BT_ONE / dt / dt / btlScalar(16.0) * m;
		}
		//avoid damping that would blow up the spring
		if(limot->m_springDampingLimited && kd * dt > m)
		{
			kd = m / dt;
		}
		btlScalar fs = ks * error * dt;
		btlScalar fd = -kd * (vel) * (rotational ? -1 : 1) * dt;
		btlScalar f = (fs+fd);

		info->m_constraintError[srow] = (vel + f * (rotational ? -1 : 1)) ;

		btlScalar minf = f < fd ? f : fd;
		btlScalar maxf = f < fd ? fd : f;
		if(!rotational)
		{
			info->m_lowerLimit[srow] = minf > 0 ? 0 : minf;
			info->m_upperLimit[srow] = maxf < 0 ? 0 : maxf;
		}
		else
		{
			info->m_lowerLimit[srow] = -maxf > 0 ? 0 : -maxf;
			info->m_upperLimit[srow] = -minf < 0 ? 0 : -minf;
		}

		info->cfm[srow] = cfm;
		srow += info->rowskip;
		++count;
	}

	return count;
}


//override the default global value of a parameter (such as ERP or CFM), optionally provide the axis (0..5). 
//If no axis is provided, it uses the default axis for this constraint.
void btlGeneric6DofSpring2Constraint::setParam(int num, btlScalar value, int axis)
{
	if((axis >= 0) && (axis < 3))
	{
		switch(num)
		{
			case BT_CONSTRAINT_STOP_ERP : 
				m_linearLimits.m_stopERP[axis] = value;
				m_flags |= BT_6DOF_FLAGS_ERP_STOP2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2);
				break;
			case BT_CONSTRAINT_STOP_CFM : 
				m_linearLimits.m_stopCFM[axis] = value;
				m_flags |= BT_6DOF_FLAGS_CFM_STOP2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2);
				break;
			case BT_CONSTRAINT_ERP : 
				m_linearLimits.m_motorERP[axis] = value;
				m_flags |= BT_6DOF_FLAGS_ERP_MOTO2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2);
				break;
			case BT_CONSTRAINT_CFM : 
				m_linearLimits.m_motorCFM[axis] = value;
				m_flags |= BT_6DOF_FLAGS_CFM_MOTO2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2);
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
				m_flags |= BT_6DOF_FLAGS_ERP_STOP2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2);
				break;
			case BT_CONSTRAINT_STOP_CFM : 
				m_angularLimits[axis - 3].m_stopCFM = value;
				m_flags |= BT_6DOF_FLAGS_CFM_STOP2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2);
				break;
			case BT_CONSTRAINT_ERP : 
				m_angularLimits[axis - 3].m_motorERP = value;
				m_flags |= BT_6DOF_FLAGS_ERP_MOTO2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2);
				break;
			case BT_CONSTRAINT_CFM : 
				m_angularLimits[axis - 3].m_motorCFM = value;
				m_flags |= BT_6DOF_FLAGS_CFM_MOTO2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2);
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

//return the local value of parameter
btlScalar btlGeneric6DofSpring2Constraint::getParam(int num, int axis) const 
{
	btlScalar retVal = 0;
	if((axis >= 0) && (axis < 3))
	{
		switch(num)
		{
			case BT_CONSTRAINT_STOP_ERP : 
				btlAssertConstrParams(m_flags & (BT_6DOF_FLAGS_ERP_STOP2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2)));
				retVal = m_linearLimits.m_stopERP[axis];
				break;
			case BT_CONSTRAINT_STOP_CFM : 
				btlAssertConstrParams(m_flags & (BT_6DOF_FLAGS_CFM_STOP2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2)));
				retVal = m_linearLimits.m_stopCFM[axis];
				break;
			case BT_CONSTRAINT_ERP : 
				btlAssertConstrParams(m_flags & (BT_6DOF_FLAGS_ERP_MOTO2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2)));
				retVal = m_linearLimits.m_motorERP[axis];
				break;
			case BT_CONSTRAINT_CFM : 
				btlAssertConstrParams(m_flags & (BT_6DOF_FLAGS_CFM_MOTO2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2)));
				retVal = m_linearLimits.m_motorCFM[axis];
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
				btlAssertConstrParams(m_flags & (BT_6DOF_FLAGS_ERP_STOP2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2)));
				retVal = m_angularLimits[axis - 3].m_stopERP;
				break;
			case BT_CONSTRAINT_STOP_CFM : 
				btlAssertConstrParams(m_flags & (BT_6DOF_FLAGS_CFM_STOP2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2)));
				retVal = m_angularLimits[axis - 3].m_stopCFM;
				break;
			case BT_CONSTRAINT_ERP : 
				btlAssertConstrParams(m_flags & (BT_6DOF_FLAGS_ERP_MOTO2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2)));
				retVal = m_angularLimits[axis - 3].m_motorERP;
				break;
			case BT_CONSTRAINT_CFM : 
				btlAssertConstrParams(m_flags & (BT_6DOF_FLAGS_CFM_MOTO2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2)));
				retVal = m_angularLimits[axis - 3].m_motorCFM;
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

 

void btlGeneric6DofSpring2Constraint::setAxis(const btlVector3& axis1,const btlVector3& axis2)
{
	btlVector3 zAxis = axis1.normalized();
	btlVector3 yAxis = axis2.normalized();
	btlVector3 xAxis = yAxis.cross(zAxis); // we want right coordinate system
	
	btlTransform frameInW;
	frameInW.setIdentity();
	frameInW.getBasis().setValue( xAxis[0], yAxis[0], zAxis[0],
	                              xAxis[1], yAxis[1], zAxis[1],
	                              xAxis[2], yAxis[2], zAxis[2]);
	
	// now get constraint frame in local coordinate systems
	m_frameInA = m_rbA.getCenterOfMassTransform().inverse() * frameInW;
	m_frameInB = m_rbB.getCenterOfMassTransform().inverse() * frameInW;
	
	calculateTransforms();
}

void btlGeneric6DofSpring2Constraint::setBounce(int index, btlScalar bounce)
{
	btlAssert((index >= 0) && (index < 6));
	if (index<3)
		m_linearLimits.m_bounce[index] = bounce;
	else
		m_angularLimits[index - 3].m_bounce = bounce;
}

void btlGeneric6DofSpring2Constraint::enableMotor(int index, bool onOff)
{
	btlAssert((index >= 0) && (index < 6));
	if (index<3)
		m_linearLimits.m_enableMotor[index] = onOff;
	else
		m_angularLimits[index - 3].m_enableMotor = onOff;
}

void btlGeneric6DofSpring2Constraint::setServo(int index, bool onOff)
{
	btlAssert((index >= 0) && (index < 6));
	if (index<3)
		m_linearLimits.m_servoMotor[index] = onOff;
	else
		m_angularLimits[index - 3].m_servoMotor = onOff;
}

void btlGeneric6DofSpring2Constraint::setTargetVelocity(int index, btlScalar velocity)
{
	btlAssert((index >= 0) && (index < 6));
	if (index<3)
		m_linearLimits.m_targetVelocity[index] = velocity;
	else
		m_angularLimits[index - 3].m_targetVelocity = velocity;
}

void btlGeneric6DofSpring2Constraint::setServoTarget(int index, btlScalar target)
{
	btlAssert((index >= 0) && (index < 6));
	if (index<3)
		m_linearLimits.m_servoTarget[index] = target;
	else
		m_angularLimits[index - 3].m_servoTarget = target;
}

void btlGeneric6DofSpring2Constraint::setMaxMotorForce(int index, btlScalar force)
{
	btlAssert((index >= 0) && (index < 6));
	if (index<3)
		m_linearLimits.m_maxMotorForce[index] = force;
	else
		m_angularLimits[index - 3].m_maxMotorForce = force;
}

void btlGeneric6DofSpring2Constraint::enableSpring(int index, bool onOff)
{
	btlAssert((index >= 0) && (index < 6));
	if (index<3)
		m_linearLimits.m_enableSpring[index] = onOff;
	else
		m_angularLimits[index - 3] .m_enableSpring = onOff;
}

void btlGeneric6DofSpring2Constraint::setStiffness(int index, btlScalar stiffness, bool limitIfNeeded)
{
	btlAssert((index >= 0) && (index < 6));
	if (index<3) {
		m_linearLimits.m_springStiffness[index] = stiffness;
		m_linearLimits.m_springStiffnessLimited[index] = limitIfNeeded;
	} else {
		m_angularLimits[index - 3].m_springStiffness = stiffness;
		m_angularLimits[index - 3].m_springStiffnessLimited = limitIfNeeded;
	}
}

void btlGeneric6DofSpring2Constraint::setDamping(int index, btlScalar damping, bool limitIfNeeded)
{
	btlAssert((index >= 0) && (index < 6));
	if (index<3) {
		m_linearLimits.m_springDamping[index] = damping;
		m_linearLimits.m_springDampingLimited[index] = limitIfNeeded;
	} else {
		m_angularLimits[index - 3].m_springDamping = damping;
		m_angularLimits[index - 3].m_springDampingLimited = limitIfNeeded;
	}
}

void btlGeneric6DofSpring2Constraint::setEquilibriumPoint()
{
	calculateTransforms();
	int i;
	for( i = 0; i < 3; i++)
		m_linearLimits.m_equilibriumPoint[i] = m_calculatedLinearDiff[i];
	for(i = 0; i < 3; i++)
		m_angularLimits[i].m_equilibriumPoint = m_calculatedAxisAngleDiff[i];
}

void btlGeneric6DofSpring2Constraint::setEquilibriumPoint(int index)
{
	btlAssert((index >= 0) && (index < 6));
	calculateTransforms();
	if (index<3)
		m_linearLimits.m_equilibriumPoint[index] = m_calculatedLinearDiff[index];
	else
		m_angularLimits[index - 3] .m_equilibriumPoint = m_calculatedAxisAngleDiff[index - 3];
}

void btlGeneric6DofSpring2Constraint::setEquilibriumPoint(int index, btlScalar val)
{
	btlAssert((index >= 0) && (index < 6));
	if (index<3)
		m_linearLimits.m_equilibriumPoint[index] = val;
	else
		m_angularLimits[index - 3] .m_equilibriumPoint = val;
}


//////////////////////////// btRotationalLimitMotor2 ////////////////////////////////////

void btlRotationalLimitMotor2::testLimitValue(btlScalar test_value)
{
	//we can't normalize the angles here because we would lost the sign that we use later, but it doesn't seem to be a problem
	if(m_loLimit > m_hiLimit) {
		m_currentLimit = 0;
		m_currentLimitError = btlScalar(0.f);
	}
	else if(m_loLimit == m_hiLimit) {
		m_currentLimitError = test_value - m_loLimit;
		m_currentLimit = 3;
	} else {
		m_currentLimitError = test_value - m_loLimit;
		m_currentLimitErrorHi = test_value - m_hiLimit;
		m_currentLimit = 4;
	}
}

//////////////////////////// btTranslationalLimitMotor2 ////////////////////////////////////

void btlTranslationalLimitMotor2::testLimitValue(int limitIndex, btlScalar test_value)
{
	btlScalar loLimit = m_lowerLimit[limitIndex];
	btlScalar hiLimit = m_upperLimit[limitIndex];
	if(loLimit > hiLimit) {
		m_currentLimitError[limitIndex] = 0;
		m_currentLimit[limitIndex] = 0;
	}
	else if(loLimit == hiLimit) {
		m_currentLimitError[limitIndex] = test_value - loLimit;
		m_currentLimit[limitIndex] = 3;
	} else {
		m_currentLimitError[limitIndex] = test_value - loLimit;
		m_currentLimitErrorHi[limitIndex] = test_value - hiLimit;
		m_currentLimit[limitIndex] = 4;
	}
}


