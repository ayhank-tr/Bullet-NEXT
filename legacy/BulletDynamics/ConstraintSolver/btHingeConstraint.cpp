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


#include "btHingeConstraint.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "LinearMath/btTransformUtil.h"
#include "LinearMath/btMinMax.h"
#include <new>
#include "btSolverBody.h"



//#define HINGE_USE_OBSOLETE_SOLVER false
#define HINGE_USE_OBSOLETE_SOLVER false

#define HINGE_USE_FRAME_OFFSET true

#ifndef __SPU__





btlHingeConstraint::btlHingeConstraint(btlRigidBody& rbA,btlRigidBody& rbB, const btlVector3& pivotInA,const btlVector3& pivotInB,
									 const btlVector3& axisInA,const btlVector3& axisInB, bool useReferenceFrameA)
									 :btlTypedConstraint(HINGE_CONSTRAINT_TYPE, rbA,rbB),
#ifdef _BT_USE_CENTER_LIMIT_
									 m_limit(),
#endif
									 m_angularOnly(false),
									 m_enableAngularMotor(false),
									 m_useSolveConstraintObsolete(HINGE_USE_OBSOLETE_SOLVER),
									 m_useOffsetForConstraintFrame(HINGE_USE_FRAME_OFFSET),
									 m_useReferenceFrameA(useReferenceFrameA),
									 m_flags(0),
									 m_normalCFM(0),
									 m_normalERP(0),
									 m_stopCFM(0),
									 m_stopERP(0)
{
	m_rbAFrame.getOrigin() = pivotInA;
	
	// since no frame is given, assume this to be zero angle and just pick rb transform axis
	btlVector3 rbAxisA1 = rbA.getCenterOfMassTransform().getBasis().getColumn(0);

	btlVector3 rbAxisA2;
	btlScalar projection = axisInA.dot(rbAxisA1);
	if (projection >= 1.0f - SIMD_EPSILON) {
		rbAxisA1 = -rbA.getCenterOfMassTransform().getBasis().getColumn(2);
		rbAxisA2 = rbA.getCenterOfMassTransform().getBasis().getColumn(1);
	} else if (projection <= -1.0f + SIMD_EPSILON) {
		rbAxisA1 = rbA.getCenterOfMassTransform().getBasis().getColumn(2);
		rbAxisA2 = rbA.getCenterOfMassTransform().getBasis().getColumn(1);      
	} else {
		rbAxisA2 = axisInA.cross(rbAxisA1);
		rbAxisA1 = rbAxisA2.cross(axisInA);
	}

	m_rbAFrame.getBasis().setValue( rbAxisA1.getX(),rbAxisA2.getX(),axisInA.getX(),
									rbAxisA1.getY(),rbAxisA2.getY(),axisInA.getY(),
									rbAxisA1.getZ(),rbAxisA2.getZ(),axisInA.getZ() );

	btlQuaternion rotationArc = shortestArcQuat(axisInA,axisInB);
	btlVector3 rbAxisB1 =  btl_quatRotate(rotationArc,rbAxisA1);
	btlVector3 rbAxisB2 =  axisInB.cross(rbAxisB1);	
	
	m_rbBFrame.getOrigin() = pivotInB;
	m_rbBFrame.getBasis().setValue( rbAxisB1.getX(),rbAxisB2.getX(),axisInB.getX(),
									rbAxisB1.getY(),rbAxisB2.getY(),axisInB.getY(),
									rbAxisB1.getZ(),rbAxisB2.getZ(),axisInB.getZ() );
	
#ifndef	_BT_USE_CENTER_LIMIT_
	//start with free
	m_lowerLimit = btlScalar(1.0f);
	m_upperLimit = btlScalar(-1.0f);
	m_biasFactor = 0.3f;
	m_relaxationFactor = 1.0f;
	m_limitSoftness = 0.9f;
	m_solveLimit = false;
#endif
	m_referenceSign = m_useReferenceFrameA ? btlScalar(-1.f) : btlScalar(1.f);
}



btlHingeConstraint::btlHingeConstraint(btlRigidBody& rbA,const btlVector3& pivotInA,const btlVector3& axisInA, bool useReferenceFrameA)
:btlTypedConstraint(HINGE_CONSTRAINT_TYPE, rbA),
#ifdef _BT_USE_CENTER_LIMIT_
m_limit(),
#endif
m_angularOnly(false), m_enableAngularMotor(false), 
m_useSolveConstraintObsolete(HINGE_USE_OBSOLETE_SOLVER),
m_useOffsetForConstraintFrame(HINGE_USE_FRAME_OFFSET),
m_useReferenceFrameA(useReferenceFrameA),
m_flags(0),
m_normalCFM(0),
m_normalERP(0),
m_stopCFM(0),
m_stopERP(0)
{

	// since no frame is given, assume this to be zero angle and just pick rb transform axis
	// fixed axis in worldspace
	btlVector3 rbAxisA1, rbAxisA2;
	btlPlaneSpace1(axisInA, rbAxisA1, rbAxisA2);

	m_rbAFrame.getOrigin() = pivotInA;
	m_rbAFrame.getBasis().setValue( rbAxisA1.getX(),rbAxisA2.getX(),axisInA.getX(),
									rbAxisA1.getY(),rbAxisA2.getY(),axisInA.getY(),
									rbAxisA1.getZ(),rbAxisA2.getZ(),axisInA.getZ() );

	btlVector3 axisInB = rbA.getCenterOfMassTransform().getBasis() * axisInA;

	btlQuaternion rotationArc = shortestArcQuat(axisInA,axisInB);
	btlVector3 rbAxisB1 =  btl_quatRotate(rotationArc,rbAxisA1);
	btlVector3 rbAxisB2 = axisInB.cross(rbAxisB1);


	m_rbBFrame.getOrigin() = rbA.getCenterOfMassTransform()(pivotInA);
	m_rbBFrame.getBasis().setValue( rbAxisB1.getX(),rbAxisB2.getX(),axisInB.getX(),
									rbAxisB1.getY(),rbAxisB2.getY(),axisInB.getY(),
									rbAxisB1.getZ(),rbAxisB2.getZ(),axisInB.getZ() );
	
#ifndef	_BT_USE_CENTER_LIMIT_
	//start with free
	m_lowerLimit = btlScalar(1.0f);
	m_upperLimit = btlScalar(-1.0f);
	m_biasFactor = 0.3f;
	m_relaxationFactor = 1.0f;
	m_limitSoftness = 0.9f;
	m_solveLimit = false;
#endif
	m_referenceSign = m_useReferenceFrameA ? btlScalar(-1.f) : btlScalar(1.f);
}



btlHingeConstraint::btlHingeConstraint(btlRigidBody& rbA,btlRigidBody& rbB, 
								     const btlTransform& rbAFrame, const btlTransform& rbBFrame, bool useReferenceFrameA)
:btlTypedConstraint(HINGE_CONSTRAINT_TYPE, rbA,rbB),m_rbAFrame(rbAFrame),m_rbBFrame(rbBFrame),
#ifdef _BT_USE_CENTER_LIMIT_
m_limit(),
#endif
m_angularOnly(false),
m_enableAngularMotor(false),
m_useSolveConstraintObsolete(HINGE_USE_OBSOLETE_SOLVER),
m_useOffsetForConstraintFrame(HINGE_USE_FRAME_OFFSET),
m_useReferenceFrameA(useReferenceFrameA),
m_flags(0),
m_normalCFM(0),
m_normalERP(0),
m_stopCFM(0),
m_stopERP(0)
{
#ifndef	_BT_USE_CENTER_LIMIT_
	//start with free
	m_lowerLimit = btlScalar(1.0f);
	m_upperLimit = btlScalar(-1.0f);
	m_biasFactor = 0.3f;
	m_relaxationFactor = 1.0f;
	m_limitSoftness = 0.9f;
	m_solveLimit = false;
#endif
	m_referenceSign = m_useReferenceFrameA ? btlScalar(-1.f) : btlScalar(1.f);
}			



btlHingeConstraint::btlHingeConstraint(btlRigidBody& rbA, const btlTransform& rbAFrame, bool useReferenceFrameA)
:btlTypedConstraint(HINGE_CONSTRAINT_TYPE, rbA),m_rbAFrame(rbAFrame),m_rbBFrame(rbAFrame),
#ifdef _BT_USE_CENTER_LIMIT_
m_limit(),
#endif
m_angularOnly(false),
m_enableAngularMotor(false),
m_useSolveConstraintObsolete(HINGE_USE_OBSOLETE_SOLVER),
m_useOffsetForConstraintFrame(HINGE_USE_FRAME_OFFSET),
m_useReferenceFrameA(useReferenceFrameA),
m_flags(0),
m_normalCFM(0),
m_normalERP(0),
m_stopCFM(0),
m_stopERP(0)
{
	///not providing rigidbody B means implicitly using worldspace for body B

	m_rbBFrame.getOrigin() = m_rbA.getCenterOfMassTransform()(m_rbAFrame.getOrigin());
#ifndef	_BT_USE_CENTER_LIMIT_
	//start with free
	m_lowerLimit = btlScalar(1.0f);
	m_upperLimit = btlScalar(-1.0f);
	m_biasFactor = 0.3f;
	m_relaxationFactor = 1.0f;
	m_limitSoftness = 0.9f;
	m_solveLimit = false;
#endif
	m_referenceSign = m_useReferenceFrameA ? btlScalar(-1.f) : btlScalar(1.f);
}



void	btlHingeConstraint::buildJacobian()
{
	if (m_useSolveConstraintObsolete)
	{
		m_appliedImpulse = btlScalar(0.);
		m_accMotorImpulse = btlScalar(0.);

		if (!m_angularOnly)
		{
			btlVector3 pivotAInW = m_rbA.getCenterOfMassTransform()*m_rbAFrame.getOrigin();
			btlVector3 pivotBInW = m_rbB.getCenterOfMassTransform()*m_rbBFrame.getOrigin();
			btlVector3 relPos = pivotBInW - pivotAInW;

			btlVector3 normal[3];
			if (relPos.length2() > SIMD_EPSILON)
			{
				normal[0] = relPos.normalized();
			}
			else
			{
				normal[0].setValue(btlScalar(1.0),0,0);
			}

			btlPlaneSpace1(normal[0], normal[1], normal[2]);

			for (int i=0;i<3;i++)
			{
				new (&m_jac[i]) btlJacobianEntry(
				m_rbA.getCenterOfMassTransform().getBasis().transpose(),
				m_rbB.getCenterOfMassTransform().getBasis().transpose(),
				pivotAInW - m_rbA.getCenterOfMassPosition(),
				pivotBInW - m_rbB.getCenterOfMassPosition(),
				normal[i],
				m_rbA.getInvInertiaDiagLocal(),
				m_rbA.getInvMass(),
				m_rbB.getInvInertiaDiagLocal(),
				m_rbB.getInvMass());
			}
		}

		//calculate two perpendicular jointAxis, orthogonal to hingeAxis
		//these two jointAxis require equal angular velocities for both bodies

		//this is unused for now, it's a todo
		btlVector3 jointAxis0local;
		btlVector3 jointAxis1local;
		
		btlPlaneSpace1(m_rbAFrame.getBasis().getColumn(2),jointAxis0local,jointAxis1local);

		btlVector3 jointAxis0 = getRigidBodyA().getCenterOfMassTransform().getBasis() * jointAxis0local;
		btlVector3 jointAxis1 = getRigidBodyA().getCenterOfMassTransform().getBasis() * jointAxis1local;
		btlVector3 hingeAxisWorld = getRigidBodyA().getCenterOfMassTransform().getBasis() * m_rbAFrame.getBasis().getColumn(2);
			
		new (&m_jacAng[0])	btlJacobianEntry(jointAxis0,
			m_rbA.getCenterOfMassTransform().getBasis().transpose(),
			m_rbB.getCenterOfMassTransform().getBasis().transpose(),
			m_rbA.getInvInertiaDiagLocal(),
			m_rbB.getInvInertiaDiagLocal());

		new (&m_jacAng[1])	btlJacobianEntry(jointAxis1,
			m_rbA.getCenterOfMassTransform().getBasis().transpose(),
			m_rbB.getCenterOfMassTransform().getBasis().transpose(),
			m_rbA.getInvInertiaDiagLocal(),
			m_rbB.getInvInertiaDiagLocal());

		new (&m_jacAng[2])	btlJacobianEntry(hingeAxisWorld,
			m_rbA.getCenterOfMassTransform().getBasis().transpose(),
			m_rbB.getCenterOfMassTransform().getBasis().transpose(),
			m_rbA.getInvInertiaDiagLocal(),
			m_rbB.getInvInertiaDiagLocal());

			// clear accumulator
			m_accLimitImpulse = btlScalar(0.);

			// test angular limit
			testLimit(m_rbA.getCenterOfMassTransform(),m_rbB.getCenterOfMassTransform());

		//Compute K = J*W*J' for hinge axis
		btlVector3 axisA =  getRigidBodyA().getCenterOfMassTransform().getBasis() *  m_rbAFrame.getBasis().getColumn(2);
		m_kHinge =   1.0f / (getRigidBodyA().computeAngularImpulseDenominator(axisA) +
							 getRigidBodyB().computeAngularImpulseDenominator(axisA));

	}
}


#endif //__SPU__


static inline btlScalar btlNormalizeAnglePositive(btlScalar angle)
{
  return btlFmod(btlFmod(angle, btlScalar(2.0*SIMD_PI)) + btlScalar(2.0*SIMD_PI), btlScalar(2.0*SIMD_PI));
}



static btlScalar btlShortestAngularDistance(btlScalar accAngle, btlScalar curAngle)
{
	btlScalar result = btlNormalizeAngle(btlNormalizeAnglePositive(btlNormalizeAnglePositive(curAngle) -
	btlNormalizeAnglePositive(accAngle)));
	return result;
}

static btlScalar btlShortestAngleUpdate(btlScalar accAngle, btlScalar curAngle)
{
	btlScalar tol(0.3);
	btlScalar result = btlShortestAngularDistance(accAngle, curAngle);

	  if (btlFabs(result) > tol)
		return curAngle;
	  else
		return accAngle + result;

	return curAngle;
}


btlScalar btlHingeAccumulatedAngleConstraint::getAccumulatedHingeAngle()
{
	btlScalar hingeAngle = getHingeAngle();
	m_accumulatedAngle = btlShortestAngleUpdate(m_accumulatedAngle,hingeAngle);
	return m_accumulatedAngle;
}
void	btlHingeAccumulatedAngleConstraint::setAccumulatedHingeAngle(btlScalar accAngle)
{
	m_accumulatedAngle  = accAngle;
}

void btlHingeAccumulatedAngleConstraint::getInfo1(btlConstraintInfo1* info)
{
	//update m_accumulatedAngle
	btlScalar curHingeAngle = getHingeAngle();
	m_accumulatedAngle = btlShortestAngleUpdate(m_accumulatedAngle,curHingeAngle);

	btlHingeConstraint::getInfo1(info);
	
}


void btlHingeConstraint::getInfo1(btlConstraintInfo1* info)
{


	if (m_useSolveConstraintObsolete)
	{
		info->m_numConstraintRows = 0;
		info->nub = 0;
	}
	else
	{
		info->m_numConstraintRows = 5; // Fixed 3 linear + 2 angular
		info->nub = 1; 
		//always add the row, to avoid computation (data is not available yet)
		//prepare constraint
		testLimit(m_rbA.getCenterOfMassTransform(),m_rbB.getCenterOfMassTransform());
		if(getSolveLimit() || getEnableAngularMotor())
		{
			info->m_numConstraintRows++; // limit 3rd anguar as well
			info->nub--; 
		}

	}
}

void btlHingeConstraint::getInfo1NonVirtual(btlConstraintInfo1* info)
{
	if (m_useSolveConstraintObsolete)
	{
		info->m_numConstraintRows = 0;
		info->nub = 0;
	}
	else
	{
		//always add the 'limit' row, to avoid computation (data is not available yet)
		info->m_numConstraintRows = 6; // Fixed 3 linear + 2 angular
		info->nub = 0; 
	}
}

void btlHingeConstraint::getInfo2 (btlConstraintInfo2* info)
{
	if(m_useOffsetForConstraintFrame)
	{
		getInfo2InternalUsingFrameOffset(info, m_rbA.getCenterOfMassTransform(),m_rbB.getCenterOfMassTransform(),m_rbA.getAngularVelocity(),m_rbB.getAngularVelocity());
	}
	else
	{
		getInfo2Internal(info, m_rbA.getCenterOfMassTransform(),m_rbB.getCenterOfMassTransform(),m_rbA.getAngularVelocity(),m_rbB.getAngularVelocity());
	}
}


void	btlHingeConstraint::getInfo2NonVirtual (btlConstraintInfo2* info,const btlTransform& transA,const btlTransform& transB,const btlVector3& angVelA,const btlVector3& angVelB)
{
	///the regular (virtual) implementation getInfo2 already performs 'testLimit' during getInfo1, so we need to do it now
	testLimit(transA,transB);

	getInfo2Internal(info,transA,transB,angVelA,angVelB);
}


void btlHingeConstraint::getInfo2Internal(btlConstraintInfo2* info, const btlTransform& transA,const btlTransform& transB,const btlVector3& angVelA,const btlVector3& angVelB)
{

	btlAssert(!m_useSolveConstraintObsolete);
	int i, skip = info->rowskip;
	// transforms in world space
	btlTransform trA = transA*m_rbAFrame;
	btlTransform trB = transB*m_rbBFrame;
	// pivot point
	btlVector3 pivotAInW = trA.getOrigin();
	btlVector3 pivotBInW = trB.getOrigin();
#if 0
	if (0)
	{
		for (i=0;i<6;i++)
		{
			info->m_J1linearAxis[i*skip]=0;
			info->m_J1linearAxis[i*skip+1]=0;
			info->m_J1linearAxis[i*skip+2]=0;

			info->m_J1angularAxis[i*skip]=0;
			info->m_J1angularAxis[i*skip+1]=0;
			info->m_J1angularAxis[i*skip+2]=0;

			info->m_J2linearAxis[i*skip]=0;
			info->m_J2linearAxis[i*skip+1]=0;
			info->m_J2linearAxis[i*skip+2]=0;

			info->m_J2angularAxis[i*skip]=0;
			info->m_J2angularAxis[i*skip+1]=0;
			info->m_J2angularAxis[i*skip+2]=0;

			info->m_constraintError[i*skip]=0.f;
		}
	}
#endif //#if 0
	// linear (all fixed)

	if (!m_angularOnly)
	{
		info->m_J1linearAxis[0] = 1;
		info->m_J1linearAxis[skip + 1] = 1;
		info->m_J1linearAxis[2 * skip + 2] = 1;

		info->m_J2linearAxis[0] = -1;
		info->m_J2linearAxis[skip + 1] = -1;
		info->m_J2linearAxis[2 * skip + 2] = -1;
	}	




	btlVector3 a1 = pivotAInW - transA.getOrigin();
	{
		btlVector3* angular0 = (btlVector3*)(info->m_J1angularAxis);
		btlVector3* angular1 = (btlVector3*)(info->m_J1angularAxis + skip);
		btlVector3* angular2 = (btlVector3*)(info->m_J1angularAxis + 2 * skip);
		btlVector3 a1neg = -a1;
		a1neg.getSkewSymmetricMatrix(angular0,angular1,angular2);
	}
	btlVector3 a2 = pivotBInW - transB.getOrigin();
	{
		btlVector3* angular0 = (btlVector3*)(info->m_J2angularAxis);
		btlVector3* angular1 = (btlVector3*)(info->m_J2angularAxis + skip);
		btlVector3* angular2 = (btlVector3*)(info->m_J2angularAxis + 2 * skip);
		a2.getSkewSymmetricMatrix(angular0,angular1,angular2);
	}
	// linear RHS
	btlScalar normalErp = (m_flags & BT_HINGE_FLAGS_ERP_NORM) ? m_normalERP : info->erp;

    btlScalar k = info->fps * normalErp;
	if (!m_angularOnly)
	{
		for(i = 0; i < 3; i++)
		{
			info->m_constraintError[i * skip] = k * (pivotBInW[i] - pivotAInW[i]);
		}
	}
	// make rotations around X and Y equal
	// the hinge axis should be the only unconstrained
	// rotational axis, the angular velocity of the two bodies perpendicular to
	// the hinge axis should be equal. thus the constraint equations are
	//    p*w1 - p*w2 = 0
	//    q*w1 - q*w2 = 0
	// where p and q are unit vectors normal to the hinge axis, and w1 and w2
	// are the angular velocity vectors of the two bodies.
	// get hinge axis (Z)
	btlVector3 ax1 = trA.getBasis().getColumn(2);
	// get 2 orthos to hinge axis (X, Y)
	btlVector3 p = trA.getBasis().getColumn(0);
	btlVector3 q = trA.getBasis().getColumn(1);
	// set the two hinge angular rows 
    int s3 = 3 * info->rowskip;
    int s4 = 4 * info->rowskip;

	info->m_J1angularAxis[s3 + 0] = p[0];
	info->m_J1angularAxis[s3 + 1] = p[1];
	info->m_J1angularAxis[s3 + 2] = p[2];
	info->m_J1angularAxis[s4 + 0] = q[0];
	info->m_J1angularAxis[s4 + 1] = q[1];
	info->m_J1angularAxis[s4 + 2] = q[2];

	info->m_J2angularAxis[s3 + 0] = -p[0];
	info->m_J2angularAxis[s3 + 1] = -p[1];
	info->m_J2angularAxis[s3 + 2] = -p[2];
	info->m_J2angularAxis[s4 + 0] = -q[0];
	info->m_J2angularAxis[s4 + 1] = -q[1];
	info->m_J2angularAxis[s4 + 2] = -q[2];
    // compute the right hand side of the constraint equation. set relative
    // body velocities along p and q to bring the hinge back into alignment.
    // if ax1,ax2 are the unit length hinge axes as computed from body1 and
    // body2, we need to rotate both bodies along the axis u = (ax1 x ax2).
    // if `theta' is the angle between ax1 and ax2, we need an angular velocity
    // along u to cover angle erp*theta in one step :
    //   |angular_velocity| = angle/time = erp*theta / stepsize
    //                      = (erp*fps) * theta
    //    angular_velocity  = |angular_velocity| * (ax1 x ax2) / |ax1 x ax2|
    //                      = (erp*fps) * theta * (ax1 x ax2) / sin(theta)
    // ...as ax1 and ax2 are unit length. if theta is smallish,
    // theta ~= sin(theta), so
    //    angular_velocity  = (erp*fps) * (ax1 x ax2)
    // ax1 x ax2 is in the plane space of ax1, so we project the angular
    // velocity to p and q to find the right hand side.
    btlVector3 ax2 = trB.getBasis().getColumn(2);
	btlVector3 u = ax1.cross(ax2);
	info->m_constraintError[s3] = k * u.dot(p);
	info->m_constraintError[s4] = k * u.dot(q);
	// check angular limits
	int nrow = 4; // last filled row
	int srow;
	btlScalar limit_err = btlScalar(0.0);
	int limit = 0;
	if(getSolveLimit())
	{
#ifdef	_BT_USE_CENTER_LIMIT_
	limit_err = m_limit.getCorrection() * m_referenceSign;
#else
	limit_err = m_correction * m_referenceSign;
#endif
	limit = (limit_err > btlScalar(0.0)) ? 1 : 2;

	}
	// if the hinge has joint limits or motor, add in the extra row
	int powered = 0;
	if(getEnableAngularMotor())
	{
		powered = 1;
	}
	if(limit || powered) 
	{
		nrow++;
		srow = nrow * info->rowskip;
		info->m_J1angularAxis[srow+0] = ax1[0];
		info->m_J1angularAxis[srow+1] = ax1[1];
		info->m_J1angularAxis[srow+2] = ax1[2];

		info->m_J2angularAxis[srow+0] = -ax1[0];
		info->m_J2angularAxis[srow+1] = -ax1[1];
		info->m_J2angularAxis[srow+2] = -ax1[2];

		btlScalar lostop = getLowerLimit();
		btlScalar histop = getUpperLimit();
		if(limit && (lostop == histop))
		{  // the joint motor is ineffective
			powered = 0;
		}
		info->m_constraintError[srow] = btlScalar(0.0f);
		btlScalar currERP = (m_flags & BT_HINGE_FLAGS_ERP_STOP) ? m_stopERP : normalErp;
		if(powered)
		{
			if(m_flags & BT_HINGE_FLAGS_CFM_NORM)
			{
				info->cfm[srow] = m_normalCFM;
			}
			btlScalar mot_fact = getMotorFactor(m_hingeAngle, lostop, histop, m_motorTargetVelocity, info->fps * currERP);
			info->m_constraintError[srow] += mot_fact * m_motorTargetVelocity * m_referenceSign;
			info->m_lowerLimit[srow] = - m_maxMotorImpulse;
			info->m_upperLimit[srow] =   m_maxMotorImpulse;
		}
		if(limit)
		{
			k = info->fps * currERP;
			info->m_constraintError[srow] += k * limit_err;
			if(m_flags & BT_HINGE_FLAGS_CFM_STOP)
			{
				info->cfm[srow] = m_stopCFM;
			}
			if(lostop == histop) 
			{
				// limited low and high simultaneously
				info->m_lowerLimit[srow] = -SIMD_INFINITY;
				info->m_upperLimit[srow] = SIMD_INFINITY;
			}
			else if(limit == 1) 
			{ // low limit
				info->m_lowerLimit[srow] = 0;
				info->m_upperLimit[srow] = SIMD_INFINITY;
			}
			else 
			{ // high limit
				info->m_lowerLimit[srow] = -SIMD_INFINITY;
				info->m_upperLimit[srow] = 0;
			}
			// bounce (we'll use slider parameter abs(1.0 - m_dampingLimAng) for that)
#ifdef	_BT_USE_CENTER_LIMIT_
			btlScalar bounce = m_limit.getRelaxationFactor();
#else
			btlScalar bounce = m_relaxationFactor;
#endif
			if(bounce > btlScalar(0.0))
			{
				btlScalar vel = angVelA.dot(ax1);
				vel -= angVelB.dot(ax1);
				// only apply bounce if the velocity is incoming, and if the
				// resulting c[] exceeds what we already have.
				if(limit == 1)
				{	// low limit
					if(vel < 0)
					{
						btlScalar newc = -bounce * vel;
						if(newc > info->m_constraintError[srow])
						{
							info->m_constraintError[srow] = newc;
						}
					}
				}
				else
				{	// high limit - all those computations are reversed
					if(vel > 0)
					{
						btlScalar newc = -bounce * vel;
						if(newc < info->m_constraintError[srow])
						{
							info->m_constraintError[srow] = newc;
						}
					}
				}
			}
#ifdef	_BT_USE_CENTER_LIMIT_
			info->m_constraintError[srow] *= m_limit.getBiasFactor();
#else
			info->m_constraintError[srow] *= m_biasFactor;
#endif
		} // if(limit)
	} // if angular limit or powered
}


void btlHingeConstraint::setFrames(const btlTransform & frameA, const btlTransform & frameB)
{
	m_rbAFrame = frameA;
	m_rbBFrame = frameB;
	buildJacobian();
}


void	btlHingeConstraint::updateRHS(btlScalar	timeStep)
{
	(void)timeStep;

}




btlScalar btlHingeConstraint::getHingeAngle()
{
	return getHingeAngle(m_rbA.getCenterOfMassTransform(),m_rbB.getCenterOfMassTransform());
}

btlScalar btlHingeConstraint::getHingeAngle(const btlTransform& transA,const btlTransform& transB)
{
	const btlVector3 refAxis0  = transA.getBasis() * m_rbAFrame.getBasis().getColumn(0);
	const btlVector3 refAxis1  = transA.getBasis() * m_rbAFrame.getBasis().getColumn(1);
	const btlVector3 swingAxis = transB.getBasis() * m_rbBFrame.getBasis().getColumn(1);
//	btScalar angle = btAtan2Fast(swingAxis.dot(refAxis0), swingAxis.dot(refAxis1));
	btlScalar angle = btlAtan2(swingAxis.dot(refAxis0), swingAxis.dot(refAxis1));
	return m_referenceSign * angle;
}



void btlHingeConstraint::testLimit(const btlTransform& transA,const btlTransform& transB)
{
	// Compute limit information
	m_hingeAngle = getHingeAngle(transA,transB);
#ifdef	_BT_USE_CENTER_LIMIT_
	m_limit.test(m_hingeAngle);
#else
	m_correction = btlScalar(0.);
	m_limitSign = btlScalar(0.);
	m_solveLimit = false;
	if (m_lowerLimit <= m_upperLimit)
	{
		m_hingeAngle = btlAdjustAngleToLimits(m_hingeAngle, m_lowerLimit, m_upperLimit);
		if (m_hingeAngle <= m_lowerLimit)
		{
			m_correction = (m_lowerLimit - m_hingeAngle);
			m_limitSign = 1.0f;
			m_solveLimit = true;
		} 
		else if (m_hingeAngle >= m_upperLimit)
		{
			m_correction = m_upperLimit - m_hingeAngle;
			m_limitSign = -1.0f;
			m_solveLimit = true;
		}
	}
#endif
	return;
}


static btlVector3 vHinge(0, 0, btlScalar(1));

void btlHingeConstraint::setMotorTarget(const btlQuaternion& qAinB, btlScalar dt)
{
	// convert target from body to constraint space
	btlQuaternion qConstraint = m_rbBFrame.getRotation().inverse() * qAinB * m_rbAFrame.getRotation();
	qConstraint.normalize();

	// extract "pure" hinge component
	btlVector3 vNoHinge = btl_quatRotate(qConstraint, vHinge); vNoHinge.normalize();
	btlQuaternion qNoHinge = shortestArcQuat(vHinge, vNoHinge);
	btlQuaternion qHinge = qNoHinge.inverse() * qConstraint;
	qHinge.normalize();

	// compute angular target, clamped to limits
	btlScalar targetAngle = qHinge.getAngle();
	if (targetAngle > SIMD_PI) // long way around. flip quat and recalculate.
	{
		qHinge = -(qHinge);
		targetAngle = qHinge.getAngle();
	}
	if (qHinge.getZ() < 0)
		targetAngle = -targetAngle;

	setMotorTarget(targetAngle, dt);
}

void btlHingeConstraint::setMotorTarget(btlScalar targetAngle, btlScalar dt)
{
#ifdef	_BT_USE_CENTER_LIMIT_
	m_limit.fit(targetAngle);
#else
	if (m_lowerLimit < m_upperLimit)
	{
		if (targetAngle < m_lowerLimit)
			targetAngle = m_lowerLimit;
		else if (targetAngle > m_upperLimit)
			targetAngle = m_upperLimit;
	}
#endif
	// compute angular velocity
	btlScalar curAngle  = getHingeAngle(m_rbA.getCenterOfMassTransform(),m_rbB.getCenterOfMassTransform());
	btlScalar dAngle = targetAngle - curAngle;
	m_motorTargetVelocity = dAngle / dt;
}



void btlHingeConstraint::getInfo2InternalUsingFrameOffset(btlConstraintInfo2* info, const btlTransform& transA,const btlTransform& transB,const btlVector3& angVelA,const btlVector3& angVelB)
{
	btlAssert(!m_useSolveConstraintObsolete);
	int i, s = info->rowskip;
	// transforms in world space
	btlTransform trA = transA*m_rbAFrame;
	btlTransform trB = transB*m_rbBFrame;
	// pivot point
//	btVector3 pivotAInW = trA.getOrigin();
//	btVector3 pivotBInW = trB.getOrigin();
#if 1
	// difference between frames in WCS
	btlVector3 ofs = trB.getOrigin() - trA.getOrigin();
	// now get weight factors depending on masses
	btlScalar miA = getRigidBodyA().getInvMass();
	btlScalar miB = getRigidBodyB().getInvMass();
	bool hasStaticBody = (miA < SIMD_EPSILON) || (miB < SIMD_EPSILON);
	btlScalar miS = miA + miB;
	btlScalar factA, factB;
	if(miS > btlScalar(0.f))
	{
		factA = miB / miS;
	}
	else 
	{
		factA = btlScalar(0.5f);
	}
	factB = btlScalar(1.0f) - factA;
	// get the desired direction of hinge axis
	// as weighted sum of Z-orthos of frameA and frameB in WCS
	btlVector3 ax1A = trA.getBasis().getColumn(2);
	btlVector3 ax1B = trB.getBasis().getColumn(2);
	btlVector3 ax1 = ax1A * factA + ax1B * factB;
	ax1.normalize();
	// fill first 3 rows 
	// we want: velA + wA x relA == velB + wB x relB
	btlTransform bodyA_trans = transA;
	btlTransform bodyB_trans = transB;
	int s0 = 0;
	int s1 = s;
	int s2 = s * 2;
	int nrow = 2; // last filled row
	btlVector3 tmpA, tmpB, relA, relB, p, q;
	// get vector from bodyB to frameB in WCS
	relB = trB.getOrigin() - bodyB_trans.getOrigin();
	// get its projection to hinge axis
	btlVector3 projB = ax1 * relB.dot(ax1);
	// get vector directed from bodyB to hinge axis (and orthogonal to it)
	btlVector3 orthoB = relB - projB;
	// same for bodyA
	relA = trA.getOrigin() - bodyA_trans.getOrigin();
	btlVector3 projA = ax1 * relA.dot(ax1);
	btlVector3 orthoA = relA - projA;
	btlVector3 totalDist = projA - projB;
	// get offset vectors relA and relB
	relA = orthoA + totalDist * factA;
	relB = orthoB - totalDist * factB;
	// now choose average ortho to hinge axis
	p = orthoB * factA + orthoA * factB;
	btlScalar len2 = p.length2();
	if(len2 > SIMD_EPSILON)
	{
		p /= btlSqrt(len2);
	}
	else
	{
		p = trA.getBasis().getColumn(1);
	}
	// make one more ortho
	q = ax1.cross(p);
	// fill three rows
	tmpA = relA.cross(p);
	tmpB = relB.cross(p);
    for (i=0; i<3; i++) info->m_J1angularAxis[s0+i] = tmpA[i];
    for (i=0; i<3; i++) info->m_J2angularAxis[s0+i] = -tmpB[i];
	tmpA = relA.cross(q);
	tmpB = relB.cross(q);
	if(hasStaticBody && getSolveLimit())
	{ // to make constraint between static and dynamic objects more rigid
		// remove wA (or wB) from equation if angular limit is hit
		tmpB *= factB;
		tmpA *= factA;
	}
	for (i=0; i<3; i++) info->m_J1angularAxis[s1+i] = tmpA[i];
    for (i=0; i<3; i++) info->m_J2angularAxis[s1+i] = -tmpB[i];
	tmpA = relA.cross(ax1);
	tmpB = relB.cross(ax1);
	if(hasStaticBody)
	{ // to make constraint between static and dynamic objects more rigid
		// remove wA (or wB) from equation
		tmpB *= factB;
		tmpA *= factA;
	}
	for (i=0; i<3; i++) info->m_J1angularAxis[s2+i] = tmpA[i];
    for (i=0; i<3; i++) info->m_J2angularAxis[s2+i] = -tmpB[i];

	btlScalar normalErp = (m_flags & BT_HINGE_FLAGS_ERP_NORM)? m_normalERP : info->erp;
	btlScalar k = info->fps * normalErp;

	if (!m_angularOnly)
	{
		for (i=0; i<3; i++) info->m_J1linearAxis[s0+i] = p[i];
		for (i=0; i<3; i++) info->m_J1linearAxis[s1+i] = q[i];
		for (i=0; i<3; i++) info->m_J1linearAxis[s2+i] = ax1[i];

		for (i=0; i<3; i++) info->m_J2linearAxis[s0+i] = -p[i];
		for (i=0; i<3; i++) info->m_J2linearAxis[s1+i] = -q[i];
		for (i=0; i<3; i++) info->m_J2linearAxis[s2+i] = -ax1[i];

	// compute three elements of right hand side
	
		btlScalar rhs = k * p.dot(ofs);
		info->m_constraintError[s0] = rhs;
		rhs = k * q.dot(ofs);
		info->m_constraintError[s1] = rhs;
		rhs = k * ax1.dot(ofs);
		info->m_constraintError[s2] = rhs;
	}
	// the hinge axis should be the only unconstrained
	// rotational axis, the angular velocity of the two bodies perpendicular to
	// the hinge axis should be equal. thus the constraint equations are
	//    p*w1 - p*w2 = 0
	//    q*w1 - q*w2 = 0
	// where p and q are unit vectors normal to the hinge axis, and w1 and w2
	// are the angular velocity vectors of the two bodies.
	int s3 = 3 * s;
	int s4 = 4 * s;
	info->m_J1angularAxis[s3 + 0] = p[0];
	info->m_J1angularAxis[s3 + 1] = p[1];
	info->m_J1angularAxis[s3 + 2] = p[2];
	info->m_J1angularAxis[s4 + 0] = q[0];
	info->m_J1angularAxis[s4 + 1] = q[1];
	info->m_J1angularAxis[s4 + 2] = q[2];

	info->m_J2angularAxis[s3 + 0] = -p[0];
	info->m_J2angularAxis[s3 + 1] = -p[1];
	info->m_J2angularAxis[s3 + 2] = -p[2];
	info->m_J2angularAxis[s4 + 0] = -q[0];
	info->m_J2angularAxis[s4 + 1] = -q[1];
	info->m_J2angularAxis[s4 + 2] = -q[2];
	// compute the right hand side of the constraint equation. set relative
	// body velocities along p and q to bring the hinge back into alignment.
	// if ax1A,ax1B are the unit length hinge axes as computed from bodyA and
	// bodyB, we need to rotate both bodies along the axis u = (ax1 x ax2).
	// if "theta" is the angle between ax1 and ax2, we need an angular velocity
	// along u to cover angle erp*theta in one step :
	//   |angular_velocity| = angle/time = erp*theta / stepsize
	//                      = (erp*fps) * theta
	//    angular_velocity  = |angular_velocity| * (ax1 x ax2) / |ax1 x ax2|
	//                      = (erp*fps) * theta * (ax1 x ax2) / sin(theta)
	// ...as ax1 and ax2 are unit length. if theta is smallish,
	// theta ~= sin(theta), so
	//    angular_velocity  = (erp*fps) * (ax1 x ax2)
	// ax1 x ax2 is in the plane space of ax1, so we project the angular
	// velocity to p and q to find the right hand side.
	k = info->fps * normalErp;//??

	btlVector3 u = ax1A.cross(ax1B);
	info->m_constraintError[s3] = k * u.dot(p);
	info->m_constraintError[s4] = k * u.dot(q);
#endif
	// check angular limits
	nrow = 4; // last filled row
	int srow;
	btlScalar limit_err = btlScalar(0.0);
	int limit = 0;
	if(getSolveLimit())
	{
#ifdef	_BT_USE_CENTER_LIMIT_
	limit_err = m_limit.getCorrection() * m_referenceSign;
#else
	limit_err = m_correction * m_referenceSign;
#endif
	limit = (limit_err > btlScalar(0.0)) ? 1 : 2;

	}
	// if the hinge has joint limits or motor, add in the extra row
	int powered = 0;
	if(getEnableAngularMotor())
	{
		powered = 1;
	}
	if(limit || powered) 
	{
		nrow++;
		srow = nrow * info->rowskip;
		info->m_J1angularAxis[srow+0] = ax1[0];
		info->m_J1angularAxis[srow+1] = ax1[1];
		info->m_J1angularAxis[srow+2] = ax1[2];

		info->m_J2angularAxis[srow+0] = -ax1[0];
		info->m_J2angularAxis[srow+1] = -ax1[1];
		info->m_J2angularAxis[srow+2] = -ax1[2];

		btlScalar lostop = getLowerLimit();
		btlScalar histop = getUpperLimit();
		if(limit && (lostop == histop))
		{  // the joint motor is ineffective
			powered = 0;
		}
		info->m_constraintError[srow] = btlScalar(0.0f);
		btlScalar currERP = (m_flags & BT_HINGE_FLAGS_ERP_STOP) ? m_stopERP : normalErp;
		if(powered)
		{
			if(m_flags & BT_HINGE_FLAGS_CFM_NORM)
			{
				info->cfm[srow] = m_normalCFM;
			}
			btlScalar mot_fact = getMotorFactor(m_hingeAngle, lostop, histop, m_motorTargetVelocity, info->fps * currERP);
			info->m_constraintError[srow] += mot_fact * m_motorTargetVelocity * m_referenceSign;
			info->m_lowerLimit[srow] = - m_maxMotorImpulse;
			info->m_upperLimit[srow] =   m_maxMotorImpulse;
		}
		if(limit)
		{
			k = info->fps * currERP;
			info->m_constraintError[srow] += k * limit_err;
			if(m_flags & BT_HINGE_FLAGS_CFM_STOP)
			{
				info->cfm[srow] = m_stopCFM;
			}
			if(lostop == histop) 
			{
				// limited low and high simultaneously
				info->m_lowerLimit[srow] = -SIMD_INFINITY;
				info->m_upperLimit[srow] = SIMD_INFINITY;
			}
			else if(limit == 1) 
			{ // low limit
				info->m_lowerLimit[srow] = 0;
				info->m_upperLimit[srow] = SIMD_INFINITY;
			}
			else 
			{ // high limit
				info->m_lowerLimit[srow] = -SIMD_INFINITY;
				info->m_upperLimit[srow] = 0;
			}
			// bounce (we'll use slider parameter abs(1.0 - m_dampingLimAng) for that)
#ifdef	_BT_USE_CENTER_LIMIT_
			btlScalar bounce = m_limit.getRelaxationFactor();
#else
			btlScalar bounce = m_relaxationFactor;
#endif
			if(bounce > btlScalar(0.0))
			{
				btlScalar vel = angVelA.dot(ax1);
				vel -= angVelB.dot(ax1);
				// only apply bounce if the velocity is incoming, and if the
				// resulting c[] exceeds what we already have.
				if(limit == 1)
				{	// low limit
					if(vel < 0)
					{
						btlScalar newc = -bounce * vel;
						if(newc > info->m_constraintError[srow])
						{
							info->m_constraintError[srow] = newc;
						}
					}
				}
				else
				{	// high limit - all those computations are reversed
					if(vel > 0)
					{
						btlScalar newc = -bounce * vel;
						if(newc < info->m_constraintError[srow])
						{
							info->m_constraintError[srow] = newc;
						}
					}
				}
			}
#ifdef	_BT_USE_CENTER_LIMIT_
			info->m_constraintError[srow] *= m_limit.getBiasFactor();
#else
			info->m_constraintError[srow] *= m_biasFactor;
#endif
		} // if(limit)
	} // if angular limit or powered
}


///override the default global value of a parameter (such as ERP or CFM), optionally provide the axis (0..5). 
///If no axis is provided, it uses the default axis for this constraint.
void btlHingeConstraint::setParam(int num, btlScalar value, int axis)
{
	if((axis == -1) || (axis == 5))
	{
		switch(num)
		{	
			case BT_CONSTRAINT_STOP_ERP :
				m_stopERP = value;
				m_flags |= BT_HINGE_FLAGS_ERP_STOP;
				break;
			case BT_CONSTRAINT_STOP_CFM :
				m_stopCFM = value;
				m_flags |= BT_HINGE_FLAGS_CFM_STOP;
				break;
			case BT_CONSTRAINT_CFM :
				m_normalCFM = value;
				m_flags |= BT_HINGE_FLAGS_CFM_NORM;
				break;
			case BT_CONSTRAINT_ERP:
				m_normalERP = value;
				m_flags |= BT_HINGE_FLAGS_ERP_NORM;
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
btlScalar btlHingeConstraint::getParam(int num, int axis) const 
{
	btlScalar retVal = 0;
	if((axis == -1) || (axis == 5))
	{
		switch(num)
		{	
			case BT_CONSTRAINT_STOP_ERP :
				btlAssertConstrParams(m_flags & BT_HINGE_FLAGS_ERP_STOP);
				retVal = m_stopERP;
				break;
			case BT_CONSTRAINT_STOP_CFM :
				btlAssertConstrParams(m_flags & BT_HINGE_FLAGS_CFM_STOP);
				retVal = m_stopCFM;
				break;
			case BT_CONSTRAINT_CFM :
				btlAssertConstrParams(m_flags & BT_HINGE_FLAGS_CFM_NORM);
				retVal = m_normalCFM;
				break;
			case BT_CONSTRAINT_ERP:
				btlAssertConstrParams(m_flags & BT_HINGE_FLAGS_ERP_NORM);
				retVal = m_normalERP;
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


