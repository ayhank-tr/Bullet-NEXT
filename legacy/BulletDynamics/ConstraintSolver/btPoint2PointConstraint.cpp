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


#include "btPoint2PointConstraint.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include <new>





btlPoint2PointConstraint::btlPoint2PointConstraint(btlRigidBody& rbA,btlRigidBody& rbB, const btlVector3& pivotInA,const btlVector3& pivotInB)
:btlTypedConstraint(POINT2POINT_CONSTRAINT_TYPE,rbA,rbB),m_pivotInA(pivotInA),m_pivotInB(pivotInB),
m_flags(0),
m_useSolveConstraintObsolete(false)
{

}


btlPoint2PointConstraint::btlPoint2PointConstraint(btlRigidBody& rbA,const btlVector3& pivotInA)
:btlTypedConstraint(POINT2POINT_CONSTRAINT_TYPE,rbA),m_pivotInA(pivotInA),m_pivotInB(rbA.getCenterOfMassTransform()(pivotInA)),
m_flags(0),
m_useSolveConstraintObsolete(false)
{
	
}

void	btlPoint2PointConstraint::buildJacobian()
{

	///we need it for both methods
	{
		m_appliedImpulse = btlScalar(0.);

		btlVector3	normal(0,0,0);

		for (int i=0;i<3;i++)
		{
			normal[i] = 1;
			new (&m_jac[i]) btlJacobianEntry(
			m_rbA.getCenterOfMassTransform().getBasis().transpose(),
			m_rbB.getCenterOfMassTransform().getBasis().transpose(),
			m_rbA.getCenterOfMassTransform()*m_pivotInA - m_rbA.getCenterOfMassPosition(),
			m_rbB.getCenterOfMassTransform()*m_pivotInB - m_rbB.getCenterOfMassPosition(),
			normal,
			m_rbA.getInvInertiaDiagLocal(),
			m_rbA.getInvMass(),
			m_rbB.getInvInertiaDiagLocal(),
			m_rbB.getInvMass());
		normal[i] = 0;
		}
	}


}

void btlPoint2PointConstraint::getInfo1 (btlConstraintInfo1* info)
{
	getInfo1NonVirtual(info);
}

void btlPoint2PointConstraint::getInfo1NonVirtual (btlConstraintInfo1* info)
{
	if (m_useSolveConstraintObsolete)
	{
		info->m_numConstraintRows = 0;
		info->nub = 0;
	} else
	{
		info->m_numConstraintRows = 3;
		info->nub = 3;
	}
}




void btlPoint2PointConstraint::getInfo2 (btlConstraintInfo2* info)
{
	getInfo2NonVirtual(info, m_rbA.getCenterOfMassTransform(),m_rbB.getCenterOfMassTransform());
}

void btlPoint2PointConstraint::getInfo2NonVirtual (btlConstraintInfo2* info, const btlTransform& body0_trans, const btlTransform& body1_trans)
{
	btlAssert(!m_useSolveConstraintObsolete);

	 //retrieve matrices

	// anchor points in global coordinates with respect to body PORs.
   
    // set jacobian
    info->m_J1linearAxis[0] = 1;
	info->m_J1linearAxis[info->rowskip+1] = 1;
	info->m_J1linearAxis[2*info->rowskip+2] = 1;

	btlVector3 a1 = body0_trans.getBasis()*getPivotInA();
	{
		btlVector3* angular0 = (btlVector3*)(info->m_J1angularAxis);
		btlVector3* angular1 = (btlVector3*)(info->m_J1angularAxis+info->rowskip);
		btlVector3* angular2 = (btlVector3*)(info->m_J1angularAxis+2*info->rowskip);
		btlVector3 a1neg = -a1;
		a1neg.getSkewSymmetricMatrix(angular0,angular1,angular2);
	}
    
	info->m_J2linearAxis[0] = -1;
    info->m_J2linearAxis[info->rowskip+1] = -1;
    info->m_J2linearAxis[2*info->rowskip+2] = -1;
	
	btlVector3 a2 = body1_trans.getBasis()*getPivotInB();
   
	{
	//	btVector3 a2n = -a2;
		btlVector3* angular0 = (btlVector3*)(info->m_J2angularAxis);
		btlVector3* angular1 = (btlVector3*)(info->m_J2angularAxis+info->rowskip);
		btlVector3* angular2 = (btlVector3*)(info->m_J2angularAxis+2*info->rowskip);
		a2.getSkewSymmetricMatrix(angular0,angular1,angular2);
	}
    


    // set right hand side
	btlScalar currERP = (m_flags & BT_P2P_FLAGS_ERP) ? m_erp : info->erp;
    btlScalar k = info->fps * currERP;
    int j;
	for (j=0; j<3; j++)
    {
        info->m_constraintError[j*info->rowskip] = k * (a2[j] + body1_trans.getOrigin()[j] - a1[j] - body0_trans.getOrigin()[j]);
		//printf("info->m_constraintError[%d]=%f\n",j,info->m_constraintError[j]);
    }
	if(m_flags & BT_P2P_FLAGS_CFM)
	{
		for (j=0; j<3; j++)
		{
			info->cfm[j*info->rowskip] = m_cfm;
		}
	}

	btlScalar impulseClamp = m_setting.m_impulseClamp;//
	for (j=0; j<3; j++)
    {
		if (m_setting.m_impulseClamp > 0)
		{
			info->m_lowerLimit[j*info->rowskip] = -impulseClamp;
			info->m_upperLimit[j*info->rowskip] = impulseClamp;
		}
	}
	info->m_damping = m_setting.m_damping;
	
}



void	btlPoint2PointConstraint::updateRHS(btlScalar	timeStep)
{
	(void)timeStep;

}

///override the default global value of a parameter (such as ERP or CFM), optionally provide the axis (0..5). 
///If no axis is provided, it uses the default axis for this constraint.
void btlPoint2PointConstraint::setParam(int num, btlScalar value, int axis)
{
	if(axis != -1)
	{
		btlAssertConstrParams(0);
	}
	else
	{
		switch(num)
		{
			case BT_CONSTRAINT_ERP :
			case BT_CONSTRAINT_STOP_ERP :
				m_erp = value; 
				m_flags |= BT_P2P_FLAGS_ERP;
				break;
			case BT_CONSTRAINT_CFM :
			case BT_CONSTRAINT_STOP_CFM :
				m_cfm = value; 
				m_flags |= BT_P2P_FLAGS_CFM;
				break;
			default: 
				btlAssertConstrParams(0);
		}
	}
}

///return the local value of parameter
btlScalar btlPoint2PointConstraint::getParam(int num, int axis) const 
{
	btlScalar retVal(SIMD_INFINITY);
	if(axis != -1)
	{
		btlAssertConstrParams(0);
	}
	else
	{
		switch(num)
		{
			case BT_CONSTRAINT_ERP :
			case BT_CONSTRAINT_STOP_ERP :
				btlAssertConstrParams(m_flags & BT_P2P_FLAGS_ERP);
				retVal = m_erp; 
				break;
			case BT_CONSTRAINT_CFM :
			case BT_CONSTRAINT_STOP_CFM :
				btlAssertConstrParams(m_flags & BT_P2P_FLAGS_CFM);
				retVal = m_cfm; 
				break;
			default: 
				btlAssertConstrParams(0);
		}
	}
	return retVal;
}
	
