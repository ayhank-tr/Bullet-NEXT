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


#include "btTypedConstraint.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "LinearMath/btSerializer.h"


#define DEFAULT_DEBUGDRAW_SIZE btlScalar(0.3f)

btlTypedConstraint::btlTypedConstraint(btlTypedConstraintType type, btlRigidBody& rbA)
:btlTypedObject(type),
m_userConstraintType(-1),
m_userConstraintPtr((void*)-1),
m_breakingImpulseThreshold(SIMD_INFINITY),
m_isEnabled(true),
m_needsFeedback(false),
m_overrideNumSolverIterations(-1),
m_rbA(rbA),
m_rbB(getFixedBody()),
m_appliedImpulse(btlScalar(0.)),
m_dbgDrawSize(DEFAULT_DEBUGDRAW_SIZE),
m_jointFeedback(0)
{
}


btlTypedConstraint::btlTypedConstraint(btlTypedConstraintType type, btlRigidBody& rbA,btlRigidBody& rbB)
:btlTypedObject(type),
m_userConstraintType(-1),
m_userConstraintPtr((void*)-1),
m_breakingImpulseThreshold(SIMD_INFINITY),
m_isEnabled(true),
m_needsFeedback(false),
m_overrideNumSolverIterations(-1),
m_rbA(rbA),
m_rbB(rbB),
m_appliedImpulse(btlScalar(0.)),
m_dbgDrawSize(DEFAULT_DEBUGDRAW_SIZE),
m_jointFeedback(0)
{
}




btlScalar btlTypedConstraint::getMotorFactor(btlScalar pos, btlScalar lowLim, btlScalar uppLim, btlScalar vel, btlScalar timeFact)
{
	if(lowLim > uppLim)
	{
		return btlScalar(1.0f);
	}
	else if(lowLim == uppLim)
	{
		return btlScalar(0.0f);
	}
	btlScalar lim_fact = btlScalar(1.0f);
	btlScalar delta_max = vel / timeFact;
	if(delta_max < btlScalar(0.0f))
	{
		if((pos >= lowLim) && (pos < (lowLim - delta_max)))
		{
			lim_fact = (lowLim - pos) / delta_max;
		}
		else if(pos  < lowLim)
		{
			lim_fact = btlScalar(0.0f);
		}
		else
		{
			lim_fact = btlScalar(1.0f);
		}
	}
	else if(delta_max > btlScalar(0.0f))
	{
		if((pos <= uppLim) && (pos > (uppLim - delta_max)))
		{
			lim_fact = (uppLim - pos) / delta_max;
		}
		else if(pos  > uppLim)
		{
			lim_fact = btlScalar(0.0f);
		}
		else
		{
			lim_fact = btlScalar(1.0f);
		}
	}
	else
	{
			lim_fact = btlScalar(0.0f);
	}
	return lim_fact;
}

///fills the dataBuffer and returns the struct name (and 0 on failure)
const char*	btlTypedConstraint::serialize(void* dataBuffer, btlSerializer* serializer) const
{
	btlTypedConstraintData2* tcd = (btlTypedConstraintData2*) dataBuffer;

	tcd->m_rbA = (btlRigidBodyData*)serializer->getUniquePointer(&m_rbA);
	tcd->m_rbB = (btlRigidBodyData*)serializer->getUniquePointer(&m_rbB);
	char* name = (char*) serializer->findNameForPointer(this);
	tcd->m_name = (char*)serializer->getUniquePointer(name);
	if (tcd->m_name)
	{
		serializer->serializeName(name);
	}

	tcd->m_objectType = m_objectType;
	tcd->m_needsFeedback = m_needsFeedback;
	tcd->m_overrideNumSolverIterations = m_overrideNumSolverIterations;
	tcd->m_breakingImpulseThreshold = m_breakingImpulseThreshold;
	tcd->m_isEnabled = m_isEnabled? 1: 0;
	
	tcd->m_userConstraintId =m_userConstraintId;
	tcd->m_userConstraintType =m_userConstraintType;

	tcd->m_appliedImpulse = m_appliedImpulse;
	tcd->m_dbgDrawSize = m_dbgDrawSize;

	tcd->m_disableCollisionsBetweenLinkedBodies = false;

	int i;
	for (i=0;i<m_rbA.getNumConstraintRefs();i++)
		if (m_rbA.getConstraintRef(i) == this)
			tcd->m_disableCollisionsBetweenLinkedBodies = true;
	for (i=0;i<m_rbB.getNumConstraintRefs();i++)
		if (m_rbB.getConstraintRef(i) == this)
			tcd->m_disableCollisionsBetweenLinkedBodies = true;

	return btlTypedConstraintDataName;
}

btlRigidBody& btlTypedConstraint::getFixedBody()
{
	static btlRigidBody s_fixed(0, 0,0);
	s_fixed.setMassProps(btlScalar(0.),btlVector3(btlScalar(0.),btlScalar(0.),btlScalar(0.)));
	return s_fixed;
}


void btlAngularLimit::set(btlScalar low, btlScalar high, btlScalar _softness, btlScalar _biasFactor, btlScalar _relaxationFactor)
{
	m_halfRange = (high - low) / 2.0f;
	m_center = btlNormalizeAngle(low + m_halfRange);
	m_softness =  _softness;
	m_biasFactor = _biasFactor;
	m_relaxationFactor = _relaxationFactor;
}

void btlAngularLimit::test(const btlScalar angle)
{
	m_correction = 0.0f;
	m_sign = 0.0f;
	m_solveLimit = false;

	if (m_halfRange >= 0.0f)
	{
		btlScalar deviation = btlNormalizeAngle(angle - m_center);
		if (deviation < -m_halfRange)
		{
			m_solveLimit = true;
			m_correction = - (deviation + m_halfRange);
			m_sign = +1.0f;
		}
		else if (deviation > m_halfRange)
		{
			m_solveLimit = true;
			m_correction = m_halfRange - deviation;
			m_sign = -1.0f;
		}
	}
}


btlScalar btlAngularLimit::getError() const
{
	return m_correction * m_sign;
}

void btlAngularLimit::fit(btlScalar& angle) const
{
	if (m_halfRange > 0.0f)
	{
		btlScalar relativeAngle = btlNormalizeAngle(angle - m_center);
		if (!btlEqual(relativeAngle, m_halfRange))
		{
			if (relativeAngle > 0.0f)
			{
				angle = getHigh();
			}
			else
			{
				angle = getLow();
			}
		}
	}
}

btlScalar btlAngularLimit::getLow() const
{
	return btlNormalizeAngle(m_center - m_halfRange);
}

btlScalar btlAngularLimit::getHigh() const
{
	return btlNormalizeAngle(m_center + m_halfRange);
}
