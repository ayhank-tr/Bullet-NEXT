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

#ifndef BT_MANIFOLD_CONTACT_POINT_H
#define BT_MANIFOLD_CONTACT_POINT_H

#include "LinearMath/btVector3.h"
#include "LinearMath/btTransformUtil.h"

#ifdef PFX_USE_FREE_VECTORMATH
	#include "physics_effects/base_level/solver/pfx_constraint_row.h"
typedef sce::PhysicsEffects::PfxConstraintRow btlConstraintRow;
#else
	// Don't change following order of parameters
	ATTRIBUTE_ALIGNED16(struct) btlConstraintRow {
		btlScalar m_normal[3];
		btlScalar m_rhs;
		btlScalar m_jacDiagInv;
		btlScalar m_lowerLimit;
		btlScalar m_upperLimit;
		btlScalar m_accumImpulse;
	};
	typedef btlConstraintRow PfxConstraintRow;
#endif //PFX_USE_FREE_VECTORMATH



/// ManifoldContactPoint collects and maintains persistent contactpoints.
/// used to improve stability and performance of rigidbody dynamics response.
class btlManifoldPoint
	{
		public:
			btlManifoldPoint()
				:m_userPersistentData(0),
				m_lateralFrictionInitialized(false),
                m_appliedImpulse(0.f),
                m_appliedImpulseLateral1(0.f),
				m_appliedImpulseLateral2(0.f),
				m_contactMotion1(0.f),
				m_contactMotion2(0.f),
				m_contactCFM1(0.f),
				m_contactCFM2(0.f),
				m_lifeTime(0)
			{
			}

			btlManifoldPoint( const btlVector3 &pointA, const btlVector3 &pointB, 
					const btlVector3 &normal, 
					btlScalar distance ) :
					m_localPointA( pointA ), 
					m_localPointB( pointB ), 
					m_normalWorldOnB( normal ), 
					m_distance1( distance ),
					m_combinedFriction(btlScalar(0.)),
					m_combinedRollingFriction(btlScalar(0.)),
					m_combinedRestitution(btlScalar(0.)),
					m_userPersistentData(0),
					m_lateralFrictionInitialized(false),
                    m_appliedImpulse(0.f),
                    m_appliedImpulseLateral1(0.f),
					m_appliedImpulseLateral2(0.f),
					m_contactMotion1(0.f),
					m_contactMotion2(0.f),
					m_contactCFM1(0.f),
					m_contactCFM2(0.f),
					m_lifeTime(0)
			{
				
			}

			

			btlVector3 m_localPointA;			
			btlVector3 m_localPointB;			
			btlVector3	m_positionWorldOnB;
			///m_positionWorldOnA is redundant information, see getPositionWorldOnA(), but for clarity
			btlVector3	m_positionWorldOnA;
			btlVector3 m_normalWorldOnB;
		
			btlScalar	m_distance1;
			btlScalar	m_combinedFriction;
			btlScalar	m_combinedRollingFriction;
			btlScalar	m_combinedRestitution;

			//BP mod, store contact triangles.
			int			m_partId0;
			int			m_partId1;
			int			m_index0;
			int			m_index1;
				
			mutable void*	m_userPersistentData;
			bool			m_lateralFrictionInitialized;

			btlScalar		m_appliedImpulse;
			btlScalar		m_appliedImpulseLateral1;
			btlScalar		m_appliedImpulseLateral2;
			btlScalar		m_contactMotion1;
			btlScalar		m_contactMotion2;
			btlScalar		m_contactCFM1;
			btlScalar		m_contactCFM2;

			int				m_lifeTime;//lifetime of the contactpoint in frames
			
			btlVector3		m_lateralFrictionDir1;
			btlVector3		m_lateralFrictionDir2;




			btlScalar getDistance() const
			{
				return m_distance1;
			}
			int	getLifeTime() const
			{
				return m_lifeTime;
			}

			const btlVector3& getPositionWorldOnA() const {
				return m_positionWorldOnA;
//				return m_positionWorldOnB + m_normalWorldOnB * m_distance1;
			}

			const btlVector3& getPositionWorldOnB() const
			{
				return m_positionWorldOnB;
			}

			void	setDistance(btlScalar dist)
			{
				m_distance1 = dist;
			}
			
			///this returns the most recent applied impulse, to satisfy contact constraints by the constraint solver
			btlScalar	getAppliedImpulse() const
			{
				return m_appliedImpulse;
			}

			

	};

#endif //BT_MANIFOLD_CONTACT_POINT_H
