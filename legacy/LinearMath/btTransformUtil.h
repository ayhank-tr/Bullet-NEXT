/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#ifndef BT_TRANSFORM_UTIL_H
#define BT_TRANSFORM_UTIL_H

#include "btTransform.h"
#define ANGULAR_MOTION_THRESHOLD btlScalar(0.5)*SIMD_HALF_PI




SIMD_FORCE_INLINE btlVector3 btlAabbSupport(const btlVector3& halfExtents,const btlVector3& supportDir)
{
	return btlVector3(supportDir.x() < btlScalar(0.0) ? -halfExtents.x() : halfExtents.x(),
      supportDir.y() < btlScalar(0.0) ? -halfExtents.y() : halfExtents.y(),
      supportDir.z() < btlScalar(0.0) ? -halfExtents.z() : halfExtents.z()); 
}






/// Utils related to temporal transforms
class btlTransformUtil
{

public:

	static void integrateTransform(const btlTransform& curTrans,const btlVector3& linvel,const btlVector3& angvel,btlScalar timeStep,btlTransform& predictedTransform)
	{
		predictedTransform.setOrigin(curTrans.getOrigin() + linvel * timeStep);
//	#define QUATERNION_DERIVATIVE
	#ifdef QUATERNION_DERIVATIVE
		btlQuaternion predictedOrn = curTrans.getRotation();
		predictedOrn += (angvel * predictedOrn) * (timeStep * btlScalar(0.5));
		predictedOrn.normalize();
	#else
		//Exponential map
		//google for "Practical Parameterization of Rotations Using the Exponential Map", F. Sebastian Grassia

		btlVector3 axis;
		btlScalar	fAngle = angvel.length(); 
		//limit the angular motion
		if (fAngle*timeStep > ANGULAR_MOTION_THRESHOLD)
		{
			fAngle = ANGULAR_MOTION_THRESHOLD / timeStep;
		}

		if ( fAngle < btlScalar(0.001) )
		{
			// use Taylor's expansions of sync function
			axis   = angvel*( btlScalar(0.5)*timeStep-(timeStep*timeStep*timeStep)*(btlScalar(0.020833333333))*fAngle*fAngle );
		}
		else
		{
			// sync(fAngle) = sin(c*fAngle)/t
			axis   = angvel*( btlSin(btlScalar(0.5)*fAngle*timeStep)/fAngle );
		}
		btlQuaternion dorn (axis.x(),axis.y(),axis.z(),btlCos( fAngle*timeStep*btlScalar(0.5) ));
		btlQuaternion orn0 = curTrans.getRotation();

		btlQuaternion predictedOrn = dorn * orn0;
		predictedOrn.normalize();
	#endif
		predictedTransform.setRotation(predictedOrn);
	}

	static void	calculateVelocityQuaternion(const btlVector3& pos0,const btlVector3& pos1,const btlQuaternion& orn0,const btlQuaternion& orn1,btlScalar timeStep,btlVector3& linVel,btlVector3& angVel)
	{
		linVel = (pos1 - pos0) / timeStep;
		btlVector3 axis;
		btlScalar  angle;
		if (orn0 != orn1)
		{
			calculateDiffAxisAngleQuaternion(orn0,orn1,axis,angle);
			angVel = axis * angle / timeStep;
		} else
		{
			angVel.setValue(0,0,0);
		}
	}

	static void calculateDiffAxisAngleQuaternion(const btlQuaternion& orn0,const btlQuaternion& orn1a,btlVector3& axis,btlScalar& angle)
	{
		btlQuaternion orn1 = orn0.nearest(orn1a);
		btlQuaternion dorn = orn1 * orn0.inverse();
		angle = dorn.getAngle();
		axis = btlVector3(dorn.x(),dorn.y(),dorn.z());
		axis[3] = btlScalar(0.);
		//check for axis length
		btlScalar len = axis.length2();
		if (len < SIMD_EPSILON*SIMD_EPSILON)
			axis = btlVector3(btlScalar(1.),btlScalar(0.),btlScalar(0.));
		else
			axis /= btlSqrt(len);
	}

	static void	calculateVelocity(const btlTransform& transform0,const btlTransform& transform1,btlScalar timeStep,btlVector3& linVel,btlVector3& angVel)
	{
		linVel = (transform1.getOrigin() - transform0.getOrigin()) / timeStep;
		btlVector3 axis;
		btlScalar  angle;
		calculateDiffAxisAngle(transform0,transform1,axis,angle);
		angVel = axis * angle / timeStep;
	}

	static void calculateDiffAxisAngle(const btlTransform& transform0,const btlTransform& transform1,btlVector3& axis,btlScalar& angle)
	{
		btlMatrix3x3 dmat = transform1.getBasis() * transform0.getBasis().inverse();
		btlQuaternion dorn;
		dmat.getRotation(dorn);

		///floating point inaccuracy can lead to w component > 1..., which breaks 
		dorn.normalize();
		
		angle = dorn.getAngle();
		axis = btlVector3(dorn.x(),dorn.y(),dorn.z());
		axis[3] = btlScalar(0.);
		//check for axis length
		btlScalar len = axis.length2();
		if (len < SIMD_EPSILON*SIMD_EPSILON)
			axis = btlVector3(btlScalar(1.),btlScalar(0.),btlScalar(0.));
		else
			axis /= btlSqrt(len);
	}

};


///The btConvexSeparatingDistanceUtil can help speed up convex collision detection 
///by conservatively updating a cached separating distance/vector instead of re-calculating the closest distance
class	btlConvexSeparatingDistanceUtil
{
	btlQuaternion	m_ornA;
	btlQuaternion	m_ornB;
	btlVector3	m_posA;
	btlVector3	m_posB;
	
	btlVector3	m_separatingNormal;

	btlScalar	m_boundingRadiusA;
	btlScalar	m_boundingRadiusB;
	btlScalar	m_separatingDistance;

public:

	btlConvexSeparatingDistanceUtil(btlScalar	boundingRadiusA,btlScalar	boundingRadiusB)
		:m_boundingRadiusA(boundingRadiusA),
		m_boundingRadiusB(boundingRadiusB),
		m_separatingDistance(0.f)
	{
	}

	btlScalar	getConservativeSeparatingDistance()
	{
		return m_separatingDistance;
	}

	void	updateSeparatingDistance(const btlTransform& transA,const btlTransform& transB)
	{
		const btlVector3& toPosA = transA.getOrigin();
		const btlVector3& toPosB = transB.getOrigin();
		btlQuaternion toOrnA = transA.getRotation();
		btlQuaternion toOrnB = transB.getRotation();

		if (m_separatingDistance>0.f)
		{
			

			btlVector3 linVelA,angVelA,linVelB,angVelB;
			btlTransformUtil::calculateVelocityQuaternion(m_posA,toPosA,m_ornA,toOrnA,btlScalar(1.),linVelA,angVelA);
			btlTransformUtil::calculateVelocityQuaternion(m_posB,toPosB,m_ornB,toOrnB,btlScalar(1.),linVelB,angVelB);
			btlScalar maxAngularProjectedVelocity = angVelA.length() * m_boundingRadiusA + angVelB.length() * m_boundingRadiusB;
			btlVector3 relLinVel = (linVelB-linVelA);
			btlScalar relLinVelocLength = relLinVel.dot(m_separatingNormal);
			if (relLinVelocLength<0.f)
			{
				relLinVelocLength = 0.f;
			}
	
			btlScalar	projectedMotion = maxAngularProjectedVelocity +relLinVelocLength;
			m_separatingDistance -= projectedMotion;
		}
	
		m_posA = toPosA;
		m_posB = toPosB;
		m_ornA = toOrnA;
		m_ornB = toOrnB;
	}

	void	initSeparatingDistance(const btlVector3& separatingVector,btlScalar separatingDistance,const btlTransform& transA,const btlTransform& transB)
	{
		m_separatingDistance = separatingDistance;

		if (m_separatingDistance>0.f)
		{
			m_separatingNormal = separatingVector;
			
			const btlVector3& toPosA = transA.getOrigin();
			const btlVector3& toPosB = transB.getOrigin();
			btlQuaternion toOrnA = transA.getRotation();
			btlQuaternion toOrnB = transB.getRotation();
			m_posA = toPosA;
			m_posB = toPosB;
			m_ornA = toOrnA;
			m_ornB = toOrnB;
		}
	}

};


#endif //BT_TRANSFORM_UTIL_H

