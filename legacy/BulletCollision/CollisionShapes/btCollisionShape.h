/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_COLLISION_SHAPE_H
#define BT_COLLISION_SHAPE_H

#include "LinearMath/btTransform.h"
#include "LinearMath/btVector3.h"
#include "LinearMath/btMatrix3x3.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h" //for the shape types
class btlSerializer;


///The btCollisionShape class provides an interface for collision shapes that can be shared among btCollisionObjects.
ATTRIBUTE_ALIGNED16(class) btlCollisionShape
{
protected:
	int m_shapeType;
	void* m_userPointer;
	int m_userIndex;

public:

	BT_DECLARE_ALIGNED_ALLOCATOR();

	btlCollisionShape() : m_shapeType (INVALID_SHAPE_PROXYTYPE), m_userPointer(0), m_userIndex(-1)
	{
	}

	virtual ~btlCollisionShape()
	{
	}

	///getAabb returns the axis aligned bounding box in the coordinate frame of the given transform t.
	virtual void getAabb(const btlTransform& t,btlVector3& aabbMin,btlVector3& aabbMax) const =0;

	virtual void	getBoundingSphere(btlVector3& center,btlScalar& radius) const;

	///getAngularMotionDisc returns the maximus radius needed for Conservative Advancement to handle time-of-impact with rotations.
	virtual btlScalar	getAngularMotionDisc() const;

	virtual btlScalar	getContactBreakingThreshold(btlScalar defaultContactThresholdFactor) const;


	///calculateTemporalAabb calculates the enclosing aabb for the moving object over interval [0..timeStep)
	///result is conservative
	void calculateTemporalAabb(const btlTransform& curTrans,const btlVector3& linvel,const btlVector3& angvel,btlScalar timeStep, btlVector3& temporalAabbMin,btlVector3& temporalAabbMax) const;



	SIMD_FORCE_INLINE bool	isPolyhedral() const
	{
		return btlBroadphaseProxy::isPolyhedral(getShapeType());
	}

	SIMD_FORCE_INLINE bool	isConvex2d() const
	{
		return btlBroadphaseProxy::isConvex2d(getShapeType());
	}

	SIMD_FORCE_INLINE bool	isConvex() const
	{
		return btlBroadphaseProxy::isConvex(getShapeType());
	}
	SIMD_FORCE_INLINE bool	isNonMoving() const
	{
		return btlBroadphaseProxy::isNonMoving(getShapeType());
	}
	SIMD_FORCE_INLINE bool	isConcave() const
	{
		return btlBroadphaseProxy::isConcave(getShapeType());
	}
	SIMD_FORCE_INLINE bool	isCompound() const
	{
		return btlBroadphaseProxy::isCompound(getShapeType());
	}

	SIMD_FORCE_INLINE bool	isSoftBody() const
	{
		return btlBroadphaseProxy::isSoftBody(getShapeType());
	}

	///isInfinite is used to catch simulation error (aabb check)
	SIMD_FORCE_INLINE bool isInfinite() const
	{
		return btlBroadphaseProxy::isInfinite(getShapeType());
	}

#ifndef __SPU__
	virtual void	setLocalScaling(const btlVector3& scaling) =0;
	virtual const btlVector3& getLocalScaling() const =0;
	virtual void	calculateLocalInertia(btlScalar mass,btlVector3& inertia) const = 0;


//debugging support
	virtual const char*	getName()const =0 ;
#endif //__SPU__

	
	int		getShapeType() const { return m_shapeType; }

	///the getAnisotropicRollingFrictionDirection can be used in combination with setAnisotropicFriction
	///See Bullet/Demos/RollingFrictionDemo for an example
	virtual btlVector3	getAnisotropicRollingFrictionDirection() const
	{
		return btlVector3(1,1,1);
	}
	virtual void	setMargin(btlScalar margin) = 0;
	virtual btlScalar	getMargin() const = 0;

	
	///optional user data pointer
	void	setUserPointer(void*  userPtr)
	{
		m_userPointer = userPtr;
	}

	void*	getUserPointer() const
	{
		return m_userPointer;
	}
	void setUserIndex(int index)
	{
		m_userIndex = index;
	}

	int getUserIndex() const
	{
		return m_userIndex;
	}


	virtual	int	calculateSerializeBufferSize() const;

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual	const char*	serialize(void* dataBuffer, btlSerializer* serializer) const;

	virtual void	serializeSingleShape(btlSerializer* serializer) const;

};	

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct	btlCollisionShapeData
{
	char	*m_name;
	int		m_shapeType;
	char	m_padding[4];
};

SIMD_FORCE_INLINE	int	btlCollisionShape::calculateSerializeBufferSize() const
{
	return sizeof(btlCollisionShapeData);
}



#endif //BT_COLLISION_SHAPE_H

