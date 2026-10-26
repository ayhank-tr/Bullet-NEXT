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

#ifndef BT_CAPSULE_SHAPE_H
#define BT_CAPSULE_SHAPE_H

#include "btConvexInternalShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h" // for the types


///The btCapsuleShape represents a capsule around the Y axis, there is also the btCapsuleShapeX aligned around the X axis and btCapsuleShapeZ around the Z axis.
///The total height is height+2*radius, so the height is just the height between the center of each 'sphere' of the capsule caps.
///The btCapsuleShape is a convex hull of two spheres. The btMultiSphereShape is a more general collision shape that takes the convex hull of multiple sphere, so it can also represent a capsule when just using two spheres.
ATTRIBUTE_ALIGNED16(class) btlCapsuleShape : public btlConvexInternalShape
{
protected:
	int	m_upAxis;

protected:
	///only used for btCapsuleShapeZ and btCapsuleShapeX subclasses.
	btlCapsuleShape() : btlConvexInternalShape() {m_shapeType = CAPSULE_SHAPE_PROXYTYPE;};

public:
	
	BT_DECLARE_ALIGNED_ALLOCATOR();
	
	btlCapsuleShape(btlScalar radius,btlScalar height);

	///CollisionShape Interface
	virtual void	calculateLocalInertia(btlScalar mass,btlVector3& inertia) const;

	/// btConvexShape Interface
	virtual btlVector3	localGetSupportingVertexWithoutMargin(const btlVector3& vec)const;

	virtual void	batchedUnitVectorGetSupportingVertexWithoutMargin(const btlVector3* vectors,btlVector3* supportVerticesOut,int numVectors) const;
	
	virtual void setMargin(btlScalar collisionMargin)
	{
		//correct the m_implicitShapeDimensions for the margin
		btlVector3 oldMargin(getMargin(),getMargin(),getMargin());
		btlVector3 implicitShapeDimensionsWithMargin = m_implicitShapeDimensions+oldMargin;
		
		btlConvexInternalShape::setMargin(collisionMargin);
		btlVector3 newMargin(getMargin(),getMargin(),getMargin());
		m_implicitShapeDimensions = implicitShapeDimensionsWithMargin - newMargin;

	}

	virtual void getAabb (const btlTransform& t, btlVector3& aabbMin, btlVector3& aabbMax) const
	{
			btlVector3 halfExtents(getRadius(),getRadius(),getRadius());
			halfExtents[m_upAxis] = getRadius() + getHalfHeight();
			halfExtents += btlVector3(getMargin(),getMargin(),getMargin());
			btlMatrix3x3 abs_b = t.getBasis().absolute();  
			btlVector3 center = t.getOrigin();
            btlVector3 extent = halfExtents.dot3(abs_b[0], abs_b[1], abs_b[2]);
        
			aabbMin = center - extent;
			aabbMax = center + extent;
	}

	virtual const char*	getName()const 
	{
		return "CapsuleShape";
	}

	int	getUpAxis() const
	{
		return m_upAxis;
	}

	btlScalar	getRadius() const
	{
		int radiusAxis = (m_upAxis+2)%3;
		return m_implicitShapeDimensions[radiusAxis];
	}

	btlScalar	getHalfHeight() const
	{
		return m_implicitShapeDimensions[m_upAxis];
	}

	virtual void	setLocalScaling(const btlVector3& scaling)
	{
		btlVector3 oldMargin(getMargin(),getMargin(),getMargin());
		btlVector3 implicitShapeDimensionsWithMargin = m_implicitShapeDimensions+oldMargin;
		btlVector3 unScaledImplicitShapeDimensionsWithMargin = implicitShapeDimensionsWithMargin / m_localScaling;

		btlConvexInternalShape::setLocalScaling(scaling);

		m_implicitShapeDimensions = (unScaledImplicitShapeDimensionsWithMargin * m_localScaling) - oldMargin;

	}

	virtual btlVector3	getAnisotropicRollingFrictionDirection() const
	{
		btlVector3 aniDir(0,0,0);
		aniDir[getUpAxis()]=1;
		return aniDir;
	}


	virtual	int	calculateSerializeBufferSize() const;

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual	const char*	serialize(void* dataBuffer, btlSerializer* serializer) const;

	SIMD_FORCE_INLINE	void	deSerializeFloat(struct btlCapsuleShapeData* dataBuffer);

};

///btCapsuleShapeX represents a capsule around the Z axis
///the total height is height+2*radius, so the height is just the height between the center of each 'sphere' of the capsule caps.
class btlCapsuleShapeX : public btlCapsuleShape
{
public:

	btlCapsuleShapeX(btlScalar radius,btlScalar height);
		
	//debugging
	virtual const char*	getName()const
	{
		return "CapsuleX";
	}

	

};

///btCapsuleShapeZ represents a capsule around the Z axis
///the total height is height+2*radius, so the height is just the height between the center of each 'sphere' of the capsule caps.
class btlCapsuleShapeZ : public btlCapsuleShape
{
public:
	btlCapsuleShapeZ(btlScalar radius,btlScalar height);

		//debugging
	virtual const char*	getName()const
	{
		return "CapsuleZ";
	}

	
};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct	btlCapsuleShapeData
{
	btlConvexInternalShapeData	m_convexInternalShapeData;

	int	m_upAxis;

	char	m_padding[4];
};

SIMD_FORCE_INLINE	int	btlCapsuleShape::calculateSerializeBufferSize() const
{
	return sizeof(btlCapsuleShapeData);
}

	///fills the dataBuffer and returns the struct name (and 0 on failure)
SIMD_FORCE_INLINE	const char*	btlCapsuleShape::serialize(void* dataBuffer, btlSerializer* serializer) const
{
	btlCapsuleShapeData* shapeData = (btlCapsuleShapeData*) dataBuffer;
	
	btlConvexInternalShape::serialize(&shapeData->m_convexInternalShapeData,serializer);

	shapeData->m_upAxis = m_upAxis;
	
	return "btCapsuleShapeData";
}

SIMD_FORCE_INLINE	void	btlCapsuleShape::deSerializeFloat(btlCapsuleShapeData* dataBuffer)
{
	m_implicitShapeDimensions.deSerializeFloat(dataBuffer->m_convexInternalShapeData.m_implicitShapeDimensions);
	m_collisionMargin = dataBuffer->m_convexInternalShapeData.m_collisionMargin;
	m_localScaling.deSerializeFloat(dataBuffer->m_convexInternalShapeData.m_localScaling);
	//it is best to already pre-allocate the matching btCapsuleShape*(X/Z) version to match m_upAxis
	m_upAxis = dataBuffer->m_upAxis;
}

#endif //BT_CAPSULE_SHAPE_H
