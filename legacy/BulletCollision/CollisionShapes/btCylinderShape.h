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

#ifndef BT_CYLINDER_MINKOWSKI_H
#define BT_CYLINDER_MINKOWSKI_H

#include "btBoxShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h" // for the types
#include "LinearMath/btVector3.h"

/// The btCylinderShape class implements a cylinder shape primitive, centered around the origin. Its central axis aligned with the Y axis. btCylinderShapeX is aligned with the X axis and btCylinderShapeZ around the Z axis.
ATTRIBUTE_ALIGNED16(class) btlCylinderShape : public btlConvexInternalShape

{

protected:

	int	m_upAxis;

public:

BT_DECLARE_ALIGNED_ALLOCATOR();

	btlVector3 getHalfExtentsWithMargin() const
	{
		btlVector3 halfExtents = getHalfExtentsWithoutMargin();
		btlVector3 margin(getMargin(),getMargin(),getMargin());
		halfExtents += margin;
		return halfExtents;
	}
	
	const btlVector3& getHalfExtentsWithoutMargin() const
	{
		return m_implicitShapeDimensions;//changed in Bullet 2.63: assume the scaling and margin are included
	}

	btlCylinderShape (const btlVector3& halfExtents);
	
	void getAabb(const btlTransform& t,btlVector3& aabbMin,btlVector3& aabbMax) const;

	virtual void	calculateLocalInertia(btlScalar mass,btlVector3& inertia) const;

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

	virtual btlVector3	localGetSupportingVertex(const btlVector3& vec) const
	{

		btlVector3 supVertex;
		supVertex = localGetSupportingVertexWithoutMargin(vec);
		
		if ( getMargin()!=btlScalar(0.) )
		{
			btlVector3 vecnorm = vec;
			if (vecnorm .length2() < (SIMD_EPSILON*SIMD_EPSILON))
			{
				vecnorm.setValue(btlScalar(-1.),btlScalar(-1.),btlScalar(-1.));
			} 
			vecnorm.normalize();
			supVertex+= getMargin() * vecnorm;
		}
		return supVertex;
	}


	//use box inertia
	//	virtual void	calculateLocalInertia(btScalar mass,btVector3& inertia) const;


	int	getUpAxis() const
	{
		return m_upAxis;
	}

	virtual btlVector3	getAnisotropicRollingFrictionDirection() const
	{
		btlVector3 aniDir(0,0,0);
		aniDir[getUpAxis()]=1;
		return aniDir;
	}

	virtual btlScalar getRadius() const
	{
		return getHalfExtentsWithMargin().getX();
	}

	virtual void	setLocalScaling(const btlVector3& scaling)
	{
		btlVector3 oldMargin(getMargin(),getMargin(),getMargin());
		btlVector3 implicitShapeDimensionsWithMargin = m_implicitShapeDimensions+oldMargin;
		btlVector3 unScaledImplicitShapeDimensionsWithMargin = implicitShapeDimensionsWithMargin / m_localScaling;

		btlConvexInternalShape::setLocalScaling(scaling);

		m_implicitShapeDimensions = (unScaledImplicitShapeDimensionsWithMargin * m_localScaling) - oldMargin;

	}

	//debugging
	virtual const char*	getName()const
	{
		return "CylinderY";
	}

	virtual	int	calculateSerializeBufferSize() const;

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual	const char*	serialize(void* dataBuffer, btlSerializer* serializer) const;

};

class btlCylinderShapeX : public btlCylinderShape
{
public:
	BT_DECLARE_ALIGNED_ALLOCATOR();
	
	btlCylinderShapeX (const btlVector3& halfExtents);

	virtual btlVector3	localGetSupportingVertexWithoutMargin(const btlVector3& vec)const;
	virtual void	batchedUnitVectorGetSupportingVertexWithoutMargin(const btlVector3* vectors,btlVector3* supportVerticesOut,int numVectors) const;
	
		//debugging
	virtual const char*	getName()const
	{
		return "CylinderX";
	}

	virtual btlScalar getRadius() const
	{
		return getHalfExtentsWithMargin().getY();
	}

};

class btlCylinderShapeZ : public btlCylinderShape
{
public:
	BT_DECLARE_ALIGNED_ALLOCATOR();
	
	btlCylinderShapeZ (const btlVector3& halfExtents);

	virtual btlVector3	localGetSupportingVertexWithoutMargin(const btlVector3& vec)const;
	virtual void	batchedUnitVectorGetSupportingVertexWithoutMargin(const btlVector3* vectors,btlVector3* supportVerticesOut,int numVectors) const;

		//debugging
	virtual const char*	getName()const
	{
		return "CylinderZ";
	}

	virtual btlScalar getRadius() const
	{
		return getHalfExtentsWithMargin().getX();
	}

};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct	btlCylinderShapeData
{
	btlConvexInternalShapeData	m_convexInternalShapeData;

	int	m_upAxis;

	char	m_padding[4];
};

SIMD_FORCE_INLINE	int	btlCylinderShape::calculateSerializeBufferSize() const
{
	return sizeof(btlCylinderShapeData);
}

	///fills the dataBuffer and returns the struct name (and 0 on failure)
SIMD_FORCE_INLINE	const char*	btlCylinderShape::serialize(void* dataBuffer, btlSerializer* serializer) const
{
	btlCylinderShapeData* shapeData = (btlCylinderShapeData*) dataBuffer;
	
	btlConvexInternalShape::serialize(&shapeData->m_convexInternalShapeData,serializer);

	shapeData->m_upAxis = m_upAxis;
	
	return "btCylinderShapeData";
}



#endif //BT_CYLINDER_MINKOWSKI_H

