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

#ifndef BT_CONE_MINKOWSKI_H
#define BT_CONE_MINKOWSKI_H

#include "btConvexInternalShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h" // for the types

///The btConeShape implements a cone shape primitive, centered around the origin and aligned with the Y axis. The btConeShapeX is aligned around the X axis and btConeShapeZ around the Z axis.
ATTRIBUTE_ALIGNED16(class) btlConeShape : public btlConvexInternalShape

{

	btlScalar m_sinAngle;
	btlScalar m_radius;
	btlScalar m_height;
	int		m_coneIndices[3];
	btlVector3 coneLocalSupport(const btlVector3& v) const;


public:
	BT_DECLARE_ALIGNED_ALLOCATOR();
	
	btlConeShape (btlScalar radius,btlScalar height);
	
	virtual btlVector3	localGetSupportingVertex(const btlVector3& vec) const;
	virtual btlVector3	localGetSupportingVertexWithoutMargin(const btlVector3& vec) const;
	virtual void	batchedUnitVectorGetSupportingVertexWithoutMargin(const btlVector3* vectors,btlVector3* supportVerticesOut,int numVectors) const;

	btlScalar getRadius() const { return m_radius;}
	btlScalar getHeight() const { return m_height;}


	virtual void	calculateLocalInertia(btlScalar mass,btlVector3& inertia) const
	{
		btlTransform identity;
		identity.setIdentity();
		btlVector3 aabbMin,aabbMax;
		getAabb(identity,aabbMin,aabbMax);

		btlVector3 halfExtents = (aabbMax-aabbMin)*btlScalar(0.5);

		btlScalar margin = getMargin();

		btlScalar lx=btlScalar(2.)*(halfExtents.x()+margin);
		btlScalar ly=btlScalar(2.)*(halfExtents.y()+margin);
		btlScalar lz=btlScalar(2.)*(halfExtents.z()+margin);
		const btlScalar x2 = lx*lx;
		const btlScalar y2 = ly*ly;
		const btlScalar z2 = lz*lz;
		const btlScalar scaledmass = mass * btlScalar(0.08333333);

		inertia = scaledmass * (btlVector3(y2+z2,x2+z2,x2+y2));

//		inertia.x() = scaledmass * (y2+z2);
//		inertia.y() = scaledmass * (x2+z2);
//		inertia.z() = scaledmass * (x2+y2);
	}


		virtual const char*	getName()const 
		{
			return "Cone";
		}
		
		///choose upAxis index
		void	setConeUpIndex(int upIndex);
		
		int	getConeUpIndex() const
		{
			return m_coneIndices[1];
		}

	virtual btlVector3	getAnisotropicRollingFrictionDirection() const
	{
		return btlVector3 (0,1,0);
	}

	virtual void	setLocalScaling(const btlVector3& scaling);
	
	
	virtual	int	calculateSerializeBufferSize() const;
	
	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual	const char*	serialize(void* dataBuffer, btlSerializer* serializer) const;
	

};

///btConeShape implements a Cone shape, around the X axis
class btlConeShapeX : public btlConeShape
{
	public:
		btlConeShapeX(btlScalar radius,btlScalar height);

	virtual btlVector3	getAnisotropicRollingFrictionDirection() const
	{
		return btlVector3 (1,0,0);
	}

	//debugging
	virtual const char*	getName()const
	{
		return "ConeX";
	}
	
	
};

///btConeShapeZ implements a Cone shape, around the Z axis
class btlConeShapeZ : public btlConeShape
{
public:
	btlConeShapeZ(btlScalar radius,btlScalar height);

	virtual btlVector3	getAnisotropicRollingFrictionDirection() const
	{
		return btlVector3 (0,0,1);
	}

	//debugging
	virtual const char*	getName()const
	{
		return "ConeZ";
	}
	
	
};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct	btlConeShapeData
{
	btlConvexInternalShapeData	m_convexInternalShapeData;
	
	int	m_upIndex;
	
	char	m_padding[4];
};

SIMD_FORCE_INLINE	int	btlConeShape::calculateSerializeBufferSize() const
{
	return sizeof(btlConeShapeData);
}

///fills the dataBuffer and returns the struct name (and 0 on failure)
SIMD_FORCE_INLINE	const char*	btlConeShape::serialize(void* dataBuffer, btlSerializer* serializer) const
{
	btlConeShapeData* shapeData = (btlConeShapeData*) dataBuffer;
	
	btlConvexInternalShape::serialize(&shapeData->m_convexInternalShapeData,serializer);
	
	shapeData->m_upIndex = m_coneIndices[1];
	
	return "btConeShapeData";
}

#endif //BT_CONE_MINKOWSKI_H

