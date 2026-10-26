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

#ifndef BT_STATIC_PLANE_SHAPE_H
#define BT_STATIC_PLANE_SHAPE_H

#include "btConcaveShape.h"


///The btStaticPlaneShape simulates an infinite non-moving (static) collision plane.
ATTRIBUTE_ALIGNED16(class) btlStaticPlaneShape : public btlConcaveShape
{
protected:
	btlVector3	m_localAabbMin;
	btlVector3	m_localAabbMax;
	
	btlVector3	m_planeNormal;
	btlScalar      m_planeConstant;
	btlVector3	m_localScaling;

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	btlStaticPlaneShape(const btlVector3& planeNormal,btlScalar planeConstant);

	virtual ~btlStaticPlaneShape();


	virtual void getAabb(const btlTransform& t,btlVector3& aabbMin,btlVector3& aabbMax) const;

	virtual void	processAllTriangles(btlTriangleCallback* callback,const btlVector3& aabbMin,const btlVector3& aabbMax) const;

	virtual void	calculateLocalInertia(btlScalar mass,btlVector3& inertia) const;

	virtual void	setLocalScaling(const btlVector3& scaling);
	virtual const btlVector3& getLocalScaling() const;
	
	const btlVector3&	getPlaneNormal() const
	{
		return	m_planeNormal;
	}

	const btlScalar&	getPlaneConstant() const
	{
		return	m_planeConstant;
	}

	//debugging
	virtual const char*	getName()const {return "STATICPLANE";}

	virtual	int	calculateSerializeBufferSize() const;

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual	const char*	serialize(void* dataBuffer, btlSerializer* serializer) const;


};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct	btlStaticPlaneShapeData
{
	btlCollisionShapeData	m_collisionShapeData;

	btlVector3FloatData	m_localScaling;
	btlVector3FloatData	m_planeNormal;
	float			m_planeConstant;
	char	m_pad[4];
};


SIMD_FORCE_INLINE	int	btlStaticPlaneShape::calculateSerializeBufferSize() const
{
	return sizeof(btlStaticPlaneShapeData);
}

///fills the dataBuffer and returns the struct name (and 0 on failure)
SIMD_FORCE_INLINE	const char*	btlStaticPlaneShape::serialize(void* dataBuffer, btlSerializer* serializer) const
{
	btlStaticPlaneShapeData* planeData = (btlStaticPlaneShapeData*) dataBuffer;
	btlCollisionShape::serialize(&planeData->m_collisionShapeData,serializer);

	m_localScaling.serializeFloat(planeData->m_localScaling);
	m_planeNormal.serializeFloat(planeData->m_planeNormal);
	planeData->m_planeConstant = float(m_planeConstant);
		
	return "btStaticPlaneShapeData";
}


#endif //BT_STATIC_PLANE_SHAPE_H



