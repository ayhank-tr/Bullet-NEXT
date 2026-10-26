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
#ifndef BT_SPHERE_MINKOWSKI_H
#define BT_SPHERE_MINKOWSKI_H

#include "btConvexInternalShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h" // for the types

///The btSphereShape implements an implicit sphere, centered around a local origin with radius.
ATTRIBUTE_ALIGNED16(class) btlSphereShape : public btlConvexInternalShape

{
	
public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	btlSphereShape (btlScalar radius) : btlConvexInternalShape ()
	{
		m_shapeType = SPHERE_SHAPE_PROXYTYPE;
		m_implicitShapeDimensions.setX(radius);
		m_collisionMargin = radius;
	}
	
	virtual btlVector3	localGetSupportingVertex(const btlVector3& vec)const;
	virtual btlVector3	localGetSupportingVertexWithoutMargin(const btlVector3& vec)const;
	//notice that the vectors should be unit length
	virtual void	batchedUnitVectorGetSupportingVertexWithoutMargin(const btlVector3* vectors,btlVector3* supportVerticesOut,int numVectors) const;


	virtual void	calculateLocalInertia(btlScalar mass,btlVector3& inertia) const;

	virtual void getAabb(const btlTransform& t,btlVector3& aabbMin,btlVector3& aabbMax) const;


	btlScalar	getRadius() const { return m_implicitShapeDimensions.getX() * m_localScaling.getX();}

	void	setUnscaledRadius(btlScalar	radius)
	{
		m_implicitShapeDimensions.setX(radius);
		btlConvexInternalShape::setMargin(radius);
	}

	//debugging
	virtual const char*	getName()const {return "SPHERE";}

	virtual void	setMargin(btlScalar margin)
	{
		btlConvexInternalShape::setMargin(margin);
	}
	virtual btlScalar	getMargin() const
	{
		//to improve gjk behaviour, use radius+margin as the full margin, so never get into the penetration case
		//this means, non-uniform scaling is not supported anymore
		return getRadius();
	}


};


#endif //BT_SPHERE_MINKOWSKI_H
