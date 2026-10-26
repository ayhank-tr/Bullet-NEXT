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

#ifndef BT_UNIFORM_SCALING_SHAPE_H
#define BT_UNIFORM_SCALING_SHAPE_H

#include "btConvexShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h" // for the types

///The btUniformScalingShape allows to re-use uniform scaled instances of btConvexShape in a memory efficient way.
///Istead of using btUniformScalingShape, it is better to use the non-uniform setLocalScaling method on convex shapes that implement it.
ATTRIBUTE_ALIGNED16(class) btlUniformScalingShape : public btlConvexShape
{
	btlConvexShape*	m_childConvexShape;

	btlScalar	m_uniformScalingFactor;
	
	public:
	
	BT_DECLARE_ALIGNED_ALLOCATOR();
	
	btlUniformScalingShape(	btlConvexShape* convexChildShape, btlScalar uniformScalingFactor);
	
	virtual ~btlUniformScalingShape();
	
	virtual btlVector3	localGetSupportingVertexWithoutMargin(const btlVector3& vec)const;

	virtual btlVector3	localGetSupportingVertex(const btlVector3& vec)const;

	virtual void	batchedUnitVectorGetSupportingVertexWithoutMargin(const btlVector3* vectors,btlVector3* supportVerticesOut,int numVectors) const;

	virtual void	calculateLocalInertia(btlScalar mass,btlVector3& inertia) const;

	btlScalar	getUniformScalingFactor() const
	{
		return m_uniformScalingFactor;
	}

	btlConvexShape*	getChildShape() 
	{
		return m_childConvexShape;
	}

	const btlConvexShape*	getChildShape() const
	{
		return m_childConvexShape;
	}

	virtual const char*	getName()const 
	{
		return "UniformScalingShape";
	}
	


	///////////////////////////


	///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
	void getAabb(const btlTransform& t,btlVector3& aabbMin,btlVector3& aabbMax) const;

	virtual void getAabbSlow(const btlTransform& t,btlVector3& aabbMin,btlVector3& aabbMax) const;

	virtual void	setLocalScaling(const btlVector3& scaling) ;
	virtual const btlVector3& getLocalScaling() const ;

	virtual void	setMargin(btlScalar margin);
	virtual btlScalar	getMargin() const;

	virtual int		getNumPreferredPenetrationDirections() const;
	
	virtual void	getPreferredPenetrationDirection(int index, btlVector3& penetrationVector) const;


};

#endif //BT_UNIFORM_SCALING_SHAPE_H
