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

#ifndef BT_POLYHEDRAL_CONVEX_SHAPE_H
#define BT_POLYHEDRAL_CONVEX_SHAPE_H

#include "LinearMath/btMatrix3x3.h"
#include "btConvexInternalShape.h"
class btlConvexPolyhedron;


///The btPolyhedralConvexShape is an internal interface class for polyhedral convex shapes.
ATTRIBUTE_ALIGNED16(class) btlPolyhedralConvexShape : public btlConvexInternalShape
{
	

protected:
	
	btlConvexPolyhedron* m_polyhedron;

public:
	
	BT_DECLARE_ALIGNED_ALLOCATOR();
	

	btlPolyhedralConvexShape();

	virtual ~btlPolyhedralConvexShape();

	///optional method mainly used to generate multiple contact points by clipping polyhedral features (faces/edges)
	///experimental/work-in-progress
	virtual bool	initializePolyhedralFeatures(int shiftVerticesByMargin=0);

	const btlConvexPolyhedron*	getConvexPolyhedron() const
	{
		return m_polyhedron;
	}

	//brute force implementations

	virtual btlVector3	localGetSupportingVertexWithoutMargin(const btlVector3& vec)const;
	virtual void	batchedUnitVectorGetSupportingVertexWithoutMargin(const btlVector3* vectors,btlVector3* supportVerticesOut,int numVectors) const;
	
	virtual void	calculateLocalInertia(btlScalar mass,btlVector3& inertia) const;
	
	
	virtual int	getNumVertices() const = 0 ;
	virtual int getNumEdges() const = 0;
	virtual void getEdge(int i,btlVector3& pa,btlVector3& pb) const = 0;
	virtual void getVertex(int i,btlVector3& vtx) const = 0;
	virtual int	getNumPlanes() const = 0;
	virtual void getPlane(btlVector3& planeNormal,btlVector3& planeSupport,int i ) const = 0;
//	virtual int getIndex(int i) const = 0 ; 

	virtual	bool isInside(const btlVector3& pt,btlScalar tolerance) const = 0;
	
};


///The btPolyhedralConvexAabbCachingShape adds aabb caching to the btPolyhedralConvexShape
class btlPolyhedralConvexAabbCachingShape : public btlPolyhedralConvexShape
{

	btlVector3	m_localAabbMin;
	btlVector3	m_localAabbMax;
	bool		m_isLocalAabbValid;
		
protected:

	void setCachedLocalAabb (const btlVector3& aabbMin, const btlVector3& aabbMax)
	{
		m_isLocalAabbValid = true;
		m_localAabbMin = aabbMin;
		m_localAabbMax = aabbMax;
	}

	inline void getCachedLocalAabb (btlVector3& aabbMin, btlVector3& aabbMax) const
	{
		btlAssert(m_isLocalAabbValid);
		aabbMin = m_localAabbMin;
		aabbMax = m_localAabbMax;
	}

public:

	btlPolyhedralConvexAabbCachingShape();
	
	inline void getNonvirtualAabb(const btlTransform& trans,btlVector3& aabbMin,btlVector3& aabbMax, btlScalar margin) const
	{

		//lazy evaluation of local aabb
		btlAssert(m_isLocalAabbValid);
		btlTransformAabb(m_localAabbMin,m_localAabbMax,margin,trans,aabbMin,aabbMax);
	}

	virtual void	setLocalScaling(const btlVector3& scaling);

	virtual void getAabb(const btlTransform& t,btlVector3& aabbMin,btlVector3& aabbMax) const;

	void	recalcLocalAabb();

};

#endif //BT_POLYHEDRAL_CONVEX_SHAPE_H
