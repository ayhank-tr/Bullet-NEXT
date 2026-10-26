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

#ifndef BT_CONVEX_POINT_CLOUD_SHAPE_H
#define BT_CONVEX_POINT_CLOUD_SHAPE_H

#include "btPolyhedralConvexShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h" // for the types
#include "LinearMath/btAlignedObjectArray.h"

///The btConvexPointCloudShape implements an implicit convex hull of an array of vertices.
ATTRIBUTE_ALIGNED16(class) btlConvexPointCloudShape : public btlPolyhedralConvexAabbCachingShape
{
	btlVector3* m_unscaledPoints;
	int m_numPoints;

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	btlConvexPointCloudShape()
	{
		m_localScaling.setValue(1.f,1.f,1.f);
		m_shapeType = CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE;
		m_unscaledPoints = 0;
		m_numPoints = 0;
	}

	btlConvexPointCloudShape(btlVector3* points,int numPoints, const btlVector3& localScaling,bool computeAabb = true)
	{
		m_localScaling = localScaling;
		m_shapeType = CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE;
		m_unscaledPoints = points;
		m_numPoints = numPoints;

		if (computeAabb)
			recalcLocalAabb();
	}

	void setPoints (btlVector3* points, int numPoints, bool computeAabb = true,const btlVector3& localScaling=btlVector3(1.f,1.f,1.f))
	{
		m_unscaledPoints = points;
		m_numPoints = numPoints;
		m_localScaling = localScaling;

		if (computeAabb)
			recalcLocalAabb();
	}

	SIMD_FORCE_INLINE	btlVector3* getUnscaledPoints()
	{
		return m_unscaledPoints;
	}

	SIMD_FORCE_INLINE	const btlVector3* getUnscaledPoints() const
	{
		return m_unscaledPoints;
	}

	SIMD_FORCE_INLINE	int getNumPoints() const 
	{
		return m_numPoints;
	}

	SIMD_FORCE_INLINE	btlVector3	getScaledPoint( int index) const
	{
		return m_unscaledPoints[index] * m_localScaling;
	}

#ifndef __SPU__
	virtual btlVector3	localGetSupportingVertex(const btlVector3& vec)const;
	virtual btlVector3	localGetSupportingVertexWithoutMargin(const btlVector3& vec)const;
	virtual void	batchedUnitVectorGetSupportingVertexWithoutMargin(const btlVector3* vectors,btlVector3* supportVerticesOut,int numVectors) const;
#endif


	//debugging
	virtual const char*	getName()const {return "ConvexPointCloud";}

	virtual int	getNumVertices() const;
	virtual int getNumEdges() const;
	virtual void getEdge(int i,btlVector3& pa,btlVector3& pb) const;
	virtual void getVertex(int i,btlVector3& vtx) const;
	virtual int	getNumPlanes() const;
	virtual void getPlane(btlVector3& planeNormal,btlVector3& planeSupport,int i ) const;
	virtual	bool isInside(const btlVector3& pt,btlScalar tolerance) const;

	///in case we receive negative scaling
	virtual void	setLocalScaling(const btlVector3& scaling);
};


#endif //BT_CONVEX_POINT_CLOUD_SHAPE_H

