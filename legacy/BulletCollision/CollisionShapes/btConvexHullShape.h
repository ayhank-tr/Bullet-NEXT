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

#ifndef BT_CONVEX_HULL_SHAPE_H
#define BT_CONVEX_HULL_SHAPE_H

#include "btPolyhedralConvexShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h" // for the types
#include "LinearMath/btAlignedObjectArray.h"


///The btConvexHullShape implements an implicit convex hull of an array of vertices.
///Bullet provides a general and fast collision detector for convex shapes based on GJK and EPA using localGetSupportingVertex.
ATTRIBUTE_ALIGNED16(class) btlConvexHullShape : public btlPolyhedralConvexAabbCachingShape
{
	btlAlignedObjectArray<btlVector3>	m_unscaledPoints;

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	
	///this constructor optionally takes in a pointer to points. Each point is assumed to be 3 consecutive btScalar (x,y,z), the striding defines the number of bytes between each point, in memory.
	///It is easier to not pass any points in the constructor, and just add one point at a time, using addPoint.
	///btConvexHullShape make an internal copy of the points.
	btlConvexHullShape(const btlScalar* points=0,int numPoints=0, int stride=sizeof(btlVector3));

	void addPoint(const btlVector3& point, bool recalculateLocalAabb = true);

	
	btlVector3* getUnscaledPoints()
	{
		return &m_unscaledPoints[0];
	}

	const btlVector3* getUnscaledPoints() const
	{
		return &m_unscaledPoints[0];
	}

	///getPoints is obsolete, please use getUnscaledPoints
	const btlVector3* getPoints() const
	{
		return getUnscaledPoints();
	}

	


	SIMD_FORCE_INLINE	btlVector3 getScaledPoint(int i) const
	{
		return m_unscaledPoints[i] * m_localScaling;
	}

	SIMD_FORCE_INLINE	int getNumPoints() const 
	{
		return m_unscaledPoints.size();
	}

	virtual btlVector3	localGetSupportingVertex(const btlVector3& vec)const;
	virtual btlVector3	localGetSupportingVertexWithoutMargin(const btlVector3& vec)const;
	virtual void	batchedUnitVectorGetSupportingVertexWithoutMargin(const btlVector3* vectors,btlVector3* supportVerticesOut,int numVectors) const;
	

	virtual void project(const btlTransform& trans, const btlVector3& dir, btlScalar& minProj, btlScalar& maxProj, btlVector3& witnesPtMin,btlVector3& witnesPtMax) const;


	//debugging
	virtual const char*	getName()const {return "Convex";}

	
	virtual int	getNumVertices() const;
	virtual int getNumEdges() const;
	virtual void getEdge(int i,btlVector3& pa,btlVector3& pb) const;
	virtual void getVertex(int i,btlVector3& vtx) const;
	virtual int	getNumPlanes() const;
	virtual void getPlane(btlVector3& planeNormal,btlVector3& planeSupport,int i ) const;
	virtual	bool isInside(const btlVector3& pt,btlScalar tolerance) const;

	///in case we receive negative scaling
	virtual void	setLocalScaling(const btlVector3& scaling);

	virtual	int	calculateSerializeBufferSize() const;

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual	const char*	serialize(void* dataBuffer, btlSerializer* serializer) const;

};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct	btlConvexHullShapeData
{
	btlConvexInternalShapeData	m_convexInternalShapeData;

	btlVector3FloatData	*m_unscaledPointsFloatPtr;
	btlVector3DoubleData	*m_unscaledPointsDoublePtr;

	int		m_numUnscaledPoints;
	char m_padding3[4];

};


SIMD_FORCE_INLINE	int	btlConvexHullShape::calculateSerializeBufferSize() const
{
	return sizeof(btlConvexHullShapeData);
}


#endif //BT_CONVEX_HULL_SHAPE_H

