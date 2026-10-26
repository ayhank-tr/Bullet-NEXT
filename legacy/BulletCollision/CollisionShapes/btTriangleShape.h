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

#ifndef BT_OBB_TRIANGLE_MINKOWSKI_H
#define BT_OBB_TRIANGLE_MINKOWSKI_H

#include "btConvexShape.h"
#include "btBoxShape.h"

ATTRIBUTE_ALIGNED16(class) btlTriangleShape : public btlPolyhedralConvexShape
{


public:

BT_DECLARE_ALIGNED_ALLOCATOR();

	btlVector3	m_vertices1[3];

	virtual int getNumVertices() const
	{
		return 3;
	}

	btlVector3& getVertexPtr(int index)
	{
		return m_vertices1[index];
	}

	const btlVector3& getVertexPtr(int index) const
	{
		return m_vertices1[index];
	}
	virtual void getVertex(int index,btlVector3& vert) const
	{
		vert = m_vertices1[index];
	}

	virtual int getNumEdges() const
	{
		return 3;
	}
	
	virtual void getEdge(int i,btlVector3& pa,btlVector3& pb) const
	{
		getVertex(i,pa);
		getVertex((i+1)%3,pb);
	}


	virtual void getAabb(const btlTransform& t,btlVector3& aabbMin,btlVector3& aabbMax)const 
	{
//		btAssert(0);
		getAabbSlow(t,aabbMin,aabbMax);
	}

	btlVector3 localGetSupportingVertexWithoutMargin(const btlVector3& dir)const 
	{
        btlVector3 dots = dir.dot3(m_vertices1[0], m_vertices1[1], m_vertices1[2]);
	  	return m_vertices1[dots.maxAxis()];

	}

	virtual void	batchedUnitVectorGetSupportingVertexWithoutMargin(const btlVector3* vectors,btlVector3* supportVerticesOut,int numVectors) const
	{
		for (int i=0;i<numVectors;i++)
		{
			const btlVector3& dir = vectors[i];
            btlVector3 dots = dir.dot3(m_vertices1[0], m_vertices1[1], m_vertices1[2]);
  			supportVerticesOut[i] = m_vertices1[dots.maxAxis()];
		}

	}

	btlTriangleShape() : btlPolyhedralConvexShape ()
    {
		m_shapeType = TRIANGLE_SHAPE_PROXYTYPE;
	}

	btlTriangleShape(const btlVector3& p0,const btlVector3& p1,const btlVector3& p2) : btlPolyhedralConvexShape ()
    {
		m_shapeType = TRIANGLE_SHAPE_PROXYTYPE;
        m_vertices1[0] = p0;
        m_vertices1[1] = p1;
        m_vertices1[2] = p2;
    }


	virtual void getPlane(btlVector3& planeNormal,btlVector3& planeSupport,int i) const
	{
		getPlaneEquation(i,planeNormal,planeSupport);
	}

	virtual int	getNumPlanes() const
	{
		return 1;
	}

	void calcNormal(btlVector3& normal) const
	{
		normal = (m_vertices1[1]-m_vertices1[0]).cross(m_vertices1[2]-m_vertices1[0]);
		normal.normalize();
	}

	virtual void getPlaneEquation(int i, btlVector3& planeNormal,btlVector3& planeSupport) const
	{
		(void)i;
		calcNormal(planeNormal);
		planeSupport = m_vertices1[0];
	}

	virtual void	calculateLocalInertia(btlScalar mass,btlVector3& inertia) const
	{
		(void)mass;
		btlAssert(0);
		inertia.setValue(btlScalar(0.),btlScalar(0.),btlScalar(0.));
	}

		virtual	bool isInside(const btlVector3& pt,btlScalar tolerance) const
	{
		btlVector3 normal;
		calcNormal(normal);
		//distance to plane
		btlScalar dist = pt.dot(normal);
		btlScalar planeconst = m_vertices1[0].dot(normal);
		dist -= planeconst;
		if (dist >= -tolerance && dist <= tolerance)
		{
			//inside check on edge-planes
			int i;
			for (i=0;i<3;i++)
			{
				btlVector3 pa,pb;
				getEdge(i,pa,pb);
				btlVector3 edge = pb-pa;
				btlVector3 edgeNormal = edge.cross(normal);
				edgeNormal.normalize();
				btlScalar dist = pt.dot( edgeNormal);
				btlScalar edgeConst = pa.dot(edgeNormal);
				dist -= edgeConst;
				if (dist < -tolerance)
					return false;
			}
			
			return true;
		}

		return false;
	}
		//debugging
		virtual const char*	getName()const
		{
			return "Triangle";
		}

		virtual int		getNumPreferredPenetrationDirections() const
		{
			return 2;
		}
		
		virtual void	getPreferredPenetrationDirection(int index, btlVector3& penetrationVector) const
		{
			calcNormal(penetrationVector);
			if (index)
				penetrationVector *= btlScalar(-1.);
		}


};

#endif //BT_OBB_TRIANGLE_MINKOWSKI_H

