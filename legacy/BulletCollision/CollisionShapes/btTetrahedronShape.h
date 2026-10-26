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

#ifndef BT_SIMPLEX_1TO4_SHAPE
#define BT_SIMPLEX_1TO4_SHAPE


#include "btPolyhedralConvexShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"


///The btBU_Simplex1to4 implements tetrahedron, triangle, line, vertex collision shapes. In most cases it is better to use btConvexHullShape instead.
ATTRIBUTE_ALIGNED16(class) btlBU_Simplex1to4 : public btlPolyhedralConvexAabbCachingShape
{
protected:

	int	m_numVertices;
	btlVector3	m_vertices[4];

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();
	
	btlBU_Simplex1to4();

	btlBU_Simplex1to4(const btlVector3& pt0);
	btlBU_Simplex1to4(const btlVector3& pt0,const btlVector3& pt1);
	btlBU_Simplex1to4(const btlVector3& pt0,const btlVector3& pt1,const btlVector3& pt2);
	btlBU_Simplex1to4(const btlVector3& pt0,const btlVector3& pt1,const btlVector3& pt2,const btlVector3& pt3);

    
	void	reset()
	{
		m_numVertices = 0;
	}
	
	virtual void getAabb(const btlTransform& t,btlVector3& aabbMin,btlVector3& aabbMax) const;

	void addVertex(const btlVector3& pt);

	//PolyhedralConvexShape interface

	virtual int	getNumVertices() const;

	virtual int getNumEdges() const;

	virtual void getEdge(int i,btlVector3& pa,btlVector3& pb) const;
	
	virtual void getVertex(int i,btlVector3& vtx) const;

	virtual int	getNumPlanes() const;

	virtual void getPlane(btlVector3& planeNormal,btlVector3& planeSupport,int i) const;

	virtual int getIndex(int i) const;

	virtual	bool isInside(const btlVector3& pt,btlScalar tolerance) const;


	///getName is for debugging
	virtual const char*	getName()const { return "btBU_Simplex1to4";}

};

#endif //BT_SIMPLEX_1TO4_SHAPE
