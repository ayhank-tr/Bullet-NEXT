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
#ifndef BT_CONVEX_TRIANGLEMESH_SHAPE_H
#define BT_CONVEX_TRIANGLEMESH_SHAPE_H


#include "btPolyhedralConvexShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h" // for the types


/// The btConvexTriangleMeshShape is a convex hull of a triangle mesh, but the performance is not as good as btConvexHullShape.
/// A small benefit of this class is that it uses the btStridingMeshInterface, so you can avoid the duplication of the triangle mesh data. Nevertheless, most users should use the much better performing btConvexHullShape instead.
ATTRIBUTE_ALIGNED16(class) btlConvexTriangleMeshShape : public btlPolyhedralConvexAabbCachingShape
{

	class btlStridingMeshInterface*	m_stridingMesh;

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();
	
	btlConvexTriangleMeshShape(btlStridingMeshInterface* meshInterface, bool calcAabb = true);

	class btlStridingMeshInterface*	getMeshInterface()
	{
		return m_stridingMesh;
	}
	const class btlStridingMeshInterface* getMeshInterface() const
	{
		return m_stridingMesh;
	}
	
	virtual btlVector3	localGetSupportingVertex(const btlVector3& vec)const;
	virtual btlVector3	localGetSupportingVertexWithoutMargin(const btlVector3& vec)const;
	virtual void	batchedUnitVectorGetSupportingVertexWithoutMargin(const btlVector3* vectors,btlVector3* supportVerticesOut,int numVectors) const;
	
	//debugging
	virtual const char*	getName()const {return "ConvexTrimesh";}
	
	virtual int	getNumVertices() const;
	virtual int getNumEdges() const;
	virtual void getEdge(int i,btlVector3& pa,btlVector3& pb) const;
	virtual void getVertex(int i,btlVector3& vtx) const;
	virtual int	getNumPlanes() const;
	virtual void getPlane(btlVector3& planeNormal,btlVector3& planeSupport,int i ) const;
	virtual	bool isInside(const btlVector3& pt,btlScalar tolerance) const;

	
	virtual void	setLocalScaling(const btlVector3& scaling);
	virtual const btlVector3& getLocalScaling() const;

	///computes the exact moment of inertia and the transform from the coordinate system defined by the principal axes of the moment of inertia
	///and the center of mass to the current coordinate system. A mass of 1 is assumed, for other masses just multiply the computed "inertia"
	///by the mass. The resulting transform "principal" has to be applied inversely to the mesh in order for the local coordinate system of the
	///shape to be centered at the center of mass and to coincide with the principal axes. This also necessitates a correction of the world transform
	///of the collision object by the principal transform. This method also computes the volume of the convex mesh.
	void calculatePrincipalAxisTransform(btlTransform& principal, btlVector3& inertia, btlScalar& volume) const;

};



#endif //BT_CONVEX_TRIANGLEMESH_SHAPE_H



