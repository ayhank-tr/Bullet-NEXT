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

#ifndef BT_TRIANGLE_MESH_SHAPE_H
#define BT_TRIANGLE_MESH_SHAPE_H

#include "btConcaveShape.h"
#include "btStridingMeshInterface.h"


///The btTriangleMeshShape is an internal concave triangle mesh interface. Don't use this class directly, use btBvhTriangleMeshShape instead.
ATTRIBUTE_ALIGNED16(class) btlTriangleMeshShape : public btlConcaveShape
{
protected:
	btlVector3	m_localAabbMin;
	btlVector3	m_localAabbMax;
	btlStridingMeshInterface* m_meshInterface;

	///btTriangleMeshShape constructor has been disabled/protected, so that users will not mistakenly use this class.
	///Don't use btTriangleMeshShape but use btBvhTriangleMeshShape instead!
	btlTriangleMeshShape(btlStridingMeshInterface* meshInterface);

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	virtual ~btlTriangleMeshShape();

	virtual btlVector3 localGetSupportingVertex(const btlVector3& vec) const;

	virtual btlVector3	localGetSupportingVertexWithoutMargin(const btlVector3& vec)const
	{
		btlAssert(0);
		return localGetSupportingVertex(vec);
	}

	void	recalcLocalAabb();

	virtual void getAabb(const btlTransform& t,btlVector3& aabbMin,btlVector3& aabbMax) const;

	virtual void	processAllTriangles(btlTriangleCallback* callback,const btlVector3& aabbMin,const btlVector3& aabbMax) const;

	virtual void	calculateLocalInertia(btlScalar mass,btlVector3& inertia) const;

	virtual void	setLocalScaling(const btlVector3& scaling);
	virtual const btlVector3& getLocalScaling() const;
	
	btlStridingMeshInterface* getMeshInterface()
	{
		return m_meshInterface;
	}

	const btlStridingMeshInterface* getMeshInterface() const
	{
		return m_meshInterface;
	}

	const btlVector3& getLocalAabbMin() const
	{
		return m_localAabbMin;
	}
	const btlVector3& getLocalAabbMax() const
	{
		return m_localAabbMax;
	}



	//debugging
	virtual const char*	getName()const {return "TRIANGLEMESH";}

	

};




#endif //BT_TRIANGLE_MESH_SHAPE_H
