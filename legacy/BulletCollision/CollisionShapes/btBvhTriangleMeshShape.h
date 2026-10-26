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

#ifndef BT_BVH_TRIANGLE_MESH_SHAPE_H
#define BT_BVH_TRIANGLE_MESH_SHAPE_H

#include "btTriangleMeshShape.h"
#include "btOptimizedBvh.h"
#include "LinearMath/btAlignedAllocator.h"
#include "btTriangleInfoMap.h"

///The btBvhTriangleMeshShape is a static-triangle mesh shape, it can only be used for fixed/non-moving objects.
///If you required moving concave triangle meshes, it is recommended to perform convex decomposition
///using HACD, see Bullet/Demos/ConvexDecompositionDemo. 
///Alternatively, you can use btGimpactMeshShape for moving concave triangle meshes.
///btBvhTriangleMeshShape has several optimizations, such as bounding volume hierarchy and 
///cache friendly traversal for PlayStation 3 Cell SPU. 
///It is recommended to enable useQuantizedAabbCompression for better memory usage.
///It takes a triangle mesh as input, for example a btTriangleMesh or btTriangleIndexVertexArray. The btBvhTriangleMeshShape class allows for triangle mesh deformations by a refit or partialRefit method.
///Instead of building the bounding volume hierarchy acceleration structure, it is also possible to serialize (save) and deserialize (load) the structure from disk.
///See Demos\ConcaveDemo\ConcavePhysicsDemo.cpp for an example.
ATTRIBUTE_ALIGNED16(class) btlBvhTriangleMeshShape : public btlTriangleMeshShape
{

	btlOptimizedBvh*	m_bvh;
	btlTriangleInfoMap*	m_triangleInfoMap;

	bool m_useQuantizedAabbCompression;
	bool m_ownsBvh;
#ifdef __clang__
	bool m_pad[11] __attribute__((unused));////need padding due to alignment
#else
	bool m_pad[11];////need padding due to alignment
#endif

public:

	BT_DECLARE_ALIGNED_ALLOCATOR();

	
	btlBvhTriangleMeshShape(btlStridingMeshInterface* meshInterface, bool useQuantizedAabbCompression, bool buildBvh = true);

	///optionally pass in a larger bvh aabb, used for quantization. This allows for deformations within this aabb
	btlBvhTriangleMeshShape(btlStridingMeshInterface* meshInterface, bool useQuantizedAabbCompression,const btlVector3& bvhAabbMin,const btlVector3& bvhAabbMax, bool buildBvh = true);
	
	virtual ~btlBvhTriangleMeshShape();

	bool getOwnsBvh () const
	{
		return m_ownsBvh;
	}


	
	void performRaycast (btlTriangleCallback* callback, const btlVector3& raySource, const btlVector3& rayTarget);
	void performConvexcast (btlTriangleCallback* callback, const btlVector3& boxSource, const btlVector3& boxTarget, const btlVector3& boxMin, const btlVector3& boxMax);

	virtual void	processAllTriangles(btlTriangleCallback* callback,const btlVector3& aabbMin,const btlVector3& aabbMax) const;

	void	refitTree(const btlVector3& aabbMin,const btlVector3& aabbMax);

	///for a fast incremental refit of parts of the tree. Note: the entire AABB of the tree will become more conservative, it never shrinks
	void	partialRefitTree(const btlVector3& aabbMin,const btlVector3& aabbMax);

	//debugging
	virtual const char*	getName()const {return "BVHTRIANGLEMESH";}


	virtual void	setLocalScaling(const btlVector3& scaling);
	
	btlOptimizedBvh*	getOptimizedBvh()
	{
		return m_bvh;
	}

	void	setOptimizedBvh(btlOptimizedBvh* bvh, const btlVector3& localScaling=btlVector3(1,1,1));

	void    buildOptimizedBvh();

	bool	usesQuantizedAabbCompression() const
	{
		return	m_useQuantizedAabbCompression;
	}

	void	setTriangleInfoMap(btlTriangleInfoMap* triangleInfoMap)
	{
		m_triangleInfoMap = triangleInfoMap;
	}

	const btlTriangleInfoMap*	getTriangleInfoMap() const
	{
		return m_triangleInfoMap;
	}
	
	btlTriangleInfoMap*	getTriangleInfoMap()
	{
		return m_triangleInfoMap;
	}

	virtual	int	calculateSerializeBufferSize() const;

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual	const char*	serialize(void* dataBuffer, btlSerializer* serializer) const;

	virtual void	serializeSingleBvh(btlSerializer* serializer) const;

	virtual void	serializeSingleTriangleInfoMap(btlSerializer* serializer) const;

};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct	btlTriangleMeshShapeData
{
	btlCollisionShapeData	m_collisionShapeData;

	btlStridingMeshInterfaceData m_meshInterface;

	btlQuantizedBvhFloatData		*m_quantizedFloatBvh;
	btlQuantizedBvhDoubleData	*m_quantizedDoubleBvh;

	btlTriangleInfoMapData	*m_triangleInfoMap;
	
	float	m_collisionMargin;

	char m_pad3[4];
	
};


SIMD_FORCE_INLINE	int	btlBvhTriangleMeshShape::calculateSerializeBufferSize() const
{
	return sizeof(btlTriangleMeshShapeData);
}



#endif //BT_BVH_TRIANGLE_MESH_SHAPE_H
