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

#ifndef BT_COMPOUND_SHAPE_H
#define BT_COMPOUND_SHAPE_H

#include "btCollisionShape.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btMatrix3x3.h"
#include "btCollisionMargin.h"
#include "LinearMath/btAlignedObjectArray.h"

//class btOptimizedBvh;
struct btlDbvt;

ATTRIBUTE_ALIGNED16(struct) btlCompoundShapeChild
{
	BT_DECLARE_ALIGNED_ALLOCATOR();

	btlTransform			m_transform;
	btlCollisionShape*	m_childShape;
	int					m_childShapeType;
	btlScalar			m_childMargin;
	struct btlDbvtNode*	m_node;
};

SIMD_FORCE_INLINE bool operator==(const btlCompoundShapeChild& c1, const btlCompoundShapeChild& c2)
{
	return  ( c1.m_transform      == c2.m_transform &&
		c1.m_childShape     == c2.m_childShape &&
		c1.m_childShapeType == c2.m_childShapeType &&
		c1.m_childMargin    == c2.m_childMargin );
}

/// The btCompoundShape allows to store multiple other btCollisionShapes
/// This allows for moving concave collision objects. This is more general then the static concave btBvhTriangleMeshShape.
/// It has an (optional) dynamic aabb tree to accelerate early rejection tests. 
/// @todo: This aabb tree can also be use to speed up ray tests on btCompoundShape, see http://code.google.com/p/bullet/issues/detail?id=25
/// Currently, removal of child shapes is only supported when disabling the aabb tree (pass 'false' in the constructor of btCompoundShape)
ATTRIBUTE_ALIGNED16(class) btlCompoundShape	: public btlCollisionShape
{
protected:
	btlAlignedObjectArray<btlCompoundShapeChild> m_children;
	btlVector3						m_localAabbMin;
	btlVector3						m_localAabbMax;

	btlDbvt*							m_dynamicAabbTree;

	///increment m_updateRevision when adding/removing/replacing child shapes, so that some caches can be updated
	int								m_updateRevision;

	btlScalar	m_collisionMargin;

	btlVector3	m_localScaling;

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	explicit btlCompoundShape(bool enableDynamicAabbTree = true, const int initialChildCapacity = 0);

	virtual ~btlCompoundShape();

	void	addChildShape(const btlTransform& localTransform,btlCollisionShape* shape);

	/// Remove all children shapes that contain the specified shape
	virtual void removeChildShape(btlCollisionShape* shape);

	void removeChildShapeByIndex(int childShapeindex);


	int		getNumChildShapes() const
	{
		return int (m_children.size());
	}

	btlCollisionShape* getChildShape(int index)
	{
		return m_children[index].m_childShape;
	}
	const btlCollisionShape* getChildShape(int index) const
	{
		return m_children[index].m_childShape;
	}

	btlTransform&	getChildTransform(int index)
	{
		return m_children[index].m_transform;
	}
	const btlTransform&	getChildTransform(int index) const
	{
		return m_children[index].m_transform;
	}

	///set a new transform for a child, and update internal data structures (local aabb and dynamic tree)
	void	updateChildTransform(int childIndex, const btlTransform& newChildTransform, bool shouldRecalculateLocalAabb = true);


	btlCompoundShapeChild* getChildList()
	{
		return &m_children[0];
	}

	///getAabb's default implementation is brute force, expected derived classes to implement a fast dedicated version
	virtual	void getAabb(const btlTransform& t,btlVector3& aabbMin,btlVector3& aabbMax) const;

	/** Re-calculate the local Aabb. Is called at the end of removeChildShapes. 
	Use this yourself if you modify the children or their transforms. */
	virtual void recalculateLocalAabb(); 

	virtual void	setLocalScaling(const btlVector3& scaling);

	virtual const btlVector3& getLocalScaling() const 
	{
		return m_localScaling;
	}

	virtual void	calculateLocalInertia(btlScalar mass,btlVector3& inertia) const;

	virtual void	setMargin(btlScalar margin)
	{
		m_collisionMargin = margin;
	}
	virtual btlScalar	getMargin() const
	{
		return m_collisionMargin;
	}
	virtual const char*	getName()const
	{
		return "Compound";
	}

	const btlDbvt*	getDynamicAabbTree() const
	{
		return m_dynamicAabbTree;
	}
	
	btlDbvt*	getDynamicAabbTree()
	{
		return m_dynamicAabbTree;
	}

	void createAabbTreeFromChildren();

	///computes the exact moment of inertia and the transform from the coordinate system defined by the principal axes of the moment of inertia
	///and the center of mass to the current coordinate system. "masses" points to an array of masses of the children. The resulting transform
	///"principal" has to be applied inversely to all children transforms in order for the local coordinate system of the compound
	///shape to be centered at the center of mass and to coincide with the principal axes. This also necessitates a correction of the world transform
	///of the collision object by the principal transform.
	void calculatePrincipalAxisTransform(btlScalar* masses, btlTransform& principal, btlVector3& inertia) const;

	int	getUpdateRevision() const
	{
		return m_updateRevision;
	}

	virtual	int	calculateSerializeBufferSize() const;

	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual	const char*	serialize(void* dataBuffer, btlSerializer* serializer) const;


};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct btlCompoundShapeChildData
{
	btlTransformFloatData	m_transform;
	btlCollisionShapeData	*m_childShape;
	int						m_childShapeType;
	float					m_childMargin;
};

///do not change those serialization structures, it requires an updated sBulletDNAstr/sBulletDNAstr64
struct	btlCompoundShapeData
{
	btlCollisionShapeData		m_collisionShapeData;

	btlCompoundShapeChildData	*m_childShapePtr;

	int							m_numChildShapes;

	float	m_collisionMargin;

};


SIMD_FORCE_INLINE	int	btlCompoundShape::calculateSerializeBufferSize() const
{
	return sizeof(btlCompoundShapeData);
}







#endif //BT_COMPOUND_SHAPE_H
