#ifndef btl_GIM_BOX_SET_H_INCLUDED
#define btl_GIM_BOX_SET_H_INCLUDED

/*! \file gim_box_set.h
\author Francisco Leon Najera
*/
/*
This source file is part of GIMPACT Library.

For the latest info, see http://gimpact.sourceforge.net/

Copyright (c) 2007 Francisco Leon Najera. C.C. 80087371.
email: projectileman@yahoo.com


This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#include "LinearMath/btAlignedObjectArray.h"

#include "btBoxCollision.h"
#include "btTriangleShapeEx.h"





//! Overlapping pair
struct btl_GIM_PAIR
{
    int m_index1;
    int m_index2;
    btl_GIM_PAIR()
    {}

    btl_GIM_PAIR(const btl_GIM_PAIR & p)
    {
    	m_index1 = p.m_index1;
    	m_index2 = p.m_index2;
	}

	btl_GIM_PAIR(int index1, int index2)
    {
    	m_index1 = index1;
    	m_index2 = index2;
	}
};

//! A pairset array
class btlPairSet: public btlAlignedObjectArray<btl_GIM_PAIR>
{
public:
	btlPairSet()
	{
		reserve(32);
	}
	inline void push_pair(int index1,int index2)
	{
		push_back(btl_GIM_PAIR(index1,index2));
	}

	inline void push_pair_inv(int index1,int index2)
	{
		push_back(btl_GIM_PAIR(index2,index1));
	}
};


///GIM_BVH_DATA is an internal GIMPACT collision structure to contain axis aligned bounding box
struct btl_GIM_BVH_DATA
{
	btlAABB m_bound;
	int m_data;
};

//! Node Structure for trees
class btl_GIM_BVH_TREE_NODE
{
public:
	btlAABB m_bound;
protected:
	int	m_escapeIndexOrDataIndex;
public:
	btl_GIM_BVH_TREE_NODE()
	{
		m_escapeIndexOrDataIndex = 0;
	}

	SIMD_FORCE_INLINE bool isLeafNode() const
	{
		//skipindex is negative (internal node), triangleindex >=0 (leafnode)
		return (m_escapeIndexOrDataIndex>=0);
	}

	SIMD_FORCE_INLINE int getEscapeIndex() const
	{
		//btAssert(m_escapeIndexOrDataIndex < 0);
		return -m_escapeIndexOrDataIndex;
	}

	SIMD_FORCE_INLINE void setEscapeIndex(int index)
	{
		m_escapeIndexOrDataIndex = -index;
	}

	SIMD_FORCE_INLINE int getDataIndex() const
	{
		//btAssert(m_escapeIndexOrDataIndex >= 0);

		return m_escapeIndexOrDataIndex;
	}

	SIMD_FORCE_INLINE void setDataIndex(int index)
	{
		m_escapeIndexOrDataIndex = index;
	}

};


class btl_GIM_BVH_DATA_ARRAY:public btlAlignedObjectArray<btl_GIM_BVH_DATA>
{
};


class btl_GIM_BVH_TREE_NODE_ARRAY:public btlAlignedObjectArray<btl_GIM_BVH_TREE_NODE>
{
};




//! Basic Box tree structure
class btlBvhTree
{
protected:
	int m_num_nodes;
	btl_GIM_BVH_TREE_NODE_ARRAY m_node_array;
protected:
	int _sort_and_calc_splitting_index(
		btl_GIM_BVH_DATA_ARRAY & primitive_boxes,
		 int startIndex,  int endIndex, int splitAxis);

	int _calc_splitting_axis(btl_GIM_BVH_DATA_ARRAY & primitive_boxes, int startIndex,  int endIndex);

	void _build_sub_tree(btl_GIM_BVH_DATA_ARRAY & primitive_boxes, int startIndex,  int endIndex);
public:
	btlBvhTree()
	{
		m_num_nodes = 0;
	}

	//! prototype functions for box tree management
	//!@{
	void build_tree(btl_GIM_BVH_DATA_ARRAY & primitive_boxes);

	SIMD_FORCE_INLINE void clearNodes()
	{
		m_node_array.clear();
		m_num_nodes = 0;
	}

	//! node count
	SIMD_FORCE_INLINE int getNodeCount() const
	{
		return m_num_nodes;
	}

	//! tells if the node is a leaf
	SIMD_FORCE_INLINE bool isLeafNode(int nodeindex) const
	{
		return m_node_array[nodeindex].isLeafNode();
	}

	SIMD_FORCE_INLINE int getNodeData(int nodeindex) const
	{
		return m_node_array[nodeindex].getDataIndex();
	}

	SIMD_FORCE_INLINE void getNodeBound(int nodeindex, btlAABB & bound) const
	{
		bound = m_node_array[nodeindex].m_bound;
	}

	SIMD_FORCE_INLINE void setNodeBound(int nodeindex, const btlAABB & bound)
	{
		m_node_array[nodeindex].m_bound = bound;
	}

	SIMD_FORCE_INLINE int getLeftNode(int nodeindex) const
	{
		return nodeindex+1;
	}

	SIMD_FORCE_INLINE int getRightNode(int nodeindex) const
	{
		if(m_node_array[nodeindex+1].isLeafNode()) return nodeindex+2;
		return nodeindex+1 + m_node_array[nodeindex+1].getEscapeIndex();
	}

	SIMD_FORCE_INLINE int getEscapeNodeIndex(int nodeindex) const
	{
		return m_node_array[nodeindex].getEscapeIndex();
	}

	SIMD_FORCE_INLINE const btl_GIM_BVH_TREE_NODE * get_node_pointer(int index = 0) const
	{
		return &m_node_array[index];
	}

	//!@}
};


//! Prototype Base class for primitive classification
/*!
This class is a wrapper for primitive collections.
This tells relevant info for the Bounding Box set classes, which take care of space classification.
This class can manage Compound shapes and trimeshes, and if it is managing trimesh then the  Hierarchy Bounding Box classes will take advantage of primitive Vs Box overlapping tests for getting optimal results and less Per Box compairisons.
*/
class btlPrimitiveManagerBase
{
public:

	virtual ~btlPrimitiveManagerBase() {}

	//! determines if this manager consist on only triangles, which special case will be optimized
	virtual bool is_trimesh() const = 0;
	virtual int get_primitive_count() const = 0;
	virtual void get_primitive_box(int prim_index ,btlAABB & primbox) const = 0;
	//! retrieves only the points of the triangle, and the collision margin
	virtual void get_primitive_triangle(int prim_index,btlPrimitiveTriangle & triangle) const= 0;
};


//! Structure for containing Boxes
/*!
This class offers an structure for managing a box tree of primitives.
Requires a Primitive prototype (like btPrimitiveManagerBase )
*/
class btlGImpactBvh
{
protected:
	btlBvhTree m_box_tree;
	btlPrimitiveManagerBase * m_primitive_manager;

protected:
	//stackless refit
	void refit();
public:

	//! this constructor doesn't build the tree. you must call	buildSet
	btlGImpactBvh()
	{
		m_primitive_manager = NULL;
	}

	//! this constructor doesn't build the tree. you must call	buildSet
	btlGImpactBvh(btlPrimitiveManagerBase * primitive_manager)
	{
		m_primitive_manager = primitive_manager;
	}

	SIMD_FORCE_INLINE btlAABB getGlobalBox()  const
	{
		btlAABB totalbox;
		getNodeBound(0, totalbox);
		return totalbox;
	}

	SIMD_FORCE_INLINE void setPrimitiveManager(btlPrimitiveManagerBase * primitive_manager)
	{
		m_primitive_manager = primitive_manager;
	}

	SIMD_FORCE_INLINE btlPrimitiveManagerBase * getPrimitiveManager() const
	{
		return m_primitive_manager;
	}


//! node manager prototype functions
///@{

	//! this attemps to refit the box set.
	SIMD_FORCE_INLINE void update()
	{
		refit();
	}

	//! this rebuild the entire set
	void buildSet();

	//! returns the indices of the primitives in the m_primitive_manager
	bool boxQuery(const btlAABB & box, btlAlignedObjectArray<int> & collided_results) const;

	//! returns the indices of the primitives in the m_primitive_manager
	SIMD_FORCE_INLINE bool boxQueryTrans(const btlAABB & box,
		 const btlTransform & transform, btlAlignedObjectArray<int> & collided_results) const
	{
		btlAABB transbox=box;
		transbox.appy_transform(transform);
		return boxQuery(transbox,collided_results);
	}

	//! returns the indices of the primitives in the m_primitive_manager
	bool rayQuery(
		const btlVector3 & ray_dir,const btlVector3 & ray_origin ,
		btlAlignedObjectArray<int> & collided_results) const;

	//! tells if this set has hierarcht
	SIMD_FORCE_INLINE bool hasHierarchy() const
	{
		return true;
	}

	//! tells if this set is a trimesh
	SIMD_FORCE_INLINE bool isTrimesh()  const
	{
		return m_primitive_manager->is_trimesh();
	}

	//! node count
	SIMD_FORCE_INLINE int getNodeCount() const
	{
		return m_box_tree.getNodeCount();
	}

	//! tells if the node is a leaf
	SIMD_FORCE_INLINE bool isLeafNode(int nodeindex) const
	{
		return m_box_tree.isLeafNode(nodeindex);
	}

	SIMD_FORCE_INLINE int getNodeData(int nodeindex) const
	{
		return m_box_tree.getNodeData(nodeindex);
	}

	SIMD_FORCE_INLINE void getNodeBound(int nodeindex, btlAABB & bound)  const
	{
		m_box_tree.getNodeBound(nodeindex, bound);
	}

	SIMD_FORCE_INLINE void setNodeBound(int nodeindex, const btlAABB & bound)
	{
		m_box_tree.setNodeBound(nodeindex, bound);
	}


	SIMD_FORCE_INLINE int getLeftNode(int nodeindex) const
	{
		return m_box_tree.getLeftNode(nodeindex);
	}

	SIMD_FORCE_INLINE int getRightNode(int nodeindex) const
	{
		return m_box_tree.getRightNode(nodeindex);
	}

	SIMD_FORCE_INLINE int getEscapeNodeIndex(int nodeindex) const
	{
		return m_box_tree.getEscapeNodeIndex(nodeindex);
	}

	SIMD_FORCE_INLINE void getNodeTriangle(int nodeindex,btlPrimitiveTriangle & triangle) const
	{
		m_primitive_manager->get_primitive_triangle(getNodeData(nodeindex),triangle);
	}


	SIMD_FORCE_INLINE const btl_GIM_BVH_TREE_NODE * get_node_pointer(int index = 0) const
	{
		return m_box_tree.get_node_pointer(index);
	}

#ifdef TRI_COLLISION_PROFILING
	static float getAverageTreeCollisionTime();
#endif //TRI_COLLISION_PROFILING

	static void find_collision(btlGImpactBvh * boxset1, const btlTransform & trans1,
		btlGImpactBvh * boxset2, const btlTransform & trans2,
		btlPairSet & collision_pairs);
};


#endif // GIM_BOXPRUNING_H_INCLUDED
