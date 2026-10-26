/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2011 Advanced Micro Devices, Inc.  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


///This file was written by Erwin Coumans


#ifndef _BT_POLYHEDRAL_FEATURES_H
#define _BT_POLYHEDRAL_FEATURES_H

#include "LinearMath/btTransform.h"
#include "LinearMath/btAlignedObjectArray.h"

#define TEST_INTERNAL_OBJECTS 1


struct btlFace
{
	btlAlignedObjectArray<int>	m_indices;
//	btAlignedObjectArray<int>	m_connectedFaces;
	btlScalar	m_plane[4];
};


ATTRIBUTE_ALIGNED16(class) btlConvexPolyhedron
{
	public:
		
	BT_DECLARE_ALIGNED_ALLOCATOR();
		
	btlConvexPolyhedron();
	virtual	~btlConvexPolyhedron();

	btlAlignedObjectArray<btlVector3>	m_vertices;
	btlAlignedObjectArray<btlFace>	m_faces;
	btlAlignedObjectArray<btlVector3> m_uniqueEdges;

	btlVector3		m_localCenter;
	btlVector3		m_extents;
	btlScalar		m_radius;
	btlVector3		mC;
	btlVector3		mE;

	void	initialize();
	bool testContainment() const;

	void project(const btlTransform& trans, const btlVector3& dir, btlScalar& minProj, btlScalar& maxProj, btlVector3& witnesPtMin,btlVector3& witnesPtMax) const;
};

	
#endif //_BT_POLYHEDRAL_FEATURES_H


