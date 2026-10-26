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

#include "btTriangleIndexVertexArray.h"

btlTriangleIndexVertexArray::btlTriangleIndexVertexArray(int numTriangles,int* triangleIndexBase,int triangleIndexStride,int numVertices,btlScalar* vertexBase,int vertexStride)
: m_hasAabb(0)
{
	btlIndexedMesh mesh;

	mesh.m_numTriangles = numTriangles;
	mesh.m_triangleIndexBase = (const unsigned char *)triangleIndexBase;
	mesh.m_triangleIndexStride = triangleIndexStride;
	mesh.m_numVertices = numVertices;
	mesh.m_vertexBase = (const unsigned char *)vertexBase;
	mesh.m_vertexStride = vertexStride;

	addIndexedMesh(mesh);

}

btlTriangleIndexVertexArray::~btlTriangleIndexVertexArray()
{

}

void	btlTriangleIndexVertexArray::getLockedVertexIndexBase(unsigned char **vertexbase, int& numverts,PHY_ScalarType& type, int& vertexStride,unsigned char **indexbase,int & indexstride,int& numfaces,PHY_ScalarType& indicestype,int subpart)
{
	btlAssert(subpart< getNumSubParts() );

	btlIndexedMesh& mesh = m_indexedMeshes[subpart];

	numverts = mesh.m_numVertices;
	(*vertexbase) = (unsigned char *) mesh.m_vertexBase;

   type = mesh.m_vertexType;

	vertexStride = mesh.m_vertexStride;

	numfaces = mesh.m_numTriangles;

	(*indexbase) = (unsigned char *)mesh.m_triangleIndexBase;
	indexstride = mesh.m_triangleIndexStride;
	indicestype = mesh.m_indexType;
}

void	btlTriangleIndexVertexArray::getLockedReadOnlyVertexIndexBase(const unsigned char **vertexbase, int& numverts,PHY_ScalarType& type, int& vertexStride,const unsigned char **indexbase,int & indexstride,int& numfaces,PHY_ScalarType& indicestype,int subpart) const
{
	const btlIndexedMesh& mesh = m_indexedMeshes[subpart];

	numverts = mesh.m_numVertices;
	(*vertexbase) = (const unsigned char *)mesh.m_vertexBase;

   type = mesh.m_vertexType;
   
	vertexStride = mesh.m_vertexStride;

	numfaces = mesh.m_numTriangles;
	(*indexbase) = (const unsigned char *)mesh.m_triangleIndexBase;
	indexstride = mesh.m_triangleIndexStride;
	indicestype = mesh.m_indexType;
}

bool	btlTriangleIndexVertexArray::hasPremadeAabb() const
{
	return (m_hasAabb == 1);
}


void	btlTriangleIndexVertexArray::setPremadeAabb(const btlVector3& aabbMin, const btlVector3& aabbMax ) const
{
	m_aabbMin = aabbMin;
	m_aabbMax = aabbMax;
	m_hasAabb = 1; // this is intentionally an int see notes in header
}

void	btlTriangleIndexVertexArray::getPremadeAabb(btlVector3* aabbMin, btlVector3* aabbMax ) const
{
	*aabbMin = m_aabbMin;
	*aabbMax = m_aabbMax;
}


