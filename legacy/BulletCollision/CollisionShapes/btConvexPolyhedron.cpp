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
///Separating axis rest based on work from Pierre Terdiman, see
///And contact clipping based on work from Simon Hobbs

#include "btConvexPolyhedron.h"
#include "LinearMath/btHashMap.h"


btlConvexPolyhedron::btlConvexPolyhedron()
{

}
btlConvexPolyhedron::~btlConvexPolyhedron()
{

}


inline bool IsAlmostZero(const btlVector3& v)
{
	if(btlFabs(v.x())>1e-6 || btlFabs(v.y())>1e-6 || btlFabs(v.z())>1e-6)	return false;
	return true;
}

struct btlInternalVertexPair
{
	btlInternalVertexPair(short int v0,short int v1)
		:m_v0(v0),
		m_v1(v1)
	{
		if (m_v1>m_v0)
			btlSwap(m_v0,m_v1);
	}
	short int m_v0;
	short int m_v1;
	int getHash() const
	{
		return m_v0+(m_v1<<16);
	}
	bool equals(const btlInternalVertexPair& other) const
	{
		return m_v0==other.m_v0 && m_v1==other.m_v1;
	}
};

struct btlInternalEdge
{
	btlInternalEdge()
		:m_face0(-1),
		m_face1(-1)
	{
	}
	short int m_face0;
	short int m_face1;
};

//

#ifdef TEST_INTERNAL_OBJECTS
bool btlConvexPolyhedron::testContainment() const
{
	for(int p=0;p<8;p++)
	{
		btlVector3 LocalPt;
		if(p==0)		LocalPt = m_localCenter + btlVector3(m_extents[0], m_extents[1], m_extents[2]);
		else if(p==1)	LocalPt = m_localCenter + btlVector3(m_extents[0], m_extents[1], -m_extents[2]);
		else if(p==2)	LocalPt = m_localCenter + btlVector3(m_extents[0], -m_extents[1], m_extents[2]);
		else if(p==3)	LocalPt = m_localCenter + btlVector3(m_extents[0], -m_extents[1], -m_extents[2]);
		else if(p==4)	LocalPt = m_localCenter + btlVector3(-m_extents[0], m_extents[1], m_extents[2]);
		else if(p==5)	LocalPt = m_localCenter + btlVector3(-m_extents[0], m_extents[1], -m_extents[2]);
		else if(p==6)	LocalPt = m_localCenter + btlVector3(-m_extents[0], -m_extents[1], m_extents[2]);
		else if(p==7)	LocalPt = m_localCenter + btlVector3(-m_extents[0], -m_extents[1], -m_extents[2]);

		for(int i=0;i<m_faces.size();i++)
		{
			const btlVector3 Normal(m_faces[i].m_plane[0], m_faces[i].m_plane[1], m_faces[i].m_plane[2]);
			const btlScalar d = LocalPt.dot(Normal) + m_faces[i].m_plane[3];
			if(d>0.0f)
				return false;
		}
	}
	return true;
}
#endif

void	btlConvexPolyhedron::initialize()
{

	btlHashMap<btlInternalVertexPair,btlInternalEdge> edges;

	btlScalar TotalArea = 0.0f;
	
	m_localCenter.setValue(0, 0, 0);
	for(int i=0;i<m_faces.size();i++)
	{
		int numVertices = m_faces[i].m_indices.size();
		int NbTris = numVertices;
		for(int j=0;j<NbTris;j++)
		{
			int k = (j+1)%numVertices;
			btlInternalVertexPair vp(m_faces[i].m_indices[j],m_faces[i].m_indices[k]);
			btlInternalEdge* edptr = edges.find(vp);
			btlVector3 edge = m_vertices[vp.m_v1]-m_vertices[vp.m_v0];
			edge.normalize();

			bool found = false;

			for (int p=0;p<m_uniqueEdges.size();p++)
			{
				
				if (IsAlmostZero(m_uniqueEdges[p]-edge) || 
					IsAlmostZero(m_uniqueEdges[p]+edge))
				{
					found = true;
					break;
				}
			}

			if (!found)
			{
				m_uniqueEdges.push_back(edge);
			}

			if (edptr)
			{
				btlAssert(edptr->m_face0>=0);
				btlAssert(edptr->m_face1<0);
				edptr->m_face1 = i;
			} else
			{
				btlInternalEdge ed;
				ed.m_face0 = i;
				edges.insert(vp,ed);
			}
		}
	}

#ifdef USE_CONNECTED_FACES
	for(int i=0;i<m_faces.size();i++)
	{
		int numVertices = m_faces[i].m_indices.size();
		m_faces[i].m_connectedFaces.resize(numVertices);

		for(int j=0;j<numVertices;j++)
		{
			int k = (j+1)%numVertices;
			btlInternalVertexPair vp(m_faces[i].m_indices[j],m_faces[i].m_indices[k]);
			btlInternalEdge* edptr = edges.find(vp);
			btlAssert(edptr);
			btlAssert(edptr->m_face0>=0);
			btlAssert(edptr->m_face1>=0);

			int connectedFace = (edptr->m_face0==i)?edptr->m_face1:edptr->m_face0;
			m_faces[i].m_connectedFaces[j] = connectedFace;
		}
	}
#endif//USE_CONNECTED_FACES

	for(int i=0;i<m_faces.size();i++)
	{
		int numVertices = m_faces[i].m_indices.size();
		int NbTris = numVertices-2;
		
		const btlVector3& p0 = m_vertices[m_faces[i].m_indices[0]];
		for(int j=1;j<=NbTris;j++)
		{
			int k = (j+1)%numVertices;
			const btlVector3& p1 = m_vertices[m_faces[i].m_indices[j]];
			const btlVector3& p2 = m_vertices[m_faces[i].m_indices[k]];
			btlScalar Area = ((p0 - p1).cross(p0 - p2)).length() * 0.5f;
			btlVector3 Center = (p0+p1+p2)/3.0f;
			m_localCenter += Area * Center;
			TotalArea += Area;
		}
	}
	m_localCenter /= TotalArea;




#ifdef TEST_INTERNAL_OBJECTS
	if(1)
	{
		m_radius = FLT_MAX;
		for(int i=0;i<m_faces.size();i++)
		{
			const btlVector3 Normal(m_faces[i].m_plane[0], m_faces[i].m_plane[1], m_faces[i].m_plane[2]);
			const btlScalar dist = btlFabs(m_localCenter.dot(Normal) + m_faces[i].m_plane[3]);
			if(dist<m_radius)
				m_radius = dist;
		}

	
		btlScalar MinX = FLT_MAX;
		btlScalar MinY = FLT_MAX;
		btlScalar MinZ = FLT_MAX;
		btlScalar MaxX = -FLT_MAX;
		btlScalar MaxY = -FLT_MAX;
		btlScalar MaxZ = -FLT_MAX;
		for(int i=0; i<m_vertices.size(); i++)
		{
			const btlVector3& pt = m_vertices[i];
			if(pt.x()<MinX)	MinX = pt.x();
			if(pt.x()>MaxX)	MaxX = pt.x();
			if(pt.y()<MinY)	MinY = pt.y();
			if(pt.y()>MaxY)	MaxY = pt.y();
			if(pt.z()<MinZ)	MinZ = pt.z();
			if(pt.z()>MaxZ)	MaxZ = pt.z();
		}
		mC.setValue(MaxX+MinX, MaxY+MinY, MaxZ+MinZ);
		mE.setValue(MaxX-MinX, MaxY-MinY, MaxZ-MinZ);



//		const btScalar r = m_radius / sqrtf(2.0f);
		const btlScalar r = m_radius / sqrtf(3.0f);
		const int LargestExtent = mE.maxAxis();
		const btlScalar Step = (mE[LargestExtent]*0.5f - r)/1024.0f;
		m_extents[0] = m_extents[1] = m_extents[2] = r;
		m_extents[LargestExtent] = mE[LargestExtent]*0.5f;
		bool FoundBox = false;
		for(int j=0;j<1024;j++)
		{
			if(testContainment())
			{
				FoundBox = true;
				break;
			}

			m_extents[LargestExtent] -= Step;
		}
		if(!FoundBox)
		{
			m_extents[0] = m_extents[1] = m_extents[2] = r;
		}
		else
		{
			// Refine the box
			const btlScalar Step = (m_radius - r)/1024.0f;
			const int e0 = (1<<LargestExtent) & 3;
			const int e1 = (1<<e0) & 3;

			for(int j=0;j<1024;j++)
			{
				const btlScalar Saved0 = m_extents[e0];
				const btlScalar Saved1 = m_extents[e1];
				m_extents[e0] += Step;
				m_extents[e1] += Step;

				if(!testContainment())
				{
					m_extents[e0] = Saved0;
					m_extents[e1] = Saved1;
					break;
				}
			}
		}
	}
#endif
}

void btlConvexPolyhedron::project(const btlTransform& trans, const btlVector3& dir, btlScalar& minProj, btlScalar& maxProj, btlVector3& witnesPtMin,btlVector3& witnesPtMax) const
{
	minProj = FLT_MAX;
	maxProj = -FLT_MAX;
	int numVerts = m_vertices.size();
	for(int i=0;i<numVerts;i++)
	{
		btlVector3 pt = trans * m_vertices[i];
		btlScalar dp = pt.dot(dir);
		if(dp < minProj)
		{
			minProj = dp;
			witnesPtMin = pt;
		}
		if(dp > maxProj)
		{
			maxProj = dp;
			witnesPtMax = pt;
		}
	}
	if(minProj>maxProj)
	{
		btlSwap(minProj,maxProj);
		btlSwap(witnesPtMin,witnesPtMax);
	}
}
