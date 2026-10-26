
/*
Stan Melax Convex Hull Computation
Copyright (c) 2008 Stan Melax http://www.melax.com/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///includes modifications/improvements by John Ratcliff, see BringOutYourDead below.

#ifndef BT_CD_HULL_H
#define BT_CD_HULL_H

#include "btVector3.h"
#include "btAlignedObjectArray.h"

typedef btlAlignedObjectArray<unsigned int> TUIntArray;

class btl_HullResult
{
public:
	btl_HullResult(void)
	{
		mPolygons = true;
		mNumOutputVertices = 0;
		mNumFaces = 0;
		mNumIndices = 0;
	}
	bool                    mPolygons;                  // true if indices represents polygons, false indices are triangles
	unsigned int            mNumOutputVertices;         // number of vertices in the output hull
	btlAlignedObjectArray<btlVector3>	m_OutputVertices;            // array of vertices
	unsigned int            mNumFaces;                  // the number of faces produced
	unsigned int            mNumIndices;                // the total number of indices
	btlAlignedObjectArray<unsigned int>    m_Indices;                   // pointer to indices.

// If triangles, then indices are array indexes into the vertex list.
// If polygons, indices are in the form (number of points in face) (p1, p2, p3, ..) etc..
};

enum HullFlag
{
	QF_TRIANGLES         = (1<<0),             // report results as triangles, not polygons.
	QF_REVERSE_ORDER     = (1<<1),             // reverse order of the triangle indices.
	QF_DEFAULT           = QF_TRIANGLES
};


class btl_HullDesc
{
public:
	btl_HullDesc(void)
	{
		mFlags          = QF_DEFAULT;
		mVcount         = 0;
		mVertices       = 0;
		mVertexStride   = sizeof(btlVector3);
		mNormalEpsilon  = 0.001f;
		mMaxVertices	= 4096; // maximum number of points to be considered for a convex hull.
		mMaxFaces	= 4096;
	};

	btl_HullDesc(HullFlag flag,
		 unsigned int vcount,
		 const btlVector3 *vertices,
		 unsigned int stride = sizeof(btlVector3))
	{
		mFlags          = flag;
		mVcount         = vcount;
		mVertices       = vertices;
		mVertexStride   = stride;
		mNormalEpsilon  = btlScalar(0.001);
		mMaxVertices    = 4096;
	}

	bool HasHullFlag(HullFlag flag) const
	{
		if ( mFlags & flag ) return true;
		return false;
	}

	void SetHullFlag(HullFlag flag)
	{
		mFlags|=flag;
	}

	void ClearHullFlag(HullFlag flag)
	{
		mFlags&=~flag;
	}

	unsigned int      mFlags;           // flags to use when generating the convex hull.
	unsigned int      mVcount;          // number of vertices in the input point cloud
	const btlVector3  *mVertices;        // the array of vertices.
	unsigned int      mVertexStride;    // the stride of each vertex, in bytes.
	btlScalar             mNormalEpsilon;   // the epsilon for removing duplicates.  This is a normalized value, if normalized bit is on.
	unsigned int      mMaxVertices;     // maximum number of vertices to be considered for the hull!
	unsigned int      mMaxFaces;
};

enum HullError
{
	QE_OK,            // success!
	QE_FAIL           // failed.
};

class btlPlane
{
	public:
	btlVector3	normal;
	btlScalar	dist;   // distance below origin - the D from plane equasion Ax+By+Cz+D=0
			btlPlane(const btlVector3 &n,btlScalar d):normal(n),dist(d){}
			btlPlane():normal(),dist(0){}
	
};



class btl_ConvexH 
{
  public:
	class HalfEdge
	{
	  public:
		short ea;         // the other half of the edge (index into edges list)
		unsigned char v;  // the vertex at the start of this edge (index into vertices list)
		unsigned char p;  // the facet on which this edge lies (index into facets list)
		HalfEdge(){}
		HalfEdge(short _ea,unsigned char _v, unsigned char _p):ea(_ea),v(_v),p(_p){}
	};
	btl_ConvexH()
	{
	}
	~btl_ConvexH()
	{
	}
	btlAlignedObjectArray<btlVector3> vertices;
	btlAlignedObjectArray<HalfEdge> edges;
	btlAlignedObjectArray<btlPlane>  facets;
	btl_ConvexH(int vertices_size,int edges_size,int facets_size);
};


class btl_int4
{
public:
	int x,y,z,w;
	btl_int4(){};
	btl_int4(int _x,int _y, int _z,int _w){x=_x;y=_y;z=_z;w=_w;}
	const int& operator[](int i) const {return (&x)[i];}
	int& operator[](int i) {return (&x)[i];}
};

class btl_PHullResult
{
public:

	btl_PHullResult(void)
	{
		mVcount = 0;
		mIndexCount = 0;
		mFaceCount = 0;
		mVertices = 0;
	}

	unsigned int mVcount;
	unsigned int mIndexCount;
	unsigned int mFaceCount;
	btlVector3*   mVertices;
	TUIntArray m_Indices;
};



///The HullLibrary class can create a convex hull from a collection of vertices, using the ComputeHull method.
///The btShapeHull class uses this HullLibrary to create a approximate convex mesh given a general (non-polyhedral) convex shape.
class btl_HullLibrary
{

	btlAlignedObjectArray<class btlHullTriangle*> m_tris;

public:

	btlAlignedObjectArray<int> m_vertexIndexMapping;


	HullError CreateConvexHull(const btl_HullDesc& desc, // describes the input request
				   btl_HullResult&     result);        // contains the resulst
	HullError ReleaseResult(btl_HullResult &result); // release memory allocated for this result, we are done with it.

private:

	bool ComputeHull(unsigned int vcount,const btlVector3 *vertices,btl_PHullResult &result,unsigned int vlimit);

	class btlHullTriangle*	allocateTriangle(int a,int b,int c);
	void	deAllocateTriangle(btlHullTriangle*);
	void b2bfix(btlHullTriangle* s,btlHullTriangle*t);

	void removeb2b(btlHullTriangle* s,btlHullTriangle*t);

	void checkit(btlHullTriangle *t);

	btlHullTriangle* extrudable(btlScalar epsilon);

	int calchull(btlVector3 *verts,int verts_count, TUIntArray& tris_out, int &tris_count,int vlimit);

	int calchullgen(btlVector3 *verts,int verts_count, int vlimit);

	btl_int4 FindSimplex(btlVector3 *verts,int verts_count,btlAlignedObjectArray<int> &allow);

	class btl_ConvexH* ConvexHCrop(btl_ConvexH& convex,const btlPlane& slice);

	void extrude(class btlHullTriangle* t0,int v);

	btl_ConvexH* test_cube();

	//BringOutYourDead (John Ratcliff): When you create a convex hull you hand it a large input set of vertices forming a 'point cloud'. 
	//After the hull is generated it give you back a set of polygon faces which index the *original* point cloud.
	//The thing is, often times, there are many 'dead vertices' in the point cloud that are on longer referenced by the hull.
	//The routine 'BringOutYourDead' find only the referenced vertices, copies them to an new buffer, and re-indexes the hull so that it is a minimal representation.
	void BringOutYourDead(const btlVector3* verts,unsigned int vcount, btlVector3* overts,unsigned int &ocount,unsigned int* indices,unsigned indexcount);

	bool CleanupVertices(unsigned int svcount,
			     const btlVector3* svertices,
			     unsigned int stride,
			     unsigned int &vcount, // output number of vertices
			     btlVector3* vertices, // location to store the results.
			     btlScalar  normalepsilon,
			     btlVector3& scale);
};


#endif //BT_CD_HULL_H

