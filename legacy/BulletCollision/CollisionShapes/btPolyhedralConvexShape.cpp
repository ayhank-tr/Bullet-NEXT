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
//#if defined (_WIN32) || defined (__i386__)
//#define BT_USE_SSE_IN_API
//#endif

#include "BulletCollision/CollisionShapes/btPolyhedralConvexShape.h"
#include "btConvexPolyhedron.h"
#include "LinearMath/btConvexHullComputer.h"
#include <new>
#include "LinearMath/btGeometryUtil.h"
#include "LinearMath/btGrahamScan2dConvexHull.h"


btlPolyhedralConvexShape::btlPolyhedralConvexShape() :btlConvexInternalShape(),
m_polyhedron(0)
{

}

btlPolyhedralConvexShape::~btlPolyhedralConvexShape()
{
	if (m_polyhedron)
	{
		m_polyhedron->~btlConvexPolyhedron();
		btlAlignedFree(m_polyhedron);
	}
}


bool	btlPolyhedralConvexShape::initializePolyhedralFeatures(int shiftVerticesByMargin)
{

	if (m_polyhedron)
	{
		m_polyhedron->~btlConvexPolyhedron();
		btlAlignedFree(m_polyhedron);
	}
	
	void* mem = btlAlignedAlloc(sizeof(btlConvexPolyhedron),16);
	m_polyhedron = new (mem) btlConvexPolyhedron;

	btlAlignedObjectArray<btlVector3> orgVertices;

	for (int i=0;i<getNumVertices();i++)
	{
		btlVector3& newVertex = orgVertices.expand();
		getVertex(i,newVertex);
	}
	
	btlConvexHullComputer conv;
	
	if (shiftVerticesByMargin)
	{
		btlAlignedObjectArray<btlVector3> planeEquations;
		btlGeometryUtil::getPlaneEquationsFromVertices(orgVertices,planeEquations);

		btlAlignedObjectArray<btlVector3> shiftedPlaneEquations;
		for (int p=0;p<planeEquations.size();p++)
		{
			   btlVector3 plane = planeEquations[p];
		//	   btScalar margin = getMargin();
			   plane[3] -= getMargin();
			   shiftedPlaneEquations.push_back(plane);
		}

		btlAlignedObjectArray<btlVector3> tmpVertices;

		btlGeometryUtil::getVerticesFromPlaneEquations(shiftedPlaneEquations,tmpVertices);
	
		conv.compute(&tmpVertices[0].getX(), sizeof(btlVector3),tmpVertices.size(),0.f,0.f);
	} else
	{
		
		conv.compute(&orgVertices[0].getX(), sizeof(btlVector3),orgVertices.size(),0.f,0.f);
	}



	btlAlignedObjectArray<btlVector3> faceNormals;
	int numFaces = conv.faces.size();
	faceNormals.resize(numFaces);
	btlConvexHullComputer* convexUtil = &conv;

	
	btlAlignedObjectArray<btlFace>	tmpFaces;
	tmpFaces.resize(numFaces);

	int numVertices = convexUtil->vertices.size();
	m_polyhedron->m_vertices.resize(numVertices);
	for (int p=0;p<numVertices;p++)
	{
		m_polyhedron->m_vertices[p] = convexUtil->vertices[p];
	}


	for (int i=0;i<numFaces;i++)
	{
		int face = convexUtil->faces[i];
		//printf("face=%d\n",face);
		const btlConvexHullComputer::Edge*  firstEdge = &convexUtil->edges[face];
		const btlConvexHullComputer::Edge*  edge = firstEdge;

		btlVector3 edges[3];
		int numEdges = 0;
		//compute face normals

		do
		{
			
			int src = edge->getSourceVertex();
			tmpFaces[i].m_indices.push_back(src);
			int targ = edge->getTargetVertex();
			btlVector3 wa = convexUtil->vertices[src];

			btlVector3 wb = convexUtil->vertices[targ];
			btlVector3 newEdge = wb-wa;
			newEdge.normalize();
			if (numEdges<2)
				edges[numEdges++] = newEdge;

			edge = edge->getNextEdgeOfFace();
		} while (edge!=firstEdge);

		btlScalar planeEq = 1e30f;

		
		if (numEdges==2)
		{
			faceNormals[i] = edges[0].cross(edges[1]);
			faceNormals[i].normalize();
			tmpFaces[i].m_plane[0] = faceNormals[i].getX();
			tmpFaces[i].m_plane[1] = faceNormals[i].getY();
			tmpFaces[i].m_plane[2] = faceNormals[i].getZ();
			tmpFaces[i].m_plane[3] = planeEq;

		}
		else
		{
			btlAssert(0);//degenerate?
			faceNormals[i].setZero();
		}

		for (int v=0;v<tmpFaces[i].m_indices.size();v++)
		{
			btlScalar eq = m_polyhedron->m_vertices[tmpFaces[i].m_indices[v]].dot(faceNormals[i]);
			if (planeEq>eq)
			{
				planeEq=eq;
			}
		}
		tmpFaces[i].m_plane[3] = -planeEq;
	}

	//merge coplanar faces and copy them to m_polyhedron

	btlScalar faceWeldThreshold= 0.999f;
	btlAlignedObjectArray<int> todoFaces;
	for (int i=0;i<tmpFaces.size();i++)
		todoFaces.push_back(i);

	while (todoFaces.size())
	{
		btlAlignedObjectArray<int> coplanarFaceGroup;
		int refFace = todoFaces[todoFaces.size()-1];

		coplanarFaceGroup.push_back(refFace);
		btlFace& faceA = tmpFaces[refFace];
		todoFaces.pop_back();

		btlVector3 faceNormalA(faceA.m_plane[0],faceA.m_plane[1],faceA.m_plane[2]);
		for (int j=todoFaces.size()-1;j>=0;j--)
		{
			int i = todoFaces[j];
			btlFace& faceB = tmpFaces[i];
			btlVector3 faceNormalB(faceB.m_plane[0],faceB.m_plane[1],faceB.m_plane[2]);
			if (faceNormalA.dot(faceNormalB)>faceWeldThreshold)
			{
				coplanarFaceGroup.push_back(i);
				todoFaces.remove(i);
			}
		}


		bool did_merge = false;
		if (coplanarFaceGroup.size()>1)
		{
			//do the merge: use Graham Scan 2d convex hull

			btlAlignedObjectArray<GrahamVector3> orgpoints;
			btlVector3 averageFaceNormal(0,0,0);

			for (int i=0;i<coplanarFaceGroup.size();i++)
			{
//				m_polyhedron->m_faces.push_back(tmpFaces[coplanarFaceGroup[i]]);

				btlFace& face = tmpFaces[coplanarFaceGroup[i]];
				btlVector3 faceNormal(face.m_plane[0],face.m_plane[1],face.m_plane[2]);
				averageFaceNormal+=faceNormal;
				for (int f=0;f<face.m_indices.size();f++)
				{
					int orgIndex = face.m_indices[f];
					btlVector3 pt = m_polyhedron->m_vertices[orgIndex];
					
					bool found = false;

					for (int i=0;i<orgpoints.size();i++)
					{
						//if ((orgpoints[i].m_orgIndex == orgIndex) || ((rotatedPt-orgpoints[i]).length2()<0.0001))
						if (orgpoints[i].m_orgIndex == orgIndex)
						{
							found=true;
							break;
						}
					}
					if (!found)
						orgpoints.push_back(GrahamVector3(pt,orgIndex));
				}
			}

			

			btlFace combinedFace;
			for (int i=0;i<4;i++)
				combinedFace.m_plane[i] = tmpFaces[coplanarFaceGroup[0]].m_plane[i];

			btlAlignedObjectArray<GrahamVector3> hull;

			averageFaceNormal.normalize();
			btl_GrahamScanConvexHull2D(orgpoints,hull,averageFaceNormal);

			for (int i=0;i<hull.size();i++)
			{
				combinedFace.m_indices.push_back(hull[i].m_orgIndex);
				for(int k = 0; k < orgpoints.size(); k++) 
				{
					if(orgpoints[k].m_orgIndex == hull[i].m_orgIndex) 
					{
						orgpoints[k].m_orgIndex = -1; // invalidate...
						break;
					}
				}
			}

			// are there rejected vertices?
			bool reject_merge = false;
			


			for(int i = 0; i < orgpoints.size(); i++) {
				if(orgpoints[i].m_orgIndex == -1)
					continue; // this is in the hull...
				// this vertex is rejected -- is anybody else using this vertex?
				for(int j = 0; j < tmpFaces.size(); j++) {
					
					btlFace& face = tmpFaces[j];
					// is this a face of the current coplanar group?
					bool is_in_current_group = false;
					for(int k = 0; k < coplanarFaceGroup.size(); k++) {
						if(coplanarFaceGroup[k] == j) {
							is_in_current_group = true;
							break;
						}
					}
					if(is_in_current_group) // ignore this face...
						continue;
					// does this face use this rejected vertex?
					for(int v = 0; v < face.m_indices.size(); v++) {
						if(face.m_indices[v] == orgpoints[i].m_orgIndex) {
							// this rejected vertex is used in another face -- reject merge
							reject_merge = true;
							break;
						}
					}
					if(reject_merge)
						break;
				}
				if(reject_merge)
					break;
			}

			if (!reject_merge)
			{
				// do this merge!
				did_merge = true;
				m_polyhedron->m_faces.push_back(combinedFace);
			}
		}
		if(!did_merge)
		{
			for (int i=0;i<coplanarFaceGroup.size();i++)
			{
				btlFace face = tmpFaces[coplanarFaceGroup[i]];
				m_polyhedron->m_faces.push_back(face);
			}

		} 



	}
	
	m_polyhedron->initialize();

	return true;
}

#ifndef MIN
    #define MIN(_a, _b)     ((_a) < (_b) ? (_a) : (_b))
#endif

btlVector3	btlPolyhedralConvexShape::localGetSupportingVertexWithoutMargin(const btlVector3& vec0)const
{


	btlVector3 supVec(0,0,0);
#ifndef __SPU__
	int i;
	btlScalar maxDot(btlScalar(-BT_LARGE_FLOAT));

	btlVector3 vec = vec0;
	btlScalar lenSqr = vec.length2();
	if (lenSqr < btlScalar(0.0001))
	{
		vec.setValue(1,0,0);
	} else
	{
		btlScalar rlen = btlScalar(1.) / btlSqrt(lenSqr );
		vec *= rlen;
	}

	btlVector3 vtx;
	btlScalar newDot;

    for( int k = 0; k < getNumVertices(); k += 128 )
    {
        btlVector3 temp[128];
        int inner_count = MIN(getNumVertices() - k, 128);
        for( i = 0; i < inner_count; i++ )
            getVertex(i,temp[i]); 
        i = (int) vec.maxDot( temp, inner_count, newDot);
		if (newDot > maxDot)
		{
			maxDot = newDot;
			supVec = temp[i];
		}        
    }
	
#endif //__SPU__
	return supVec;
}



void	btlPolyhedralConvexShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const btlVector3* vectors,btlVector3* supportVerticesOut,int numVectors) const
{
#ifndef __SPU__
	int i;

	btlVector3 vtx;
	btlScalar newDot;

	for (i=0;i<numVectors;i++)
	{
		supportVerticesOut[i][3] = btlScalar(-BT_LARGE_FLOAT);
	}

	for (int j=0;j<numVectors;j++)
	{
        const btlVector3& vec = vectors[j];
        
        for( int k = 0; k < getNumVertices(); k += 128 )
        {
            btlVector3 temp[128];
            int inner_count = MIN(getNumVertices() - k, 128);
            for( i = 0; i < inner_count; i++ )
                getVertex(i,temp[i]); 
            i = (int) vec.maxDot( temp, inner_count, newDot);
            if (newDot > supportVerticesOut[j][3])
            {
				supportVerticesOut[j] = temp[i];
				supportVerticesOut[j][3] = newDot;
            }        
        }
    }

#endif //__SPU__
}



void	btlPolyhedralConvexShape::calculateLocalInertia(btlScalar mass,btlVector3& inertia) const
{
#ifndef __SPU__
	//not yet, return box inertia

	btlScalar margin = getMargin();

	btlTransform ident;
	ident.setIdentity();
	btlVector3 aabbMin,aabbMax;
	getAabb(ident,aabbMin,aabbMax);
	btlVector3 halfExtents = (aabbMax-aabbMin)*btlScalar(0.5);

	btlScalar lx=btlScalar(2.)*(halfExtents.x()+margin);
	btlScalar ly=btlScalar(2.)*(halfExtents.y()+margin);
	btlScalar lz=btlScalar(2.)*(halfExtents.z()+margin);
	const btlScalar x2 = lx*lx;
	const btlScalar y2 = ly*ly;
	const btlScalar z2 = lz*lz;
	const btlScalar scaledmass = mass * btlScalar(0.08333333);

	inertia = scaledmass * (btlVector3(y2+z2,x2+z2,x2+y2));
#endif //__SPU__
}



void	btlPolyhedralConvexAabbCachingShape::setLocalScaling(const btlVector3& scaling)
{
	btlConvexInternalShape::setLocalScaling(scaling);
	recalcLocalAabb();
}

btlPolyhedralConvexAabbCachingShape::btlPolyhedralConvexAabbCachingShape()
:btlPolyhedralConvexShape(),
m_localAabbMin(1,1,1),
m_localAabbMax(-1,-1,-1),
m_isLocalAabbValid(false)
{
}

void btlPolyhedralConvexAabbCachingShape::getAabb(const btlTransform& trans,btlVector3& aabbMin,btlVector3& aabbMax) const
{
	getNonvirtualAabb(trans,aabbMin,aabbMax,getMargin());
}

void	btlPolyhedralConvexAabbCachingShape::recalcLocalAabb()
{
	m_isLocalAabbValid = true;
	
	#if 1
	static const btlVector3 _directions[] =
	{
		btlVector3( 1.,  0.,  0.),
		btlVector3( 0.,  1.,  0.),
		btlVector3( 0.,  0.,  1.),
		btlVector3( -1., 0.,  0.),
		btlVector3( 0., -1.,  0.),
		btlVector3( 0.,  0., -1.)
	};
	
	btlVector3 _supporting[] =
	{
		btlVector3( 0., 0., 0.),
		btlVector3( 0., 0., 0.),
		btlVector3( 0., 0., 0.),
		btlVector3( 0., 0., 0.),
		btlVector3( 0., 0., 0.),
		btlVector3( 0., 0., 0.)
	};
	
	batchedUnitVectorGetSupportingVertexWithoutMargin(_directions, _supporting, 6);
	
	for ( int i = 0; i < 3; ++i )
	{
		m_localAabbMax[i] = _supporting[i][i] + m_collisionMargin;
		m_localAabbMin[i] = _supporting[i + 3][i] - m_collisionMargin;
	}
	
	#else

	for (int i=0;i<3;i++)
	{
		btlVector3 vec(btlScalar(0.),btlScalar(0.),btlScalar(0.));
		vec[i] = btlScalar(1.);
		btlVector3 tmp = localGetSupportingVertex(vec);
		m_localAabbMax[i] = tmp[i];
		vec[i] = btlScalar(-1.);
		tmp = localGetSupportingVertex(vec);
		m_localAabbMin[i] = tmp[i];
	}
	#endif
}




