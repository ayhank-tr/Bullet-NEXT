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


#include "btPolyhedralContactClipping.h"
#include "BulletCollision/CollisionShapes/btConvexPolyhedron.h"

#include <float.h> //for FLT_MAX

int btl_gExpectedNbTests=0;
int btl_gActualNbTests = 0;
bool btl_gUseInternalObject = true;

// Clips a face to the back of a plane
void btlPolyhedralContactClipping::clipFace(const btlVertexArray& pVtxIn, btlVertexArray& ppVtxOut, const btlVector3& planeNormalWS,btlScalar planeEqWS)
{
	
	int ve;
	btlScalar ds, de;
	int numVerts = pVtxIn.size();
	if (numVerts < 2)
		return;

	btlVector3 firstVertex=pVtxIn[pVtxIn.size()-1];
	btlVector3 endVertex = pVtxIn[0];
	
	ds = planeNormalWS.dot(firstVertex)+planeEqWS;

	for (ve = 0; ve < numVerts; ve++)
	{
		endVertex=pVtxIn[ve];

		de = planeNormalWS.dot(endVertex)+planeEqWS;

		if (ds<0)
		{
			if (de<0)
			{
				// Start < 0, end < 0, so output endVertex
				ppVtxOut.push_back(endVertex);
			}
			else
			{
				// Start < 0, end >= 0, so output intersection
				ppVtxOut.push_back( 	firstVertex.lerp(endVertex,btlScalar(ds * 1.f/(ds - de))));
			}
		}
		else
		{
			if (de<0)
			{
				// Start >= 0, end < 0 so output intersection and end
				ppVtxOut.push_back(firstVertex.lerp(endVertex,btlScalar(ds * 1.f/(ds - de))));
				ppVtxOut.push_back(endVertex);
			}
		}
		firstVertex = endVertex;
		ds = de;
	}
}


static bool TestSepAxis(const btlConvexPolyhedron& hullA, const btlConvexPolyhedron& hullB, const btlTransform& transA,const btlTransform& transB, const btlVector3& sep_axis, btlScalar& depth, btlVector3& witnessPointA, btlVector3& witnessPointB)
{
	btlScalar Min0,Max0;
	btlScalar Min1,Max1;
	btlVector3 witnesPtMinA,witnesPtMaxA;
	btlVector3 witnesPtMinB,witnesPtMaxB;

	hullA.project(transA,sep_axis, Min0, Max0,witnesPtMinA,witnesPtMaxA);
	hullB.project(transB, sep_axis, Min1, Max1,witnesPtMinB,witnesPtMaxB);

	if(Max0<Min1 || Max1<Min0)
		return false;

	btlScalar d0 = Max0 - Min1;
	btlAssert(d0>=0.0f);
	btlScalar d1 = Max1 - Min0;
	btlAssert(d1>=0.0f);
	if (d0<d1)
	{
		depth = d0;
		witnessPointA = witnesPtMaxA;
		witnessPointB = witnesPtMinB;

	} else
	{
		depth = d1;
		witnessPointA = witnesPtMinA;
		witnessPointB = witnesPtMaxB;
	}
	
	return true;
}



static int gActualSATPairTests=0;

inline bool IsAlmostZero(const btlVector3& v)
{
	if(btlFabs(v.x())>1e-6 || btlFabs(v.y())>1e-6 || btlFabs(v.z())>1e-6)	return false;
	return true;
}

#ifdef TEST_INTERNAL_OBJECTS

inline void btl_BoxSupport(const btlScalar extents[3], const btlScalar sv[3], btlScalar p[3])
{
	// This version is ~11.000 cycles (4%) faster overall in one of the tests.
//	IR(p[0]) = IR(extents[0])|(IR(sv[0])&SIGN_BITMASK);
//	IR(p[1]) = IR(extents[1])|(IR(sv[1])&SIGN_BITMASK);
//	IR(p[2]) = IR(extents[2])|(IR(sv[2])&SIGN_BITMASK);
	p[0] = sv[0] < 0.0f ? -extents[0] : extents[0];
	p[1] = sv[1] < 0.0f ? -extents[1] : extents[1];
	p[2] = sv[2] < 0.0f ? -extents[2] : extents[2];
}

void btl_InverseTransformPoint3x3(btlVector3& out, const btlVector3& in, const btlTransform& tr)
{
	const btlMatrix3x3& rot = tr.getBasis();
	const btlVector3& r0 = rot[0];
	const btlVector3& r1 = rot[1];
	const btlVector3& r2 = rot[2];

	const btlScalar x = r0.x()*in.x() + r1.x()*in.y() + r2.x()*in.z();
	const btlScalar y = r0.y()*in.x() + r1.y()*in.y() + r2.y()*in.z();
	const btlScalar z = r0.z()*in.x() + r1.z()*in.y() + r2.z()*in.z();

	out.setValue(x, y, z);
}

 bool btl_TestInternalObjects( const btlTransform& trans0, const btlTransform& trans1, const btlVector3& delta_c, const btlVector3& axis, const btlConvexPolyhedron& convex0, const btlConvexPolyhedron& convex1, btlScalar dmin)
{
	const btlScalar dp = delta_c.dot(axis);

	btlVector3 localAxis0;
	btl_InverseTransformPoint3x3(localAxis0, axis,trans0);
	btlVector3 localAxis1;
	btl_InverseTransformPoint3x3(localAxis1, axis,trans1);

	btlScalar p0[3];
	btl_BoxSupport(convex0.m_extents, localAxis0, p0);
	btlScalar p1[3];
	btl_BoxSupport(convex1.m_extents, localAxis1, p1);

	const btlScalar Radius0 = p0[0]*localAxis0.x() + p0[1]*localAxis0.y() + p0[2]*localAxis0.z();
	const btlScalar Radius1 = p1[0]*localAxis1.x() + p1[1]*localAxis1.y() + p1[2]*localAxis1.z();

	const btlScalar MinRadius = Radius0>convex0.m_radius ? Radius0 : convex0.m_radius;
	const btlScalar MaxRadius = Radius1>convex1.m_radius ? Radius1 : convex1.m_radius;

	const btlScalar MinMaxRadius = MaxRadius + MinRadius;
	const btlScalar d0 = MinMaxRadius + dp;
	const btlScalar d1 = MinMaxRadius - dp;

	const btlScalar depth = d0<d1 ? d0:d1;
	if(depth>dmin)
		return false;
	return true;
}
#endif //TEST_INTERNAL_OBJECTS

 
 
 SIMD_FORCE_INLINE void btlSegmentsClosestPoints(
	btlVector3& ptsVector,
	btlVector3& offsetA,
	btlVector3& offsetB,
	btlScalar& tA, btlScalar& tB,
	const btlVector3& translation,
	const btlVector3& dirA, btlScalar hlenA,
	const btlVector3& dirB, btlScalar hlenB )
{
	// compute the parameters of the closest points on each line segment

	btlScalar dirA_dot_dirB = btlDot(dirA,dirB);
	btlScalar dirA_dot_trans = btlDot(dirA,translation);
	btlScalar dirB_dot_trans = btlDot(dirB,translation);

	btlScalar denom = 1.0f - dirA_dot_dirB * dirA_dot_dirB;

	if ( denom == 0.0f ) {
		tA = 0.0f;
	} else {
		tA = ( dirA_dot_trans - dirB_dot_trans * dirA_dot_dirB ) / denom;
		if ( tA < -hlenA )
			tA = -hlenA;
		else if ( tA > hlenA )
			tA = hlenA;
	}

	tB = tA * dirA_dot_dirB - dirB_dot_trans;

	if ( tB < -hlenB ) {
		tB = -hlenB;
		tA = tB * dirA_dot_dirB + dirA_dot_trans;

		if ( tA < -hlenA )
			tA = -hlenA;
		else if ( tA > hlenA )
			tA = hlenA;
	} else if ( tB > hlenB ) {
		tB = hlenB;
		tA = tB * dirA_dot_dirB + dirA_dot_trans;

		if ( tA < -hlenA )
			tA = -hlenA;
		else if ( tA > hlenA )
			tA = hlenA;
	}

	// compute the closest points relative to segment centers.

	offsetA = dirA * tA;
	offsetB = dirB * tB;

	ptsVector = translation - offsetA + offsetB;
}



bool btlPolyhedralContactClipping::findSeparatingAxis(	const btlConvexPolyhedron& hullA, const btlConvexPolyhedron& hullB, const btlTransform& transA,const btlTransform& transB, btlVector3& sep, btlDiscreteCollisionDetectorInterface::Result& resultOut)
{
	gActualSATPairTests++;

//#ifdef TEST_INTERNAL_OBJECTS
	const btlVector3 c0 = transA * hullA.m_localCenter;
	const btlVector3 c1 = transB * hullB.m_localCenter;
	const btlVector3 DeltaC2 = c0 - c1;
//#endif

	btlScalar dmin = FLT_MAX;
	int curPlaneTests=0;

	int numFacesA = hullA.m_faces.size();
	// Test normals from hullA
	for(int i=0;i<numFacesA;i++)
	{
		const btlVector3 Normal(hullA.m_faces[i].m_plane[0], hullA.m_faces[i].m_plane[1], hullA.m_faces[i].m_plane[2]);
		btlVector3 faceANormalWS = transA.getBasis() * Normal;
		if (DeltaC2.dot(faceANormalWS)<0)
			faceANormalWS*=-1.f;

		curPlaneTests++;
#ifdef TEST_INTERNAL_OBJECTS
		btl_gExpectedNbTests++;
		if(btl_gUseInternalObject && !btl_TestInternalObjects(transA,transB, DeltaC2, faceANormalWS, hullA, hullB, dmin))
			continue;
		btl_gActualNbTests++;
#endif

		btlScalar d;
		btlVector3 wA,wB;
		if(!TestSepAxis( hullA, hullB, transA,transB, faceANormalWS, d,wA,wB))
			return false;

		if(d<dmin)
		{
			dmin = d;
			sep = faceANormalWS;
		}
	}

	int numFacesB = hullB.m_faces.size();
	// Test normals from hullB
	for(int i=0;i<numFacesB;i++)
	{
		const btlVector3 Normal(hullB.m_faces[i].m_plane[0], hullB.m_faces[i].m_plane[1], hullB.m_faces[i].m_plane[2]);
		btlVector3 WorldNormal = transB.getBasis() * Normal;
		if (DeltaC2.dot(WorldNormal)<0)
			WorldNormal *=-1.f;

		curPlaneTests++;
#ifdef TEST_INTERNAL_OBJECTS
		btl_gExpectedNbTests++;
		if(btl_gUseInternalObject && !btl_TestInternalObjects(transA,transB,DeltaC2, WorldNormal, hullA, hullB, dmin))
			continue;
		btl_gActualNbTests++;
#endif

		btlScalar d;
		btlVector3 wA,wB;
		if(!TestSepAxis(hullA, hullB,transA,transB, WorldNormal,d,wA,wB))
			return false;

		if(d<dmin)
		{
			dmin = d;
			sep = WorldNormal;
		}
	}

	btlVector3 edgeAstart,edgeAend,edgeBstart,edgeBend;
	int edgeA=-1;
	int edgeB=-1;
	btlVector3 worldEdgeA;
	btlVector3 worldEdgeB;
	btlVector3 witnessPointA(0,0,0),witnessPointB(0,0,0);
	

	int curEdgeEdge = 0;
	// Test edges
	for(int e0=0;e0<hullA.m_uniqueEdges.size();e0++)
	{
		const btlVector3 edge0 = hullA.m_uniqueEdges[e0];
		const btlVector3 WorldEdge0 = transA.getBasis() * edge0;
		for(int e1=0;e1<hullB.m_uniqueEdges.size();e1++)
		{
			const btlVector3 edge1 = hullB.m_uniqueEdges[e1];
			const btlVector3 WorldEdge1 = transB.getBasis() * edge1;

			btlVector3 Cross = WorldEdge0.cross(WorldEdge1);
			curEdgeEdge++;
			if(!IsAlmostZero(Cross))
			{
				Cross = Cross.normalize();
				if (DeltaC2.dot(Cross)<0)
					Cross *= -1.f;


#ifdef TEST_INTERNAL_OBJECTS
				btl_gExpectedNbTests++;
				if(btl_gUseInternalObject && !btl_TestInternalObjects(transA,transB,DeltaC2, Cross, hullA, hullB, dmin))
					continue;
				btl_gActualNbTests++;
#endif

				btlScalar dist;
				btlVector3 wA,wB;
				if(!TestSepAxis( hullA, hullB, transA,transB, Cross, dist,wA,wB))
					return false;

				if(dist<dmin)
				{
					dmin = dist;
					sep = Cross;
					edgeA=e0;
					edgeB=e1;
					worldEdgeA = WorldEdge0;
					worldEdgeB = WorldEdge1;
					witnessPointA=wA;
					witnessPointB=wB;
				}
			}
		}

	}

	if (edgeA>=0&&edgeB>=0)
	{
//		printf("edge-edge\n");
		//add an edge-edge contact

		btlVector3 ptsVector;
		btlVector3 offsetA;
		btlVector3 offsetB;
		btlScalar tA;
		btlScalar tB;

		btlVector3 translation = witnessPointB-witnessPointA;

		btlVector3 dirA = worldEdgeA;
		btlVector3 dirB = worldEdgeB;
		
		btlScalar hlenB = 1e30f;
		btlScalar hlenA = 1e30f;

		btlSegmentsClosestPoints(ptsVector,offsetA,offsetB,tA,tB,
			translation,
			dirA, hlenA,
			dirB,hlenB);

		btlScalar nlSqrt = ptsVector.length2();
		if (nlSqrt>SIMD_EPSILON)
		{
			btlScalar nl = btlSqrt(nlSqrt);
			ptsVector *= 1.f/nl;
			if (ptsVector.dot(DeltaC2)<0.f)
			{
				ptsVector*=-1.f;
			}
			btlVector3 ptOnB = witnessPointB + offsetB;
			btlScalar distance = nl;
			resultOut.addContactPoint(ptsVector, ptOnB,-distance);
		}

	}


	if((DeltaC2.dot(sep))<0.0f)
		sep = -sep;

	return true;
}

void	btlPolyhedralContactClipping::clipFaceAgainstHull(const btlVector3& separatingNormal, const btlConvexPolyhedron& hullA,  const btlTransform& transA, btlVertexArray& worldVertsB1, const btlScalar minDist, btlScalar maxDist,btlDiscreteCollisionDetectorInterface::Result& resultOut)
{
	btlVertexArray worldVertsB2;
	btlVertexArray* pVtxIn = &worldVertsB1;
	btlVertexArray* pVtxOut = &worldVertsB2;
	pVtxOut->reserve(pVtxIn->size());

	int closestFaceA=-1;
	{
		btlScalar dmin = FLT_MAX;
		for(int face=0;face<hullA.m_faces.size();face++)
		{
			const btlVector3 Normal(hullA.m_faces[face].m_plane[0], hullA.m_faces[face].m_plane[1], hullA.m_faces[face].m_plane[2]);
			const btlVector3 faceANormalWS = transA.getBasis() * Normal;
		
			btlScalar d = faceANormalWS.dot(separatingNormal);
			if (d < dmin)
			{
				dmin = d;
				closestFaceA = face;
			}
		}
	}
	if (closestFaceA<0)
		return;

	const btlFace& polyA = hullA.m_faces[closestFaceA];

		// clip polygon to back of planes of all faces of hull A that are adjacent to witness face
	int numVerticesA = polyA.m_indices.size();
	for(int e0=0;e0<numVerticesA;e0++)
	{
		const btlVector3& a = hullA.m_vertices[polyA.m_indices[e0]];
		const btlVector3& b = hullA.m_vertices[polyA.m_indices[(e0+1)%numVerticesA]];
		const btlVector3 edge0 = a - b;
		const btlVector3 WorldEdge0 = transA.getBasis() * edge0;
		btlVector3 worldPlaneAnormal1 = transA.getBasis()* btlVector3(polyA.m_plane[0],polyA.m_plane[1],polyA.m_plane[2]);

		btlVector3 planeNormalWS1 = -WorldEdge0.cross(worldPlaneAnormal1);//.cross(WorldEdge0);
		btlVector3 worldA1 = transA*a;
		btlScalar planeEqWS1 = -worldA1.dot(planeNormalWS1);
		
//int otherFace=0;
#ifdef BLA1
		int otherFace = polyA.m_connectedFaces[e0];
		btlVector3 localPlaneNormal (hullA.m_faces[otherFace].m_plane[0],hullA.m_faces[otherFace].m_plane[1],hullA.m_faces[otherFace].m_plane[2]);
		btlScalar localPlaneEq = hullA.m_faces[otherFace].m_plane[3];

		btlVector3 planeNormalWS = transA.getBasis()*localPlaneNormal;
		btlScalar planeEqWS=localPlaneEq-planeNormalWS.dot(transA.getOrigin());
#else 
		btlVector3 planeNormalWS = planeNormalWS1;
		btlScalar planeEqWS=planeEqWS1;
		
#endif
		//clip face

		clipFace(*pVtxIn, *pVtxOut,planeNormalWS,planeEqWS);
		btlSwap(pVtxIn,pVtxOut);
		pVtxOut->resize(0);
	}



//#define ONLY_REPORT_DEEPEST_POINT

	btlVector3 point;
	

	// only keep points that are behind the witness face
	{
		btlVector3 localPlaneNormal (polyA.m_plane[0],polyA.m_plane[1],polyA.m_plane[2]);
		btlScalar localPlaneEq = polyA.m_plane[3];
		btlVector3 planeNormalWS = transA.getBasis()*localPlaneNormal;
		btlScalar planeEqWS=localPlaneEq-planeNormalWS.dot(transA.getOrigin());
		for (int i=0;i<pVtxIn->size();i++)
		{
			btlVector3 vtx = pVtxIn->at(i);
			btlScalar depth = planeNormalWS.dot(vtx)+planeEqWS;
			if (depth <=minDist)
			{
//				printf("clamped: depth=%f to minDist=%f\n",depth,minDist);
				depth = minDist;
			}

			if (depth <=maxDist)
			{
				btlVector3 point = pVtxIn->at(i);
#ifdef ONLY_REPORT_DEEPEST_POINT
				curMaxDist = depth;
#else
#if 0
				if (depth<-3)
				{
					printf("error in btPolyhedralContactClipping depth = %f\n", depth);
					printf("likely wrong separatingNormal passed in\n");
				} 
#endif				
				resultOut.addContactPoint(separatingNormal,point,depth);
#endif
			}
		}
	}
#ifdef ONLY_REPORT_DEEPEST_POINT
	if (curMaxDist<maxDist)
	{
		resultOut.addContactPoint(separatingNormal,point,curMaxDist);
	}
#endif //ONLY_REPORT_DEEPEST_POINT

}





void	btlPolyhedralContactClipping::clipHullAgainstHull(const btlVector3& separatingNormal1, const btlConvexPolyhedron& hullA, const btlConvexPolyhedron& hullB, const btlTransform& transA,const btlTransform& transB, const btlScalar minDist, btlScalar maxDist,btlDiscreteCollisionDetectorInterface::Result& resultOut)
{

	btlVector3 separatingNormal = separatingNormal1.normalized();
//	const btVector3 c0 = transA * hullA.m_localCenter;
//	const btVector3 c1 = transB * hullB.m_localCenter;
	//const btVector3 DeltaC2 = c0 - c1;



	int closestFaceB=-1;
	btlScalar dmax = -FLT_MAX;
	{
		for(int face=0;face<hullB.m_faces.size();face++)
		{
			const btlVector3 Normal(hullB.m_faces[face].m_plane[0], hullB.m_faces[face].m_plane[1], hullB.m_faces[face].m_plane[2]);
			const btlVector3 WorldNormal = transB.getBasis() * Normal;
			btlScalar d = WorldNormal.dot(separatingNormal);
			if (d > dmax)
			{
				dmax = d;
				closestFaceB = face;
			}
		}
	}
				btlVertexArray worldVertsB1;
				{
					const btlFace& polyB = hullB.m_faces[closestFaceB];
					const int numVertices = polyB.m_indices.size();
					for(int e0=0;e0<numVertices;e0++)
					{
						const btlVector3& b = hullB.m_vertices[polyB.m_indices[e0]];
						worldVertsB1.push_back(transB*b);
					}
				}

	
	if (closestFaceB>=0)
		clipFaceAgainstHull(separatingNormal, hullA, transA,worldVertsB1, minDist, maxDist,resultOut);

}
