#include "btInternalEdgeUtility.h"

#include "BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btScaledBvhTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btTriangleShape.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/NarrowPhaseCollision/btManifoldPoint.h"
#include "LinearMath/btIDebugDraw.h"
#include "BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h"

//#define DEBUG_INTERNAL_EDGE

#ifdef DEBUG_INTERNAL_EDGE
#include <stdio.h>
#endif //DEBUG_INTERNAL_EDGE


#ifdef BT_INTERNAL_EDGE_DEBUG_DRAW
static btlIDebugDraw* gDebugDrawer = 0;

void	btlSetDebugDrawer(btlIDebugDraw* debugDrawer)
{
	gDebugDrawer = debugDrawer;
}

static void    btlDebugDrawLine(const btlVector3& from,const btlVector3& to, const btlVector3& color)
{
	if (gDebugDrawer)
		gDebugDrawer->drawLine(from,to,color);
}
#endif //BT_INTERNAL_EDGE_DEBUG_DRAW


static int	btlGetHash(int partId, int triangleIndex)
{
	int hash = (partId<<(31-MAX_NUM_PARTS_IN_BITS)) | triangleIndex;
	return hash;
}



static btlScalar btlGetAngle(const btlVector3& edgeA, const btlVector3& normalA,const btlVector3& normalB)
{
	const btlVector3 refAxis0  = edgeA;
	const btlVector3 refAxis1  = normalA;
	const btlVector3 swingAxis = normalB;
	btlScalar angle = btlAtan2(swingAxis.dot(refAxis0), swingAxis.dot(refAxis1));
	return  angle;
}


struct btlConnectivityProcessor : public btlTriangleCallback
{
	int				m_partIdA;
	int				m_triangleIndexA;
	btlVector3*		m_triangleVerticesA;
	btlTriangleInfoMap*	m_triangleInfoMap;


	virtual void processTriangle(btlVector3* triangle, int partId, int triangleIndex)
	{
		//skip self-collisions
		if ((m_partIdA == partId) && (m_triangleIndexA == triangleIndex))
			return;

		//skip duplicates (disabled for now)
		//if ((m_partIdA <= partId) && (m_triangleIndexA <= triangleIndex))
		//	return;

		//search for shared vertices and edges
		int numshared = 0;
		int sharedVertsA[3]={-1,-1,-1};
		int sharedVertsB[3]={-1,-1,-1};

		///skip degenerate triangles
		btlScalar crossBSqr = ((triangle[1]-triangle[0]).cross(triangle[2]-triangle[0])).length2();
		if (crossBSqr < m_triangleInfoMap->m_equalVertexThreshold)
			return;


		btlScalar crossASqr = ((m_triangleVerticesA[1]-m_triangleVerticesA[0]).cross(m_triangleVerticesA[2]-m_triangleVerticesA[0])).length2();
		///skip degenerate triangles
		if (crossASqr< m_triangleInfoMap->m_equalVertexThreshold)
			return;

#if 0
		printf("triangle A[0]	=	(%f,%f,%f)\ntriangle A[1]	=	(%f,%f,%f)\ntriangle A[2]	=	(%f,%f,%f)\n",
			m_triangleVerticesA[0].getX(),m_triangleVerticesA[0].getY(),m_triangleVerticesA[0].getZ(),
			m_triangleVerticesA[1].getX(),m_triangleVerticesA[1].getY(),m_triangleVerticesA[1].getZ(),
			m_triangleVerticesA[2].getX(),m_triangleVerticesA[2].getY(),m_triangleVerticesA[2].getZ());

		printf("partId=%d, triangleIndex=%d\n",partId,triangleIndex);
		printf("triangle B[0]	=	(%f,%f,%f)\ntriangle B[1]	=	(%f,%f,%f)\ntriangle B[2]	=	(%f,%f,%f)\n",
			triangle[0].getX(),triangle[0].getY(),triangle[0].getZ(),
			triangle[1].getX(),triangle[1].getY(),triangle[1].getZ(),
			triangle[2].getX(),triangle[2].getY(),triangle[2].getZ());
#endif

		for (int i=0;i<3;i++)
		{
			for (int j=0;j<3;j++)
			{
				if ( (m_triangleVerticesA[i]-triangle[j]).length2() < m_triangleInfoMap->m_equalVertexThreshold)
				{
					sharedVertsA[numshared] = i;
					sharedVertsB[numshared] = j;
					numshared++;
					///degenerate case
					if(numshared >= 3)
						return;
				}
			}
			///degenerate case
			if(numshared >= 3)
				return;
		}
		switch (numshared)
		{
		case 0:
			{
				break;
			}
		case 1:
			{
				//shared vertex
				break;
			}
		case 2:
			{
				//shared edge
				//we need to make sure the edge is in the order V2V0 and not V0V2 so that the signs are correct
				if (sharedVertsA[0] == 0 && sharedVertsA[1] == 2)
				{
					sharedVertsA[0] = 2;
					sharedVertsA[1] = 0;
					int tmp = sharedVertsB[1];
					sharedVertsB[1] = sharedVertsB[0];
					sharedVertsB[0] = tmp;
				}

				int hash = btlGetHash(m_partIdA,m_triangleIndexA);

				btlTriangleInfo* info = m_triangleInfoMap->find(hash);
				if (!info)
				{
					btlTriangleInfo tmp;
					m_triangleInfoMap->insert(hash,tmp);
					info = m_triangleInfoMap->find(hash);
				}

				int sumvertsA = sharedVertsA[0]+sharedVertsA[1];
				int otherIndexA = 3-sumvertsA;

				
				btlVector3 edge(m_triangleVerticesA[sharedVertsA[1]]-m_triangleVerticesA[sharedVertsA[0]]);

				btlTriangleShape tA(m_triangleVerticesA[0],m_triangleVerticesA[1],m_triangleVerticesA[2]);
				int otherIndexB = 3-(sharedVertsB[0]+sharedVertsB[1]);

				btlTriangleShape tB(triangle[sharedVertsB[1]],triangle[sharedVertsB[0]],triangle[otherIndexB]);
				//btTriangleShape tB(triangle[0],triangle[1],triangle[2]);

				btlVector3 normalA;
				btlVector3 normalB;
				tA.calcNormal(normalA);
				tB.calcNormal(normalB);
				edge.normalize();
				btlVector3 edgeCrossA = edge.cross(normalA).normalize();

				{
					btlVector3 tmp = m_triangleVerticesA[otherIndexA]-m_triangleVerticesA[sharedVertsA[0]];
					if (edgeCrossA.dot(tmp) < 0)
					{
						edgeCrossA*=-1;
					}
				}

				btlVector3 edgeCrossB = edge.cross(normalB).normalize();

				{
					btlVector3 tmp = triangle[otherIndexB]-triangle[sharedVertsB[0]];
					if (edgeCrossB.dot(tmp) < 0)
					{
						edgeCrossB*=-1;
					}
				}

				btlScalar	angle2 = 0;
				btlScalar	ang4 = 0.f;


				btlVector3 calculatedEdge = edgeCrossA.cross(edgeCrossB);
				btlScalar len2 = calculatedEdge.length2();

				btlScalar correctedAngle(0);
				//btVector3 calculatedNormalB = normalA;
				bool isConvex = false;

				if (len2<m_triangleInfoMap->m_planarEpsilon)
				{
					angle2 = 0.f;
					ang4 = 0.f;
				} else
				{

					calculatedEdge.normalize();
					btlVector3 calculatedNormalA = calculatedEdge.cross(edgeCrossA);
					calculatedNormalA.normalize();
					angle2 = btlGetAngle(calculatedNormalA,edgeCrossA,edgeCrossB);
					ang4 = SIMD_PI-angle2;
					btlScalar dotA = normalA.dot(edgeCrossB);
					///@todo: check if we need some epsilon, due to floating point imprecision
					isConvex = (dotA<0.);

					correctedAngle = isConvex ? ang4 : -ang4;
				}

				

				
							
				//alternatively use 
				//btVector3 calculatedNormalB2 = quatRotate(orn,normalA);


				switch (sumvertsA)
				{
				case 1:
					{
						btlVector3 edge = m_triangleVerticesA[0]-m_triangleVerticesA[1];
						btlQuaternion orn(edge,-correctedAngle);
						btlVector3 computedNormalB = btl_quatRotate(orn,normalA);
						btlScalar bla = computedNormalB.dot(normalB);
						if (bla<0)
						{
							computedNormalB*=-1;
							info->m_flags |= TRI_INFO_V0V1_SWAP_NORMALB;
						}
#ifdef DEBUG_INTERNAL_EDGE
						if ((computedNormalB-normalB).length()>0.0001)
						{
							printf("warning: normals not identical\n");
						}
#endif//DEBUG_INTERNAL_EDGE

						info->m_edgeV0V1Angle = -correctedAngle;

						if (isConvex)
							info->m_flags |= TRI_INFO_V0V1_CONVEX;
						break;
					}
				case 2:
					{
						btlVector3 edge = m_triangleVerticesA[2]-m_triangleVerticesA[0];
						btlQuaternion orn(edge,-correctedAngle);
						btlVector3 computedNormalB = btl_quatRotate(orn,normalA);
						if (computedNormalB.dot(normalB)<0)
						{
							computedNormalB*=-1;
							info->m_flags |= TRI_INFO_V2V0_SWAP_NORMALB;
						}

#ifdef DEBUG_INTERNAL_EDGE
						if ((computedNormalB-normalB).length()>0.0001)
						{
							printf("warning: normals not identical\n");
						}
#endif //DEBUG_INTERNAL_EDGE
						info->m_edgeV2V0Angle = -correctedAngle;
						if (isConvex)
							info->m_flags |= TRI_INFO_V2V0_CONVEX;
						break;	
					}
				case 3:
					{
						btlVector3 edge = m_triangleVerticesA[1]-m_triangleVerticesA[2];
						btlQuaternion orn(edge,-correctedAngle);
						btlVector3 computedNormalB = btl_quatRotate(orn,normalA);
						if (computedNormalB.dot(normalB)<0)
						{
							info->m_flags |= TRI_INFO_V1V2_SWAP_NORMALB;
							computedNormalB*=-1;
						}
#ifdef DEBUG_INTERNAL_EDGE
						if ((computedNormalB-normalB).length()>0.0001)
						{
							printf("warning: normals not identical\n");
						}
#endif //DEBUG_INTERNAL_EDGE
						info->m_edgeV1V2Angle = -correctedAngle;

						if (isConvex)
							info->m_flags |= TRI_INFO_V1V2_CONVEX;
						break;
					}
				}

				break;
			}
		default:
			{
				//				printf("warning: duplicate triangle\n");
			}

		}
	}
};
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

void btlGenerateInternalEdgeInfo (btlBvhTriangleMeshShape*trimeshShape, btlTriangleInfoMap* triangleInfoMap)
{
	//the user pointer shouldn't already be used for other purposes, we intend to store connectivity info there!
	if (trimeshShape->getTriangleInfoMap())
		return;

	trimeshShape->setTriangleInfoMap(triangleInfoMap);

	btlStridingMeshInterface* meshInterface = trimeshShape->getMeshInterface();
	const btlVector3& meshScaling = meshInterface->getScaling();

	for (int partId = 0; partId< meshInterface->getNumSubParts();partId++)
	{
		const unsigned char *vertexbase = 0;
		int numverts = 0;
		PHY_ScalarType type = PHY_INTEGER;
		int stride = 0;
		const unsigned char *indexbase = 0;
		int indexstride = 0;
		int numfaces = 0;
		PHY_ScalarType indicestype = PHY_INTEGER;
		//PHY_ScalarType indexType=0;

		btlVector3 triangleVerts[3];
		meshInterface->getLockedReadOnlyVertexIndexBase(&vertexbase,numverts,	type,stride,&indexbase,indexstride,numfaces,indicestype,partId);
		btlVector3 aabbMin,aabbMax;

		for (int triangleIndex = 0 ; triangleIndex < numfaces;triangleIndex++)
		{
			unsigned int* gfxbase = (unsigned int*)(indexbase+triangleIndex*indexstride);

			for (int j=2;j>=0;j--)
			{

				int graphicsindex = indicestype==PHY_SHORT?((unsigned short*)gfxbase)[j]:gfxbase[j];
				if (type == PHY_FLOAT)
				{
					float* graphicsbase = (float*)(vertexbase+graphicsindex*stride);
					triangleVerts[j] = btlVector3(
						graphicsbase[0]*meshScaling.getX(),
						graphicsbase[1]*meshScaling.getY(),
						graphicsbase[2]*meshScaling.getZ());
				}
				else
				{
					double* graphicsbase = (double*)(vertexbase+graphicsindex*stride);
					triangleVerts[j] = btlVector3( btlScalar(graphicsbase[0]*meshScaling.getX()), btlScalar(graphicsbase[1]*meshScaling.getY()), btlScalar(graphicsbase[2]*meshScaling.getZ()));
				}
			}
			aabbMin.setValue(btlScalar(BT_LARGE_FLOAT),btlScalar(BT_LARGE_FLOAT),btlScalar(BT_LARGE_FLOAT));
			aabbMax.setValue(btlScalar(-BT_LARGE_FLOAT),btlScalar(-BT_LARGE_FLOAT),btlScalar(-BT_LARGE_FLOAT)); 
			aabbMin.setMin(triangleVerts[0]);
			aabbMax.setMax(triangleVerts[0]);
			aabbMin.setMin(triangleVerts[1]);
			aabbMax.setMax(triangleVerts[1]);
			aabbMin.setMin(triangleVerts[2]);
			aabbMax.setMax(triangleVerts[2]);

			btlConnectivityProcessor connectivityProcessor;
			connectivityProcessor.m_partIdA = partId;
			connectivityProcessor.m_triangleIndexA = triangleIndex;
			connectivityProcessor.m_triangleVerticesA = &triangleVerts[0];
			connectivityProcessor.m_triangleInfoMap  = triangleInfoMap;

			trimeshShape->processAllTriangles(&connectivityProcessor,aabbMin,aabbMax);
		}

	}

}




// Given a point and a line segment (defined by two points), compute the closest point
// in the line.  Cap the point at the endpoints of the line segment.
void btlNearestPointInLineSegment(const btlVector3 &point, const btlVector3& line0, const btlVector3& line1, btlVector3& nearestPoint)
{
	btlVector3 lineDelta     = line1 - line0;

	// Handle degenerate lines
	if ( lineDelta.fuzzyZero())
	{
		nearestPoint = line0;
	}
	else
	{
		btlScalar delta = (point-line0).dot(lineDelta) / (lineDelta).dot(lineDelta);

		// Clamp the point to conform to the segment's endpoints
		if ( delta < 0 )
			delta = 0;
		else if ( delta > 1 )
			delta = 1;

		nearestPoint = line0 + lineDelta*delta;
	}
}




bool	btlClampNormal(const btlVector3& edge,const btlVector3& tri_normal_org,const btlVector3& localContactNormalOnB, btlScalar correctedEdgeAngle, btlVector3 & clampedLocalNormal)
{
	btlVector3 tri_normal = tri_normal_org;
	//we only have a local triangle normal, not a local contact normal -> only normal in world space...
	//either compute the current angle all in local space, or all in world space

	btlVector3 edgeCross = edge.cross(tri_normal).normalize();
	btlScalar curAngle = btlGetAngle(edgeCross,tri_normal,localContactNormalOnB);

	if (correctedEdgeAngle<0)
	{
		if (curAngle < correctedEdgeAngle)
		{
			btlScalar diffAngle = correctedEdgeAngle-curAngle;
			btlQuaternion rotation(edge,diffAngle );
			clampedLocalNormal = btlMatrix3x3(rotation)*localContactNormalOnB;
			return true;
		}
	}

	if (correctedEdgeAngle>=0)
	{
		if (curAngle > correctedEdgeAngle)
		{
			btlScalar diffAngle = correctedEdgeAngle-curAngle;
			btlQuaternion rotation(edge,diffAngle );
			clampedLocalNormal = btlMatrix3x3(rotation)*localContactNormalOnB;
			return true;
		}
	}
	return false;
}



/// Changes a btManifoldPoint collision normal to the normal from the mesh.
void btlAdjustInternalEdgeContacts(btlManifoldPoint& cp, const btlCollisionObjectWrapper* colObj0Wrap,const btlCollisionObjectWrapper* colObj1Wrap, int partId0, int index0, int normalAdjustFlags)
{
	//btAssert(colObj0->getCollisionShape()->getShapeType() == TRIANGLE_SHAPE_PROXYTYPE);
	if (colObj0Wrap->getCollisionShape()->getShapeType() != TRIANGLE_SHAPE_PROXYTYPE)
		return;

	btlBvhTriangleMeshShape* trimesh = 0;
	
	if( colObj0Wrap->getCollisionObject()->getCollisionShape()->getShapeType() == SCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE )
	   trimesh = ((btlScaledBvhTriangleMeshShape*)colObj0Wrap->getCollisionObject()->getCollisionShape())->getChildShape();
   else	   
	   trimesh = (btlBvhTriangleMeshShape*)colObj0Wrap->getCollisionObject()->getCollisionShape();
	   
   	btlTriangleInfoMap* triangleInfoMapPtr = (btlTriangleInfoMap*) trimesh->getTriangleInfoMap();
	if (!triangleInfoMapPtr)
		return;

	int hash = btlGetHash(partId0,index0);


	btlTriangleInfo* info = triangleInfoMapPtr->find(hash);
	if (!info)
		return;

	btlScalar frontFacing = (normalAdjustFlags & BT_TRIANGLE_CONVEX_BACKFACE_MODE)==0? 1.f : -1.f;
	
	const btlTriangleShape* tri_shape = static_cast<const btlTriangleShape*>(colObj0Wrap->getCollisionShape());
	btlVector3 v0,v1,v2;
	tri_shape->getVertex(0,v0);
	tri_shape->getVertex(1,v1);
	tri_shape->getVertex(2,v2);

	//btVector3 center = (v0+v1+v2)*btScalar(1./3.);

	btlVector3 red(1,0,0), green(0,1,0),blue(0,0,1),white(1,1,1),black(0,0,0);
	btlVector3 tri_normal;
	tri_shape->calcNormal(tri_normal);

	//btScalar dot = tri_normal.dot(cp.m_normalWorldOnB);
	btlVector3 nearest;
	btlNearestPointInLineSegment(cp.m_localPointB,v0,v1,nearest);

	btlVector3 contact = cp.m_localPointB;
#ifdef BT_INTERNAL_EDGE_DEBUG_DRAW
	const btlTransform& tr = colObj0->getWorldTransform();
	btlDebugDrawLine(tr*nearest,tr*cp.m_localPointB,red);
#endif //BT_INTERNAL_EDGE_DEBUG_DRAW



	bool isNearEdge = false;

	int numConcaveEdgeHits = 0;
	int numConvexEdgeHits = 0;

	btlVector3 localContactNormalOnB = colObj0Wrap->getWorldTransform().getBasis().transpose() * cp.m_normalWorldOnB;
	localContactNormalOnB.normalize();//is this necessary?
	
	// Get closest edge
	int      bestedge=-1;
	btlScalar    disttobestedge=BT_LARGE_FLOAT;
	//
	// Edge 0 -> 1
	if (btlFabs(info->m_edgeV0V1Angle)< triangleInfoMapPtr->m_maxEdgeAngleThreshold)
	{	
	   btlVector3 nearest;
	   btlNearestPointInLineSegment( cp.m_localPointB, v0, v1, nearest );
	   btlScalar     len=(contact-nearest).length();
	   //
	   if( len < disttobestedge )
	   {
	      bestedge=0;
	      disttobestedge=len;
      }	      
   }	   
	// Edge 1 -> 2
	if (btlFabs(info->m_edgeV1V2Angle)< triangleInfoMapPtr->m_maxEdgeAngleThreshold)
	{	
	   btlVector3 nearest;
	   btlNearestPointInLineSegment( cp.m_localPointB, v1, v2, nearest );
	   btlScalar     len=(contact-nearest).length();
	   //
	   if( len < disttobestedge )
	   {
	      bestedge=1;
	      disttobestedge=len;
      }	      
   }	   
	// Edge 2 -> 0
	if (btlFabs(info->m_edgeV2V0Angle)< triangleInfoMapPtr->m_maxEdgeAngleThreshold)
	{	
	   btlVector3 nearest;
	   btlNearestPointInLineSegment( cp.m_localPointB, v2, v0, nearest );
	   btlScalar     len=(contact-nearest).length();
	   //
	   if( len < disttobestedge )
	   {
	      bestedge=2;
	      disttobestedge=len;
      }	      
   }   	      	
	
#ifdef BT_INTERNAL_EDGE_DEBUG_DRAW
   btlVector3 upfix=tri_normal * btlVector3(0.1f,0.1f,0.1f);
   btlDebugDrawLine(tr * v0 + upfix, tr * v1 + upfix, red );
#endif   
	if (btlFabs(info->m_edgeV0V1Angle)< triangleInfoMapPtr->m_maxEdgeAngleThreshold)
	{
#ifdef BT_INTERNAL_EDGE_DEBUG_DRAW
		btlDebugDrawLine(tr*contact,tr*(contact+cp.m_normalWorldOnB*10),black);
#endif
		btlScalar len = (contact-nearest).length();
		if(len<triangleInfoMapPtr->m_edgeDistanceThreshold)
		if( bestedge==0 )
		{
			btlVector3 edge(v0-v1);
			isNearEdge = true;

			if (info->m_edgeV0V1Angle==btlScalar(0))
			{
				numConcaveEdgeHits++;
			} else
			{

				bool isEdgeConvex = (info->m_flags & TRI_INFO_V0V1_CONVEX);
				btlScalar swapFactor = isEdgeConvex ? btlScalar(1) : btlScalar(-1);
	#ifdef BT_INTERNAL_EDGE_DEBUG_DRAW
				btlDebugDrawLine(tr*nearest,tr*(nearest+swapFactor*tri_normal*10),white);
	#endif //BT_INTERNAL_EDGE_DEBUG_DRAW

				btlVector3 nA = swapFactor * tri_normal;

				btlQuaternion orn(edge,info->m_edgeV0V1Angle);
				btlVector3 computedNormalB = btl_quatRotate(orn,tri_normal);
				if (info->m_flags & TRI_INFO_V0V1_SWAP_NORMALB)
					computedNormalB*=-1;
				btlVector3 nB = swapFactor*computedNormalB;

				btlScalar	NdotA = localContactNormalOnB.dot(nA);
				btlScalar	NdotB = localContactNormalOnB.dot(nB);
				bool backFacingNormal = (NdotA< triangleInfoMapPtr->m_convexEpsilon) && (NdotB<triangleInfoMapPtr->m_convexEpsilon);

#ifdef DEBUG_INTERNAL_EDGE
				{
					
					btlDebugDrawLine(cp.getPositionWorldOnB(),cp.getPositionWorldOnB()+tr.getBasis()*(nB*20),red);
				}
#endif //DEBUG_INTERNAL_EDGE


				if (backFacingNormal)
				{
					numConcaveEdgeHits++;
				}
				else
				{
					numConvexEdgeHits++;
					btlVector3 clampedLocalNormal;
					bool isClamped = btlClampNormal(edge,swapFactor*tri_normal,localContactNormalOnB, info->m_edgeV0V1Angle,clampedLocalNormal);
					if (isClamped)
					{
						if (((normalAdjustFlags & BT_TRIANGLE_CONVEX_DOUBLE_SIDED)!=0) || (clampedLocalNormal.dot(frontFacing*tri_normal)>0))
						{
							btlVector3 newNormal = colObj0Wrap->getWorldTransform().getBasis() * clampedLocalNormal;
							//					cp.m_distance1 = cp.m_distance1 * newNormal.dot(cp.m_normalWorldOnB);
							cp.m_normalWorldOnB = newNormal;
							// Reproject collision point along normal. (what about cp.m_distance1?)
							cp.m_positionWorldOnB = cp.m_positionWorldOnA - cp.m_normalWorldOnB * cp.m_distance1;
							cp.m_localPointB = colObj0Wrap->getWorldTransform().invXform(cp.m_positionWorldOnB);
							
						}
					}
				}
			}
		}
	}

	btlNearestPointInLineSegment(contact,v1,v2,nearest);
#ifdef BT_INTERNAL_EDGE_DEBUG_DRAW
	btlDebugDrawLine(tr*nearest,tr*cp.m_localPointB,green);
#endif //BT_INTERNAL_EDGE_DEBUG_DRAW

#ifdef BT_INTERNAL_EDGE_DEBUG_DRAW
   btlDebugDrawLine(tr * v1 + upfix, tr * v2 + upfix , green );
#endif   

	if (btlFabs(info->m_edgeV1V2Angle)< triangleInfoMapPtr->m_maxEdgeAngleThreshold)
	{
#ifdef BT_INTERNAL_EDGE_DEBUG_DRAW
		btlDebugDrawLine(tr*contact,tr*(contact+cp.m_normalWorldOnB*10),black);
#endif //BT_INTERNAL_EDGE_DEBUG_DRAW



		btlScalar len = (contact-nearest).length();
		if(len<triangleInfoMapPtr->m_edgeDistanceThreshold)
		if( bestedge==1 )
		{
			isNearEdge = true;
#ifdef BT_INTERNAL_EDGE_DEBUG_DRAW
			btlDebugDrawLine(tr*nearest,tr*(nearest+tri_normal*10),white);
#endif //BT_INTERNAL_EDGE_DEBUG_DRAW

			btlVector3 edge(v1-v2);

			isNearEdge = true;

			if (info->m_edgeV1V2Angle == btlScalar(0))
			{
				numConcaveEdgeHits++;
			} else
			{
				bool isEdgeConvex = (info->m_flags & TRI_INFO_V1V2_CONVEX)!=0;
				btlScalar swapFactor = isEdgeConvex ? btlScalar(1) : btlScalar(-1);
	#ifdef BT_INTERNAL_EDGE_DEBUG_DRAW
				btlDebugDrawLine(tr*nearest,tr*(nearest+swapFactor*tri_normal*10),white);
	#endif //BT_INTERNAL_EDGE_DEBUG_DRAW

				btlVector3 nA = swapFactor * tri_normal;
				
				btlQuaternion orn(edge,info->m_edgeV1V2Angle);
				btlVector3 computedNormalB = btl_quatRotate(orn,tri_normal);
				if (info->m_flags & TRI_INFO_V1V2_SWAP_NORMALB)
					computedNormalB*=-1;
				btlVector3 nB = swapFactor*computedNormalB;

#ifdef DEBUG_INTERNAL_EDGE
				{
					btlDebugDrawLine(cp.getPositionWorldOnB(),cp.getPositionWorldOnB()+tr.getBasis()*(nB*20),red);
				}
#endif //DEBUG_INTERNAL_EDGE


				btlScalar	NdotA = localContactNormalOnB.dot(nA);
				btlScalar	NdotB = localContactNormalOnB.dot(nB);
				bool backFacingNormal = (NdotA< triangleInfoMapPtr->m_convexEpsilon) && (NdotB<triangleInfoMapPtr->m_convexEpsilon);

				if (backFacingNormal)
				{
					numConcaveEdgeHits++;
				}
				else
				{
					numConvexEdgeHits++;
					btlVector3 localContactNormalOnB = colObj0Wrap->getWorldTransform().getBasis().transpose() * cp.m_normalWorldOnB;
					btlVector3 clampedLocalNormal;
					bool isClamped = btlClampNormal(edge,swapFactor*tri_normal,localContactNormalOnB, info->m_edgeV1V2Angle,clampedLocalNormal);
					if (isClamped)
					{
						if (((normalAdjustFlags & BT_TRIANGLE_CONVEX_DOUBLE_SIDED)!=0) || (clampedLocalNormal.dot(frontFacing*tri_normal)>0))
						{
							btlVector3 newNormal = colObj0Wrap->getWorldTransform().getBasis() * clampedLocalNormal;
							//					cp.m_distance1 = cp.m_distance1 * newNormal.dot(cp.m_normalWorldOnB);
							cp.m_normalWorldOnB = newNormal;
							// Reproject collision point along normal.
							cp.m_positionWorldOnB = cp.m_positionWorldOnA - cp.m_normalWorldOnB * cp.m_distance1;
							cp.m_localPointB = colObj0Wrap->getWorldTransform().invXform(cp.m_positionWorldOnB);
						}
					}
				}
			}
		}
	}

	btlNearestPointInLineSegment(contact,v2,v0,nearest);
#ifdef BT_INTERNAL_EDGE_DEBUG_DRAW
	btlDebugDrawLine(tr*nearest,tr*cp.m_localPointB,blue);
#endif //BT_INTERNAL_EDGE_DEBUG_DRAW
#ifdef BT_INTERNAL_EDGE_DEBUG_DRAW
   btlDebugDrawLine(tr * v2 + upfix, tr * v0 + upfix , blue );
#endif   

	if (btlFabs(info->m_edgeV2V0Angle)< triangleInfoMapPtr->m_maxEdgeAngleThreshold)
	{

#ifdef BT_INTERNAL_EDGE_DEBUG_DRAW
		btlDebugDrawLine(tr*contact,tr*(contact+cp.m_normalWorldOnB*10),black);
#endif //BT_INTERNAL_EDGE_DEBUG_DRAW

		btlScalar len = (contact-nearest).length();
		if(len<triangleInfoMapPtr->m_edgeDistanceThreshold)
		if( bestedge==2 )
		{
			isNearEdge = true;
#ifdef BT_INTERNAL_EDGE_DEBUG_DRAW
			btlDebugDrawLine(tr*nearest,tr*(nearest+tri_normal*10),white);
#endif //BT_INTERNAL_EDGE_DEBUG_DRAW

			btlVector3 edge(v2-v0);

			if (info->m_edgeV2V0Angle==btlScalar(0))
			{
				numConcaveEdgeHits++;
			} else
			{

				bool isEdgeConvex = (info->m_flags & TRI_INFO_V2V0_CONVEX)!=0;
				btlScalar swapFactor = isEdgeConvex ? btlScalar(1) : btlScalar(-1);
	#ifdef BT_INTERNAL_EDGE_DEBUG_DRAW
				btlDebugDrawLine(tr*nearest,tr*(nearest+swapFactor*tri_normal*10),white);
	#endif //BT_INTERNAL_EDGE_DEBUG_DRAW

				btlVector3 nA = swapFactor * tri_normal;
				btlQuaternion orn(edge,info->m_edgeV2V0Angle);
				btlVector3 computedNormalB = btl_quatRotate(orn,tri_normal);
				if (info->m_flags & TRI_INFO_V2V0_SWAP_NORMALB)
					computedNormalB*=-1;
				btlVector3 nB = swapFactor*computedNormalB;

#ifdef DEBUG_INTERNAL_EDGE
				{
					btlDebugDrawLine(cp.getPositionWorldOnB(),cp.getPositionWorldOnB()+tr.getBasis()*(nB*20),red);
				}
#endif //DEBUG_INTERNAL_EDGE

				btlScalar	NdotA = localContactNormalOnB.dot(nA);
				btlScalar	NdotB = localContactNormalOnB.dot(nB);
				bool backFacingNormal = (NdotA< triangleInfoMapPtr->m_convexEpsilon) && (NdotB<triangleInfoMapPtr->m_convexEpsilon);

				if (backFacingNormal)
				{
					numConcaveEdgeHits++;
				}
				else
				{
					numConvexEdgeHits++;
					//				printf("hitting convex edge\n");


					btlVector3 localContactNormalOnB = colObj0Wrap->getWorldTransform().getBasis().transpose() * cp.m_normalWorldOnB;
					btlVector3 clampedLocalNormal;
					bool isClamped = btlClampNormal(edge,swapFactor*tri_normal,localContactNormalOnB,info->m_edgeV2V0Angle,clampedLocalNormal);
					if (isClamped)
					{
						if (((normalAdjustFlags & BT_TRIANGLE_CONVEX_DOUBLE_SIDED)!=0) || (clampedLocalNormal.dot(frontFacing*tri_normal)>0))
						{
							btlVector3 newNormal = colObj0Wrap->getWorldTransform().getBasis() * clampedLocalNormal;
							//					cp.m_distance1 = cp.m_distance1 * newNormal.dot(cp.m_normalWorldOnB);
							cp.m_normalWorldOnB = newNormal;
							// Reproject collision point along normal.
							cp.m_positionWorldOnB = cp.m_positionWorldOnA - cp.m_normalWorldOnB * cp.m_distance1;
							cp.m_localPointB = colObj0Wrap->getWorldTransform().invXform(cp.m_positionWorldOnB);
						}
					}
				} 
			}
			

		}
	}

#ifdef DEBUG_INTERNAL_EDGE
	{
		btlVector3 color(0,1,1);
		btlDebugDrawLine(cp.getPositionWorldOnB(),cp.getPositionWorldOnB()+cp.m_normalWorldOnB*10,color);
	}
#endif //DEBUG_INTERNAL_EDGE

	if (isNearEdge)
	{

		if (numConcaveEdgeHits>0)
		{
			if ((normalAdjustFlags & BT_TRIANGLE_CONCAVE_DOUBLE_SIDED)!=0)
			{
				//fix tri_normal so it pointing the same direction as the current local contact normal
				if (tri_normal.dot(localContactNormalOnB) < 0)
				{
					tri_normal *= -1;
				}
				cp.m_normalWorldOnB = colObj0Wrap->getWorldTransform().getBasis()*tri_normal;
			} else
			{
				btlVector3 newNormal = tri_normal *frontFacing;
				//if the tri_normal is pointing opposite direction as the current local contact normal, skip it
				btlScalar d = newNormal.dot(localContactNormalOnB) ;
				if (d< 0)
				{
					return;
				}
				//modify the normal to be the triangle normal (or backfacing normal)
				cp.m_normalWorldOnB = colObj0Wrap->getWorldTransform().getBasis() *newNormal;
			}
						
			// Reproject collision point along normal.
			cp.m_positionWorldOnB = cp.m_positionWorldOnA - cp.m_normalWorldOnB * cp.m_distance1;
			cp.m_localPointB = colObj0Wrap->getWorldTransform().invXform(cp.m_positionWorldOnB);
		}
	}
}
