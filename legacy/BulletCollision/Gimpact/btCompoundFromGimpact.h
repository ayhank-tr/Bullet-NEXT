#ifndef BT_COMPOUND_FROM_GIMPACT
#define BT_COMPOUND_FROM_GIMPACT

#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "btGImpactShape.h"
#include "BulletCollision/NarrowPhaseCollision/btRaycastCallback.h"

struct MyCallback : public btlTriangleRaycastCallback
		{
			int	m_ignorePart;
			int	m_ignoreTriangleIndex;
			

			MyCallback(const btlVector3& from, const btlVector3& to, int ignorePart, int ignoreTriangleIndex)
			:btlTriangleRaycastCallback(from,to),
			m_ignorePart(ignorePart),
			m_ignoreTriangleIndex(ignoreTriangleIndex)
			{
				
			}
			virtual btlScalar reportHit(const btlVector3& hitNormalLocal, btlScalar hitFraction, int partId, int triangleIndex)
			{
				if (partId!=m_ignorePart || triangleIndex!=m_ignoreTriangleIndex)
				{
					if (hitFraction < m_hitFraction)
						return hitFraction;
				}

				return m_hitFraction;
			}
		};
		struct MyInternalTriangleIndexCallback :public btlInternalTriangleIndexCallback
		{
			const btlGImpactMeshShape*		m_gimpactShape;
			btlCompoundShape*			m_colShape;
			btlScalar	m_depth;

			MyInternalTriangleIndexCallback (btlCompoundShape* colShape, const btlGImpactMeshShape* meshShape, btlScalar depth)
			:m_colShape(colShape),
			m_gimpactShape(meshShape),
			m_depth(depth)
			{
			}
			
			virtual void internalProcessTriangleIndex(btlVector3* triangle,int partId,int  triangleIndex)
			{
				btlVector3 scale = m_gimpactShape->getLocalScaling();
				btlVector3 v0=triangle[0]*scale;
				btlVector3 v1=triangle[1]*scale;
				btlVector3 v2=triangle[2]*scale;
				
				btlVector3 centroid = (v0+v1+v2)/3;
				btlVector3 normal = (v1-v0).cross(v2-v0);
				normal.normalize();
				btlVector3 rayFrom = centroid;
				btlVector3 rayTo = centroid-normal*m_depth;
				
				MyCallback cb(rayFrom,rayTo,partId,triangleIndex);
				
				m_gimpactShape->processAllTrianglesRay(&cb,rayFrom, rayTo);
				if (cb.m_hitFraction<1)
				{
					rayTo.setInterpolate3(cb.m_from,cb.m_to,cb.m_hitFraction);
					//rayTo = cb.m_from;
					//rayTo = rayTo.lerp(cb.m_to,cb.m_hitFraction);
					//gDebugDraw.drawLine(tr(centroid),tr(centroid+normal),btVector3(1,0,0));
				}
				

				
				btlBU_Simplex1to4* tet = new btlBU_Simplex1to4(v0,v1,v2,rayTo);
				btlTransform ident;
				ident.setIdentity();
				m_colShape->addChildShape(ident,tet);
			}
		};
		
btlCompoundShape*	btlCreateCompoundFromGimpactShape(const btlGImpactMeshShape* gimpactMesh, btlScalar depth)
{
	btlCompoundShape* colShape = new btlCompoundShape();
		
		btlTransform tr;
		tr.setIdentity();
		
		MyInternalTriangleIndexCallback cb(colShape,gimpactMesh, depth);
		btlVector3 aabbMin,aabbMax;
		gimpactMesh->getAabb(tr,aabbMin,aabbMax);
		gimpactMesh->getMeshInterface()->InternalProcessAllTriangles(&cb,aabbMin,aabbMax);

	return colShape;	
}	

#endif //BT_COMPOUND_FROM_GIMPACT