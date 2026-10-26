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

#include "btConvexTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btCollisionMargin.h"

#include "LinearMath/btQuaternion.h"
#include "BulletCollision/CollisionShapes/btStridingMeshInterface.h"


btlConvexTriangleMeshShape ::btlConvexTriangleMeshShape (btlStridingMeshInterface* meshInterface, bool calcAabb)
: btlPolyhedralConvexAabbCachingShape(), m_stridingMesh(meshInterface)
{
	m_shapeType = CONVEX_TRIANGLEMESH_SHAPE_PROXYTYPE;
	if ( calcAabb )
		recalcLocalAabb();
}




///It's not nice to have all this virtual function overhead, so perhaps we can also gather the points once
///but then we are duplicating
class btl_LocalSupportVertexCallback: public btlInternalTriangleIndexCallback
{

	btlVector3 m_supportVertexLocal;
public:

	btlScalar m_maxDot;
	btlVector3 m_supportVecLocal;

	btl_LocalSupportVertexCallback(const btlVector3& supportVecLocal)
		: m_supportVertexLocal(btlScalar(0.),btlScalar(0.),btlScalar(0.)),
		m_maxDot(btlScalar(-BT_LARGE_FLOAT)),
                m_supportVecLocal(supportVecLocal)
	{
	}

	virtual void internalProcessTriangleIndex(btlVector3* triangle,int partId,int  triangleIndex)
	{
		(void)triangleIndex;
		(void)partId;

		for (int i=0;i<3;i++)
		{
			btlScalar dot = m_supportVecLocal.dot(triangle[i]);
			if (dot > m_maxDot)
			{
				m_maxDot = dot;
				m_supportVertexLocal = triangle[i];
			}
		}
	}
	
	btlVector3	GetSupportVertexLocal()
	{
		return m_supportVertexLocal;
	}

};





btlVector3	btlConvexTriangleMeshShape::localGetSupportingVertexWithoutMargin(const btlVector3& vec0)const
{
	btlVector3 supVec(btlScalar(0.),btlScalar(0.),btlScalar(0.));

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

	btl_LocalSupportVertexCallback	supportCallback(vec);
	btlVector3 aabbMax(btlScalar(BT_LARGE_FLOAT),btlScalar(BT_LARGE_FLOAT),btlScalar(BT_LARGE_FLOAT));
	m_stridingMesh->InternalProcessAllTriangles(&supportCallback,-aabbMax,aabbMax);
	supVec = supportCallback.GetSupportVertexLocal();

	return supVec;
}

void	btlConvexTriangleMeshShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const btlVector3* vectors,btlVector3* supportVerticesOut,int numVectors) const
{
	//use 'w' component of supportVerticesOut?
	{
		for (int i=0;i<numVectors;i++)
		{
			supportVerticesOut[i][3] = btlScalar(-BT_LARGE_FLOAT);
		}
	}
	
	///@todo: could do the batch inside the callback!


	for (int j=0;j<numVectors;j++)
	{
		const btlVector3& vec = vectors[j];
		btl_LocalSupportVertexCallback	supportCallback(vec);
		btlVector3 aabbMax(btlScalar(BT_LARGE_FLOAT),btlScalar(BT_LARGE_FLOAT),btlScalar(BT_LARGE_FLOAT));
		m_stridingMesh->InternalProcessAllTriangles(&supportCallback,-aabbMax,aabbMax);
		supportVerticesOut[j] = supportCallback.GetSupportVertexLocal();
	}
	
}
	


btlVector3	btlConvexTriangleMeshShape::localGetSupportingVertex(const btlVector3& vec)const
{
	btlVector3 supVertex = localGetSupportingVertexWithoutMargin(vec);

	if ( getMargin()!=btlScalar(0.) )
	{
		btlVector3 vecnorm = vec;
		if (vecnorm .length2() < (SIMD_EPSILON*SIMD_EPSILON))
		{
			vecnorm.setValue(btlScalar(-1.),btlScalar(-1.),btlScalar(-1.));
		} 
		vecnorm.normalize();
		supVertex+= getMargin() * vecnorm;
	}
	return supVertex;
}









//currently just for debugging (drawing), perhaps future support for algebraic continuous collision detection
//Please note that you can debug-draw btConvexTriangleMeshShape with the Raytracer Demo
int	btlConvexTriangleMeshShape::getNumVertices() const
{
	//cache this?
	return 0;
	
}

int btlConvexTriangleMeshShape::getNumEdges() const
{
	return 0;
}

void btlConvexTriangleMeshShape::getEdge(int ,btlVector3& ,btlVector3& ) const
{
	btlAssert(0);	
}

void btlConvexTriangleMeshShape::getVertex(int ,btlVector3& ) const
{
	btlAssert(0);
}

int	btlConvexTriangleMeshShape::getNumPlanes() const
{
	return 0;
}

void btlConvexTriangleMeshShape::getPlane(btlVector3& ,btlVector3& ,int  ) const
{
	btlAssert(0);
}

//not yet
bool btlConvexTriangleMeshShape::isInside(const btlVector3& ,btlScalar ) const
{
	btlAssert(0);
	return false;
}



void	btlConvexTriangleMeshShape::setLocalScaling(const btlVector3& scaling)
{
	m_stridingMesh->setScaling(scaling);
	
	recalcLocalAabb();
	
}


const btlVector3& btlConvexTriangleMeshShape::getLocalScaling() const
{
	return m_stridingMesh->getScaling();
}

void btlConvexTriangleMeshShape::calculatePrincipalAxisTransform(btlTransform& principal, btlVector3& inertia, btlScalar& volume) const
{
   class CenterCallback: public btlInternalTriangleIndexCallback
   {
      bool first;
      btlVector3 ref;
      btlVector3 sum;
      btlScalar volume;

   public:

      CenterCallback() : first(true), ref(0, 0, 0), sum(0, 0, 0), volume(0)
      {
      }

      virtual void internalProcessTriangleIndex(btlVector3* triangle, int partId, int triangleIndex)
      {
         (void) triangleIndex;
         (void) partId;
         if (first)
         {
            ref = triangle[0];
            first = false;
         }
         else
         {
            btlScalar vol = btlFabs((triangle[0] - ref).triple(triangle[1] - ref, triangle[2] - ref));
            sum += (btlScalar(0.25) * vol) * ((triangle[0] + triangle[1] + triangle[2] + ref));
            volume += vol;
         }
      }
      
      btlVector3 getCenter()
      {
         return (volume > 0) ? sum / volume : ref;
      }

      btlScalar getVolume()
      {
         return volume * btlScalar(1. / 6);
      }

   };

   class InertiaCallback: public btlInternalTriangleIndexCallback
   {
      btlMatrix3x3 sum;
      btlVector3 center;

   public:

      InertiaCallback(btlVector3& center) : sum(0, 0, 0, 0, 0, 0, 0, 0, 0), center(center)
      {
      }

      virtual void internalProcessTriangleIndex(btlVector3* triangle, int partId, int triangleIndex)
      {
         (void) triangleIndex;
         (void) partId;
         btlMatrix3x3 i;
         btlVector3 a = triangle[0] - center;
         btlVector3 b = triangle[1] - center;
         btlVector3 c = triangle[2] - center;
         btlScalar volNeg = -btlFabs(a.triple(b, c)) * btlScalar(1. / 6);
         for (int j = 0; j < 3; j++)
         {
            for (int k = 0; k <= j; k++)
            {
               i[j][k] = i[k][j] = volNeg * (btlScalar(0.1) * (a[j] * a[k] + b[j] * b[k] + c[j] * c[k])
                  + btlScalar(0.05) * (a[j] * b[k] + a[k] * b[j] + a[j] * c[k] + a[k] * c[j] + b[j] * c[k] + b[k] * c[j]));
            }
         }
         btlScalar i00 = -i[0][0];
         btlScalar i11 = -i[1][1];
         btlScalar i22 = -i[2][2];
         i[0][0] = i11 + i22; 
         i[1][1] = i22 + i00; 
         i[2][2] = i00 + i11;
         sum[0] += i[0];
         sum[1] += i[1];
         sum[2] += i[2];
      }
      
      btlMatrix3x3& getInertia()
      {
         return sum;
      }

   };

   CenterCallback centerCallback;
   btlVector3 aabbMax(btlScalar(BT_LARGE_FLOAT),btlScalar(BT_LARGE_FLOAT),btlScalar(BT_LARGE_FLOAT));
   m_stridingMesh->InternalProcessAllTriangles(&centerCallback, -aabbMax, aabbMax);
   btlVector3 center = centerCallback.getCenter();
   principal.setOrigin(center);
   volume = centerCallback.getVolume();

   InertiaCallback inertiaCallback(center);
   m_stridingMesh->InternalProcessAllTriangles(&inertiaCallback, -aabbMax, aabbMax);

   btlMatrix3x3& i = inertiaCallback.getInertia();
   i.diagonalize(principal.getBasis(), btlScalar(0.00001), 20);
   inertia.setValue(i[0][0], i[1][1], i[2][2]);
   inertia /= volume;
}

